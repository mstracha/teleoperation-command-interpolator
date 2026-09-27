#include "teleoperation/RealTimeHarness.hpp"

#include "teleoperation/CommandInterpolator.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#endif

namespace teleoperation
{
    namespace
    {
        constexpr std::size_t TelemetryDrainBatchSize = 128;

        enum class DeadlineWaitResult : std::uint8_t
        {
            DeadlineReached,
            StopRequested
        };

        class ControllerDeadlineWaiter
        {
          public:
            ControllerDeadlineWaiter(std::condition_variable &wake_condition,
                                     std::mutex &wake_mutex,
                                     const ShutdownCoordinator &shutdown_coordinator,
                                     ControllerWaitPreference wait_preference) noexcept
                : wake_condition_(wake_condition), wake_mutex_(wake_mutex),
                  shutdown_coordinator_(shutdown_coordinator)
            {
#ifdef _WIN32
                if (wait_preference != ControllerWaitPreference::PortableConditionVariable)
                {
                    timer_ = CreateWaitableTimerExW(nullptr, nullptr,
                                                    CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                                    TIMER_MODIFY_STATE | SYNCHRONIZE);

                    stop_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
                    use_high_resolution_timer_ = timer_ != nullptr && stop_event_ != nullptr;
                }
#else
                static_cast<void>(wait_preference);
#endif
            }

            ~ControllerDeadlineWaiter()
            {
#ifdef _WIN32
                if (timer_ != nullptr)
                {
                    CloseHandle(timer_);
                }

                if (stop_event_ != nullptr)
                {
                    CloseHandle(stop_event_);
                }
#endif
            }

            ControllerDeadlineWaiter(const ControllerDeadlineWaiter &) = delete;
            ControllerDeadlineWaiter &operator=(const ControllerDeadlineWaiter &) = delete;

            ControllerWaitStrategy strategy() const noexcept
            {
#ifdef _WIN32
                if (use_high_resolution_timer_)
                {
                    return ControllerWaitStrategy::WindowsHighResolutionWaitableTimer;
                }
#endif

                return ControllerWaitStrategy::PortableConditionVariable;
            }

            DeadlineWaitResult waitUntil(ControllerClock::time_point deadline) noexcept
            {
                if (shutdown_coordinator_.stopRequested())
                {
                    return DeadlineWaitResult::StopRequested;
                }

#ifdef _WIN32
                if (use_high_resolution_timer_)
                {
                    const ControllerClock::time_point now = ControllerClock::now();

                    if (deadline > now)
                    {
                        constexpr std::int64_t NanosecondsPerTimerTick = 100;
                        const std::int64_t remaining_nanoseconds =
                            std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now)
                                .count();

                        const std::int64_t timer_ticks = std::max<std::int64_t>(
                            1, remaining_nanoseconds / NanosecondsPerTimerTick +
                                   (remaining_nanoseconds % NanosecondsPerTimerTick != 0 ? 1 : 0));

                        LARGE_INTEGER due_time{};
                        due_time.QuadPart = -timer_ticks;

                        if (SetWaitableTimer(timer_, &due_time, 0, nullptr, nullptr, FALSE) !=
                            FALSE)
                        {
                            const HANDLE wait_handles[] = {stop_event_, timer_};
                            const DWORD wait_result =
                                WaitForMultipleObjects(2, wait_handles, FALSE, INFINITE);

                            if (wait_result == WAIT_OBJECT_0)
                            {
                                return DeadlineWaitResult::StopRequested;
                            }

                            if (wait_result == WAIT_OBJECT_0 + 1)
                            {
                                return DeadlineWaitResult::DeadlineReached;
                            }
                        }

                        // Preserve functional behavior if the platform timer becomes unavailable.
                        // The result records the fallback so reports never claim the timer was
                        // used.
                        use_high_resolution_timer_ = false;
                    }
                    else
                    {
                        return DeadlineWaitResult::DeadlineReached;
                    }
                }
#endif

                std::unique_lock<std::mutex> lock(wake_mutex_);
                wake_condition_.wait_until(lock, deadline,
                                           [&]() { return shutdown_coordinator_.stopRequested(); });

                return shutdown_coordinator_.stopRequested() ? DeadlineWaitResult::StopRequested
                                                             : DeadlineWaitResult::DeadlineReached;
            }

            void signalStop() noexcept
            {
#ifdef _WIN32
                if (stop_event_ != nullptr)
                {
                    static_cast<void>(SetEvent(stop_event_));
                }
#endif

                wake_condition_.notify_all();
            }

          private:
            std::condition_variable &wake_condition_;
            std::mutex &wake_mutex_;
            const ShutdownCoordinator &shutdown_coordinator_;

#ifdef _WIN32
            HANDLE timer_{nullptr};
            HANDLE stop_event_{nullptr};
            bool use_high_resolution_timer_{false};
#endif
        };

        void drainTelemetry(CommandInterpolator &interpolator,
                            std::vector<InputTelemetrySample> &input_telemetry,
                            std::vector<ControlTelemetrySample> &control_telemetry)
        {
            std::array<InputTelemetrySample, TelemetryDrainBatchSize> input_batch{};
            std::array<ControlTelemetrySample, TelemetryDrainBatchSize> control_batch{};

            for (;;)
            {
                const std::size_t count = interpolator.drainInputTelemetry(input_batch);

                input_telemetry.insert(input_telemetry.end(), input_batch.begin(),
                                       input_batch.begin() + count);

                if (count < input_batch.size())
                {
                    break;
                }
            }

            for (;;)
            {
                const std::size_t count = interpolator.drainControlTelemetry(control_batch);

                control_telemetry.insert(control_telemetry.end(), control_batch.begin(),
                                         control_batch.begin() + count);

                if (count < control_batch.size())
                {
                    break;
                }
            }
        }

        ControllerTime toControllerTime(ControllerClock::time_point time) noexcept
        {
            return ControllerTime{
                std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch())
                    .count()};
        }

        ControllerAffinityResult applyControllerAffinity(
            const std::optional<std::size_t> logical_processor) noexcept
        {
            if (!logical_processor.has_value())
            {
                return ControllerAffinityResult::NotRequested;
            }

#ifdef _WIN32
            constexpr std::size_t AffinityMaskBitCount = sizeof(DWORD_PTR) * CHAR_BIT;

            if (*logical_processor >= AffinityMaskBitCount)
            {
                return ControllerAffinityResult::InvalidLogicalProcessor;
            }

            const DWORD_PTR affinity_mask = static_cast<DWORD_PTR>(1) << *logical_processor;

            return SetThreadAffinityMask(GetCurrentThread(), affinity_mask) != 0
                       ? ControllerAffinityResult::Applied
                       : ControllerAffinityResult::ApplicationFailed;
#else
            return ControllerAffinityResult::Unsupported;
#endif
        }

        struct ControllerLatenessPercentiles
        {
            std::chrono::nanoseconds p50{0};
            std::chrono::nanoseconds p95{0};
            std::chrono::nanoseconds p99{0};
        };

        ControllerLatenessPercentiles calculateLatenessPercentiles(
            const std::vector<ControllerTimingSample> &samples)
        {
            if (samples.empty())
            {
                return {};
            }

            std::vector<std::chrono::nanoseconds> sorted_lateness;
            sorted_lateness.reserve(samples.size());

            for (const ControllerTimingSample &sample : samples)
            {
                sorted_lateness.push_back(sample.lateness);
            }

            std::sort(sorted_lateness.begin(), sorted_lateness.end());

            const auto percentile = [&](std::size_t percentage)
            {
                const std::size_t rank =
                    (sorted_lateness.size() * percentage + 99) / 100;
                return sorted_lateness[rank - 1];
            };

            return {percentile(50), percentile(95), percentile(99)};
        }
    }

    RealTimeHarness::RealTimeHarness(const JointLimits &joint_limits,
                                     const InterpolatorConfiguration &configuration)
        : joint_limits_(joint_limits), configuration_(configuration)
    {
        if (!configuration_.isValid())
        {
            throw std::invalid_argument("Invalid real-time configuration");
        }
    }

    RealTimeRunResult RealTimeHarness::run(const RealTimeScenario &scenario) const
    {
        RealTimeRunResult result{};
        result.configured_run_duration = scenario.run_duration;
        result.configured_controller_period = scenario.controller_period;
        result.configured_logger_poll_period = scenario.logger_poll_period;
        result.configured_startup_delay = scenario.startup_delay;
        result.configured_controller_wait_preference = scenario.controller_wait_preference;
        result.requested_controller_logical_processor =
            scenario.controller_logical_processor;
        result.scheduled_command_count = scenario.commands.size();

        if (scenario.run_duration.count() <= 0)
        {
            result.outcome = RealTimeRunOutcome::InvalidRunDuration;
            return result;
        }

        if (scenario.controller_period.count() <= 0)
        {
            result.outcome = RealTimeRunOutcome::InvalidControllerPeriod;
            return result;
        }

        if (scenario.logger_poll_period.count() <= 0)
        {
            result.outcome = RealTimeRunOutcome::InvalidLoggerPeriod;
            return result;
        }

        if (scenario.startup_delay.count() < 0)
        {
            result.outcome = RealTimeRunOutcome::InvalidStartupDelay;
            return result;
        }

        for (std::size_t command_index = 0; command_index < scenario.commands.size();
             ++command_index)
        {
            if (scenario.commands[command_index].delivery_offset.count() < 0)
            {
                result.outcome = RealTimeRunOutcome::DeliveryOffsetsOutOfOrder;
                return result;
            }

            if (command_index > 0 && scenario.commands[command_index].delivery_offset <
                                         scenario.commands[command_index - 1].delivery_offset)
            {
                result.outcome = RealTimeRunOutcome::DeliveryOffsetsOutOfOrder;
                return result;
            }
        }

        if (scenario.maximum_controller_ticks == 0)
        {
            result.outcome = RealTimeRunOutcome::ControllerTickLimitExceeded;
            return result;
        }

        const std::uint64_t maximum_intervals = static_cast<std::uint64_t>(
            scenario.run_duration.count() / scenario.controller_period.count());

        if (maximum_intervals >= static_cast<std::uint64_t>(scenario.maximum_controller_ticks))
        {
            result.outcome = RealTimeRunOutcome::ControllerTickLimitExceeded;
            return result;
        }

        CommandInterpolator interpolator(joint_limits_, configuration_);
        ShutdownCoordinator shutdown_coordinator;
        std::condition_variable wake_condition;
        std::mutex wake_mutex;
        ControllerDeadlineWaiter controller_deadline_waiter(wake_condition, wake_mutex,
                                                            shutdown_coordinator,
                                                            scenario.controller_wait_preference);

        std::size_t delivered_command_count = 0;
        std::size_t controller_tick_count = 0;
        std::size_t skipped_controller_period_count = 0;
        ControllerAffinityResult controller_affinity_result =
            ControllerAffinityResult::NotRequested;
        std::chrono::nanoseconds maximum_controller_lateness{0};
        std::chrono::nanoseconds maximum_control_call_duration{0};
        InterpolatorState final_state = InterpolatorState::Buffering;
        std::vector<InputTelemetrySample> input_telemetry;
        std::vector<ControlTelemetrySample> control_telemetry;
        std::vector<ControllerTimingSample> controller_timing;

        input_telemetry.reserve(scenario.commands.size());
        control_telemetry.reserve(static_cast<std::size_t>(maximum_intervals + 1));
        controller_timing.reserve(static_cast<std::size_t>(maximum_intervals + 1));

        const ControllerClock::time_point run_begin = ControllerClock::now();
        const ControllerClock::time_point scheduled_start = run_begin + scenario.startup_delay;
        const ControllerClock::time_point scheduled_end = scheduled_start + scenario.run_duration;

        std::thread logger_thread(
            [&]()
            {
                for (;;)
                {
                    drainTelemetry(interpolator, input_telemetry, control_telemetry);

                    if (shutdown_coordinator.stopRequested())
                    {
                        static_cast<void>(shutdown_coordinator.beginLogDrain());

                        if (shutdown_coordinator.producersStopped())
                        {
                            drainTelemetry(interpolator, input_telemetry, control_telemetry);

                            static_cast<void>(shutdown_coordinator.markLoggingComplete());

                            if (shutdown_coordinator.phase() == ShutdownPhase::Complete)
                            {
                                break;
                            }
                        }
                    }

                    std::unique_lock<std::mutex> lock(wake_mutex);
                    wake_condition.wait_for(lock, scenario.logger_poll_period);
                }
            });

        std::thread network_thread(
            [&]()
            {
                for (const RealTimeScheduledCommand &scheduled_command : scenario.commands)
                {
                    const ControllerClock::time_point delivery_deadline =
                        scheduled_start + scheduled_command.delivery_offset;

                    std::unique_lock<std::mutex> lock(wake_mutex);
                    wake_condition.wait_until(lock, delivery_deadline, [&]()
                                              { return shutdown_coordinator.stopRequested(); });

                    if (shutdown_coordinator.stopRequested())
                    {
                        break;
                    }

                    lock.unlock();
                    const ControllerTime receive_time = toControllerTime(ControllerClock::now());

                    static_cast<void>(
                        interpolator.submitCommand(scheduled_command.command, receive_time));

                    ++delivered_command_count;
                    wake_condition.notify_all();
                }

                shutdown_coordinator.markNetworkStopped();
                wake_condition.notify_all();
            });

        std::thread controller_thread(
            [&]()
            {
                JointPositions target_positions{};
                ControllerClock::time_point next_tick = scheduled_start;
                controller_affinity_result =
                    applyControllerAffinity(scenario.controller_logical_processor);

                for (;;)
                {
                    if (controller_deadline_waiter.waitUntil(next_tick) ==
                        DeadlineWaitResult::StopRequested)
                    {
                        break;
                    }

                    const ControllerClock::time_point call_start = ControllerClock::now();

                    if (call_start > scheduled_end)
                    {
                        static_cast<void>(shutdown_coordinator.requestStop());
                        controller_deadline_waiter.signalStop();
                        break;
                    }

                    const std::chrono::nanoseconds controller_lateness =
                        call_start > next_tick
                            ? std::chrono::duration_cast<std::chrono::nanoseconds>(call_start -
                                                                                   next_tick)
                            : std::chrono::nanoseconds{0};

                    maximum_controller_lateness =
                        std::max(maximum_controller_lateness, controller_lateness);

                    const ControlResult control_result =
                        interpolator.updateTarget(toControllerTime(call_start), target_positions);

                    const ControllerClock::time_point call_end = ControllerClock::now();

                    maximum_control_call_duration =
                        std::max(maximum_control_call_duration,
                                 std::chrono::duration_cast<std::chrono::nanoseconds>(call_end -
                                                                                      call_start));

                    controller_timing.push_back(ControllerTimingSample{
                        std::chrono::duration_cast<std::chrono::nanoseconds>(next_tick -
                                                                             scheduled_start),
                        std::chrono::duration_cast<std::chrono::nanoseconds>(call_start -
                                                                             scheduled_start),
                        controller_lateness,
                        std::chrono::duration_cast<std::chrono::nanoseconds>(call_end -
                                                                             call_start)});

                    ++controller_tick_count;
                    wake_condition.notify_all();

                    if (control_result == ControlResult::Faulted)
                    {
                        static_cast<void>(
                            shutdown_coordinator.requestStop(interpolator.faultReason()));

                        controller_deadline_waiter.signalStop();
                        break;
                    }

                    next_tick += scenario.controller_period;

                    while (next_tick <= call_end && next_tick <= scheduled_end)
                    {
                        ++skipped_controller_period_count;
                        next_tick += scenario.controller_period;
                    }

                    if (next_tick > scheduled_end)
                    {
                        static_cast<void>(shutdown_coordinator.requestStop());
                        controller_deadline_waiter.signalStop();
                        break;
                    }
                }

                final_state = interpolator.state();
                shutdown_coordinator.markControllerStopped();
                wake_condition.notify_all();
            });

        controller_thread.join();
        network_thread.join();
        logger_thread.join();

        const ControllerClock::time_point run_end = ControllerClock::now();
        const ControllerLatenessPercentiles lateness_percentiles =
            calculateLatenessPercentiles(controller_timing);
        result.fault_reason = shutdown_coordinator.faultReason();
        result.outcome = result.fault_reason == FaultReason::None ? RealTimeRunOutcome::Completed
                                                                  : RealTimeRunOutcome::Faulted;
        result.final_state = final_state;
        result.shutdown_phase = shutdown_coordinator.phase();
        result.controller_wait_strategy = controller_deadline_waiter.strategy();
        result.controller_affinity_result = controller_affinity_result;
        result.delivered_command_count = delivered_command_count;
        result.controller_tick_count = controller_tick_count;
        result.skipped_controller_period_count = skipped_controller_period_count;
        result.p50_controller_lateness = lateness_percentiles.p50;
        result.p95_controller_lateness = lateness_percentiles.p95;
        result.p99_controller_lateness = lateness_percentiles.p99;
        result.maximum_controller_lateness = maximum_controller_lateness;
        result.maximum_control_call_duration = maximum_control_call_duration;
        result.wall_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(run_end - run_begin);
        result.dropped_input_telemetry_count = interpolator.droppedInputTelemetryCount();
        result.dropped_control_telemetry_count = interpolator.droppedControlTelemetryCount();
        result.input_telemetry = std::move(input_telemetry);
        result.control_telemetry = std::move(control_telemetry);
        result.controller_timing = std::move(controller_timing);

        return result;
    }
}
