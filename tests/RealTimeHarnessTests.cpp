#include "teleoperation/RealTimeHarness.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    constexpr std::int64_t Millisecond = 1'000'000;

    template <typename Actual, typename Expected>
    void expectEqual(Actual actual, Expected expected, const char *test_name)
    {
        if (actual != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectTrue(bool condition, const char *test_name)
    {
        if (!condition)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    TargetCommand makeCommand(std::int64_t sender_time, double position)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_time};
        command.positions.fill(position);
        return command;
    }
}

int main()
{
    InterpolatorConfiguration relaxed_watchdog{};
    relaxed_watchdog.hold_timeout = std::chrono::seconds{1};
    relaxed_watchdog.fault_timeout = std::chrono::seconds{2};
    RealTimeHarness harness(DefaultToyJointLimits, relaxed_watchdog);

    RealTimeScenario healthy_scenario{};
    healthy_scenario.run_duration = std::chrono::seconds{1};

    for (std::int64_t command_index = 0; command_index < 7; ++command_index)
    {
        healthy_scenario.commands.push_back(
            RealTimeScheduledCommand{std::chrono::milliseconds{40 * command_index},
                                     makeCommand(40 * command_index * Millisecond,
                                                 0.01 * static_cast<double>(command_index))});
    }

    const RealTimeRunResult healthy_result = harness.run(healthy_scenario);

    expectEqual(healthy_result.outcome, RealTimeRunOutcome::Completed,
                "healthy threaded scenario completes normally");

    expectEqual(healthy_result.fault_reason, FaultReason::None,
                "healthy threaded scenario has no fault");

    expectEqual(healthy_result.shutdown_phase, ShutdownPhase::Complete,
                "normal threaded completion drains logs and shuts down");

    expectEqual(healthy_result.delivered_command_count, std::size_t{7},
                "healthy threaded scenario delivers every command");

    expectEqual(healthy_result.configured_run_duration, healthy_scenario.run_duration,
                "threaded result preserves configured run duration");

    expectEqual(healthy_result.configured_controller_period, healthy_scenario.controller_period,
                "threaded result preserves configured controller period");

    expectEqual(healthy_result.configured_logger_poll_period, healthy_scenario.logger_poll_period,
                "threaded result preserves configured logger period");

    expectEqual(healthy_result.configured_startup_delay, healthy_scenario.startup_delay,
                "threaded result preserves configured startup delay");

    expectEqual(healthy_result.configured_controller_wait_preference,
                ControllerWaitPreference::Automatic,
                "threaded result preserves the requested wait preference");

#ifdef _WIN32
    expectEqual(healthy_result.controller_wait_strategy,
                ControllerWaitStrategy::WindowsHighResolutionWaitableTimer,
                "Windows threaded scenario uses the high-resolution waitable timer");
#else
    expectEqual(healthy_result.controller_wait_strategy,
                ControllerWaitStrategy::PortableConditionVariable,
                "non-Windows threaded scenario uses the portable condition-variable wait");
#endif

    expectEqual(healthy_result.scheduled_command_count, healthy_scenario.commands.size(),
                "threaded result preserves scheduled command count");

    const std::size_t expected_controller_tick_slots = static_cast<std::size_t>(
        healthy_scenario.run_duration / healthy_scenario.controller_period);

    const std::size_t accounted_controller_tick_slots =
        healthy_result.controller_tick_count + healthy_result.skipped_controller_period_count;

    expectTrue(accounted_controller_tick_slots <= expected_controller_tick_slots,
               "threaded scheduler never accounts for more than the configured tick slots");

    expectTrue(accounted_controller_tick_slots + 1 >= expected_controller_tick_slots,
               "threaded scheduler accounts for every configured tick slot");

    expectTrue(healthy_result.controller_tick_count * 3 >= expected_controller_tick_slots * 2,
               "healthy threaded scenario executes sustained controller ticks");

    expectEqual(healthy_result.input_telemetry.size(), healthy_result.delivered_command_count,
                "threaded logger captures every input event");

    expectEqual(healthy_result.control_telemetry.size(), healthy_result.controller_tick_count,
                "threaded logger captures every control event");

    expectEqual(healthy_result.dropped_input_telemetry_count, std::size_t{0},
                "threaded logger loses no input events");

    expectEqual(healthy_result.dropped_control_telemetry_count, std::size_t{0},
                "threaded logger loses no control events");

    expectTrue(healthy_result.maximum_control_call_duration.count() > 0,
               "threaded harness measures controller execution time");

    expectEqual(healthy_result.controller_timing.size(), healthy_result.controller_tick_count,
                "threaded harness records one timing sample per controller tick");

    expectTrue(healthy_result.p50_controller_lateness <=
                   healthy_result.p95_controller_lateness &&
                   healthy_result.p95_controller_lateness <=
                       healthy_result.p99_controller_lateness &&
                   healthy_result.p99_controller_lateness <=
                       healthy_result.maximum_controller_lateness,
               "threaded lateness percentiles are ordered and bounded by the maximum");

    if (!healthy_result.controller_timing.empty())
    {
        expectEqual(healthy_result.controller_timing.front().scheduled_offset,
                    std::chrono::nanoseconds{0},
                    "first controller timing sample is scheduled at the run origin");
    }

    const bool produced_target =
        std::any_of(healthy_result.control_telemetry.begin(),
                    healthy_result.control_telemetry.end(), [](const ControlTelemetrySample &sample)
                    { return sample.result == ControlResult::TargetProduced; });

    expectTrue(produced_target, "healthy threaded scenario produces interpolated targets");

    double observed_controller_rate_hz = 0.0;

    if (healthy_result.control_telemetry.size() > 1)
    {
        const std::int64_t observed_span_nanoseconds =
            healthy_result.control_telemetry.back()
                .controller_time.nanoseconds_since_controller_epoch -
            healthy_result.control_telemetry.front()
                .controller_time.nanoseconds_since_controller_epoch;

        if (observed_span_nanoseconds > 0)
        {
            observed_controller_rate_hz =
                static_cast<double>(healthy_result.control_telemetry.size() - 1) * 1'000'000'000.0 /
                static_cast<double>(observed_span_nanoseconds);
        }
    }

    expectTrue(std::isfinite(observed_controller_rate_hz) && observed_controller_rate_hz > 0.0,
               "threaded harness derives a finite observed controller rate");

    const double requested_controller_rate_hz =
        1'000'000'000.0 / static_cast<double>(healthy_scenario.controller_period.count());

    std::cout << std::fixed << std::setprecision(2) << "Real-time controller diagnostic\n"
              << "  requested_frequency_hz: " << requested_controller_rate_hz << '\n'
              << "  configured_duration_ms: "
              << static_cast<double>(healthy_scenario.run_duration.count()) / 1'000'000.0 << '\n'
              << "  wait_strategy: "
              << (healthy_result.controller_wait_strategy ==
                          ControllerWaitStrategy::WindowsHighResolutionWaitableTimer
                      ? "windows_high_resolution_waitable_timer"
                      : "portable_condition_variable")
              << '\n'
              << "  controller_ticks: " << healthy_result.controller_tick_count << '\n'
              << "  skipped_periods: " << healthy_result.skipped_controller_period_count << '\n'
              << "  observed_frequency_hz: " << observed_controller_rate_hz << '\n'
              << "  maximum_lateness_us: "
              << static_cast<double>(healthy_result.maximum_controller_lateness.count()) / 1'000.0
              << '\n'
              << "  p50_lateness_us: "
              << static_cast<double>(healthy_result.p50_controller_lateness.count()) / 1'000.0
              << '\n'
              << "  p95_lateness_us: "
              << static_cast<double>(healthy_result.p95_controller_lateness.count()) / 1'000.0
              << '\n'
              << "  p99_lateness_us: "
              << static_cast<double>(healthy_result.p99_controller_lateness.count()) / 1'000.0
              << '\n'
              << "  maximum_update_duration_us: "
              << static_cast<double>(healthy_result.maximum_control_call_duration.count()) / 1'000.0
              << '\n'
              << "  wall_time_ms: "
              << static_cast<double>(healthy_result.wall_time.count()) / 1'000'000.0 << '\n'
              << "  dropped_input_telemetry: " << healthy_result.dropped_input_telemetry_count
              << '\n'
              << "  dropped_control_telemetry: " << healthy_result.dropped_control_telemetry_count
              << '\n';

    RealTimeScenario unsafe_scenario{};
    unsafe_scenario.run_duration = std::chrono::milliseconds{200};

    for (std::int64_t command_index = 0; command_index < 4; ++command_index)
    {
        unsafe_scenario.commands.push_back(
            RealTimeScheduledCommand{std::chrono::milliseconds{20 * command_index},
                                     makeCommand(20 * command_index * Millisecond,
                                                 0.1 * static_cast<double>(command_index))});
    }

    const RealTimeRunResult unsafe_result = harness.run(unsafe_scenario);

    expectEqual(unsafe_result.outcome, RealTimeRunOutcome::Faulted,
                "unsafe threaded trajectory faults");

    expectEqual(unsafe_result.fault_reason, FaultReason::TrajectoryVelocityLimitExceeded,
                "unsafe threaded trajectory preserves fault reason");

    expectEqual(unsafe_result.shutdown_phase, ShutdownPhase::Complete,
                "faulted threaded run drains logs and shuts down");

    expectEqual(unsafe_result.delivered_command_count, std::size_t{4},
                "unsafe threaded scenario delivers its setup window");

    expectTrue(!unsafe_result.control_telemetry.empty(),
               "unsafe threaded scenario captures control telemetry");

    if (!unsafe_result.control_telemetry.empty())
    {
        expectEqual(unsafe_result.control_telemetry.back().fault_reason,
                    FaultReason::TrajectoryVelocityLimitExceeded,
                    "final threaded telemetry contains terminal fault");
    }

    RealTimeScenario invalid_scenario{};
    invalid_scenario.controller_period = std::chrono::nanoseconds{0};

    expectEqual(harness.run(invalid_scenario).outcome, RealTimeRunOutcome::InvalidControllerPeriod,
                "invalid threaded controller period is rejected");

    RealTimeScenario portable_wait_scenario{};
    portable_wait_scenario.run_duration = std::chrono::milliseconds{20};
    portable_wait_scenario.controller_wait_preference =
        ControllerWaitPreference::PortableConditionVariable;

    const RealTimeRunResult portable_wait_result = harness.run(portable_wait_scenario);

    expectEqual(portable_wait_result.controller_wait_strategy,
                ControllerWaitStrategy::PortableConditionVariable,
                "portable wait preference forces the portable wait strategy");

    RealTimeScenario affinity_scenario = portable_wait_scenario;

#ifdef _WIN32
    affinity_scenario.controller_logical_processor =
        std::numeric_limits<std::uintptr_t>::digits;
#else
    affinity_scenario.controller_logical_processor = 0;
#endif

    const RealTimeRunResult affinity_result = harness.run(affinity_scenario);

#ifdef _WIN32
    expectEqual(affinity_result.controller_affinity_result,
                ControllerAffinityResult::InvalidLogicalProcessor,
                "Windows rejects a logical processor outside the affinity mask");
#else
    expectEqual(affinity_result.controller_affinity_result,
                ControllerAffinityResult::Unsupported,
                "non-Windows harness reports controller affinity as unsupported");
#endif

    if (failure_count != 0)
    {
        std::cerr << failure_count << " real-time harness test(s) failed\n";
        return 1;
    }

    std::cout << "All real-time harness tests passed\n";
    return 0;
}
