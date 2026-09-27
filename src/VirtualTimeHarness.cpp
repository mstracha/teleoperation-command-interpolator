#include "teleoperation/VirtualTimeHarness.hpp"

#include "teleoperation/CommandInterpolator.hpp"

#include <array>
#include <limits>
#include <stdexcept>

namespace teleoperation
{
    namespace
    {
        constexpr std::size_t TelemetryDrainBatchSize = 128;

        std::uint64_t positiveDifference(std::int64_t later, std::int64_t earlier) noexcept
        {
            return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
        }

        void drainTelemetry(CommandInterpolator &interpolator, VirtualTimeRunResult &result)
        {
            std::array<InputTelemetrySample, TelemetryDrainBatchSize> input_batch{};
            std::array<ControlTelemetrySample, TelemetryDrainBatchSize> control_batch{};

            for (;;)
            {
                const std::size_t count = interpolator.drainInputTelemetry(input_batch);

                result.input_telemetry.insert(result.input_telemetry.end(), input_batch.begin(),
                                              input_batch.begin() + count);

                if (count < input_batch.size())
                {
                    break;
                }
            }

            for (;;)
            {
                const std::size_t count = interpolator.drainControlTelemetry(control_batch);

                result.control_telemetry.insert(result.control_telemetry.end(),
                                                control_batch.begin(),
                                                control_batch.begin() + count);

                if (count < control_batch.size())
                {
                    break;
                }
            }
        }
    }

    VirtualTimeHarness::VirtualTimeHarness(const JointLimits &joint_limits,
                                           const InterpolatorConfiguration &configuration)
        : joint_limits_(joint_limits), configuration_(configuration)
    {
        if (!configuration_.isValid())
        {
            throw std::invalid_argument("Invalid virtual-time configuration");
        }
    }

    VirtualTimeRunResult VirtualTimeHarness::run(const VirtualTimeScenario &scenario) const
    {
        VirtualTimeRunResult result{};
        const std::int64_t controller_start =
            scenario.controller_start.nanoseconds_since_controller_epoch;

        const std::int64_t controller_end =
            scenario.controller_end.nanoseconds_since_controller_epoch;

        if (controller_end < controller_start)
        {
            result.outcome = VirtualTimeRunOutcome::InvalidControllerRange;
            return result;
        }

        const std::int64_t controller_period = scenario.controller_period.count();

        if (controller_period <= 0)
        {
            result.outcome = VirtualTimeRunOutcome::InvalidControllerPeriod;
            return result;
        }

        for (std::size_t command_index = 1; command_index < scenario.commands.size();
             ++command_index)
        {
            const std::int64_t previous_delivery_time =
                scenario.commands[command_index - 1]
                    .delivery_time.nanoseconds_since_controller_epoch;

            const std::int64_t delivery_time =
                scenario.commands[command_index].delivery_time.nanoseconds_since_controller_epoch;

            if (delivery_time < previous_delivery_time)
            {
                result.outcome = VirtualTimeRunOutcome::DeliveryTimesOutOfOrder;
                return result;
            }
        }

        if (scenario.maximum_controller_ticks == 0)
        {
            result.outcome = VirtualTimeRunOutcome::ControllerTickLimitExceeded;
            return result;
        }

        const std::uint64_t scenario_duration =
            positiveDifference(controller_end, controller_start);

        const auto period = static_cast<std::uint64_t>(controller_period);
        const std::uint64_t controller_intervals = scenario_duration / period;

        if (controller_intervals >= static_cast<std::uint64_t>(scenario.maximum_controller_ticks))
        {
            result.outcome = VirtualTimeRunOutcome::ControllerTickLimitExceeded;
            return result;
        }

        CommandInterpolator interpolator(joint_limits_, configuration_);
        ShutdownCoordinator shutdown_coordinator;
        JointPositions target_positions{};
        std::size_t next_command_index = 0;
        std::int64_t current_time = controller_start;

        for (;;)
        {
            while (next_command_index < scenario.commands.size() &&
                   scenario.commands[next_command_index]
                           .delivery_time.nanoseconds_since_controller_epoch <= current_time)
            {
                const ScheduledTargetCommand &scheduled_command =
                    scenario.commands[next_command_index];

                static_cast<void>(interpolator.submitCommand(scheduled_command.command,
                                                             scheduled_command.delivery_time));

                ++next_command_index;
                ++result.delivered_command_count;
            }

            const ControlResult control_result =
                interpolator.updateTarget(ControllerTime{current_time}, target_positions);

            ++result.controller_tick_count;
            drainTelemetry(interpolator, result);

            if (control_result == ControlResult::Faulted)
            {
                result.outcome = VirtualTimeRunOutcome::Faulted;
                result.fault_reason = interpolator.faultReason();

                static_cast<void>(shutdown_coordinator.requestStop(result.fault_reason));

                static_cast<void>(shutdown_coordinator.beginLogDrain());
                shutdown_coordinator.markControllerStopped();
                shutdown_coordinator.markNetworkStopped();
                drainTelemetry(interpolator, result);
                static_cast<void>(shutdown_coordinator.markLoggingComplete());
                break;
            }

            const std::uint64_t remaining_time = positiveDifference(controller_end, current_time);

            if (remaining_time < period)
            {
                break;
            }

            current_time += controller_period;
        }

        drainTelemetry(interpolator, result);
        result.final_state = interpolator.state();
        result.fault_reason = interpolator.faultReason();
        result.shutdown_phase = shutdown_coordinator.phase();
        result.dropped_input_telemetry_count = interpolator.droppedInputTelemetryCount();
        result.dropped_control_telemetry_count = interpolator.droppedControlTelemetryCount();

        return result;
    }
}
