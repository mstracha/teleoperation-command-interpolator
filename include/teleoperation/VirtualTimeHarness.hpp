#pragma once

#include "teleoperation/InterpolatorConfiguration.hpp"
#include "teleoperation/JointLimits.hpp"
#include "teleoperation/ShutdownCoordinator.hpp"
#include "teleoperation/TelemetrySamples.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace teleoperation
{
    struct ScheduledTargetCommand
    {
        ControllerTime delivery_time{};
        TargetCommand command{};
    };

    struct VirtualTimeScenario
    {
        ControllerTime controller_start{};
        ControllerTime controller_end{};
        std::chrono::nanoseconds controller_period{std::chrono::milliseconds{2}};
        std::size_t maximum_controller_ticks{1'000'000};
        std::vector<ScheduledTargetCommand> commands;
    };

    enum class VirtualTimeRunOutcome : std::uint8_t
    {
        Completed,
        Faulted,
        InvalidControllerRange,
        InvalidControllerPeriod,
        DeliveryTimesOutOfOrder,
        ControllerTickLimitExceeded
    };

    struct VirtualTimeRunResult
    {
        VirtualTimeRunOutcome outcome{VirtualTimeRunOutcome::Completed};
        InterpolatorState final_state{InterpolatorState::Buffering};
        FaultReason fault_reason{FaultReason::None};
        ShutdownPhase shutdown_phase{ShutdownPhase::Running};
        std::size_t delivered_command_count{0};
        std::size_t controller_tick_count{0};
        std::size_t dropped_input_telemetry_count{0};
        std::size_t dropped_control_telemetry_count{0};
        std::vector<InputTelemetrySample> input_telemetry;
        std::vector<ControlTelemetrySample> control_telemetry;
    };

    // Runs the network producer, 500 Hz controller, and telemetry consumer in one
    // deterministic virtual-time event loop. It never sleeps and does not create
    // threads; the real concurrent harness is a separate step.
    class VirtualTimeHarness
    {
      public:
        explicit VirtualTimeHarness(const JointLimits &joint_limits = DefaultToyJointLimits,
                                    const InterpolatorConfiguration &configuration = {});

        VirtualTimeRunResult run(const VirtualTimeScenario &scenario) const;

      private:
        JointLimits joint_limits_;
        InterpolatorConfiguration configuration_;
    };
}
