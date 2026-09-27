#pragma once

#include "teleoperation/CommandTypes.hpp"
#include "teleoperation/FaultReason.hpp"
#include "teleoperation/InterpolatorResults.hpp"
#include "teleoperation/InterpolatorStateMachine.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace teleoperation
{
    struct InputTelemetrySample
    {
        ControllerTime received_time{};
        SenderTime sender_time{};
        JointPositions commanded_positions{};
        CommandResult result{CommandResult::Accepted};
    };

    struct ControlTelemetrySample
    {
        static constexpr std::size_t NoFailedCommand = 4;
        static constexpr std::size_t NoFailedJoint = 6;

        ControllerTime controller_time{};
        JointPositions target_positions{};
        std::int64_t segment_elapsed_nanoseconds{0};
        std::int64_t segment_duration_nanoseconds{0};
        ControlResult result{ControlResult::Buffering};
        InterpolatorState state{InterpolatorState::Buffering};
        FaultReason fault_reason{FaultReason::None};
        std::size_t failed_command{NoFailedCommand};
        std::size_t failed_joint{NoFailedJoint};
        double trajectory_failure_time_seconds{0.0};
        double trajectory_failure_value{0.0};
        bool has_target{false};
    };

    static_assert(std::is_trivially_copyable<InputTelemetrySample>::value,
                  "Input telemetry must remain a fixed-cost value type");

    static_assert(std::is_trivially_copyable<ControlTelemetrySample>::value,
                  "Control telemetry must remain a fixed-cost value type");
}
