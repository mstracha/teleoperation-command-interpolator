#pragma once

#include "teleoperation/CommandTypes.hpp"
#include "teleoperation/JointTrajectorySegment.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace teleoperation
{
    struct EstimatedJointTrajectorySegment
    {
        JointTrajectoryBoundaryState start;
        JointTrajectoryBoundaryState end;
        double duration_seconds{0.0};
    };

    enum class DerivativeEstimationResult : std::uint8_t
    {
        Success,
        InvalidTimestampOrder,
        NonFinitePosition,
        NonFiniteEstimate
    };

    struct DerivativeEstimationStatus
    {
        static constexpr std::size_t NoCommand = 4;
        static constexpr std::size_t NoJoint = 6;

        DerivativeEstimationResult result{DerivativeEstimationResult::Success};
        std::size_t failed_command{NoCommand};
        std::size_t failed_joint{NoJoint};

        bool succeeded() const noexcept
        {
            return result == DerivativeEstimationResult::Success;
        }
    };

    // Estimates the velocity and acceleration at the middle two commands of a
    // four-command window. Each boundary uses the quadratic through itself and
    // its immediate neighbors. This supports nonuniform sender-time intervals and
    // gives adjacent sliding windows identical estimates at their shared command.
    class CommandDerivativeEstimator
    {
      public:
        static constexpr std::size_t CommandCount = 4;
        static constexpr std::size_t JointCount = 6;

        static DerivativeEstimationStatus estimate(
            const std::array<TargetCommand, CommandCount> &commands,
            EstimatedJointTrajectorySegment &estimate) noexcept;
    };
}
