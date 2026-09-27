#pragma once

#include "teleoperation/JointLimits.hpp"
#include "teleoperation/QuinticSegment.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace teleoperation
{
    struct JointTrajectoryBoundaryState
    {
        JointPositions positions{};
        JointPositions velocities{};
        JointPositions accelerations{};
    };

    struct JointTrajectorySample
    {
        JointPositions positions{};
        JointPositions velocities{};
        JointPositions accelerations{};
        JointPositions jerks{};
    };

    enum class JointTrajectoryConfigurationResult : std::uint8_t
    {
        Success,
        InvalidJointLimits,
        InvalidDuration,
        NonFiniteBoundary,
        BoundaryPositionOutOfRange,
        BoundaryVelocityOutOfRange,
        BoundaryAccelerationOutOfRange,
        NonFiniteCoefficients,
        ValidationNumericalFailure,
        PositionLimitExceeded,
        VelocityLimitExceeded,
        AccelerationLimitExceeded,
        JerkLimitExceeded
    };

    struct JointTrajectoryConfigurationStatus
    {
        static constexpr std::size_t NoJoint = 6;

        JointTrajectoryConfigurationResult result{JointTrajectoryConfigurationResult::Success};

        std::size_t failed_joint{NoJoint};
        double failure_time_seconds{0.0};
        double failure_value{0.0};

        bool succeeded() const noexcept
        {
            return result == JointTrajectoryConfigurationResult::Success;
        }
    };

    enum class JointTrajectoryEvaluationResult : std::uint8_t
    {
        Success,
        NotConfigured,
        NonFiniteTime,
        TimeBeforeSegment,
        TimeAfterSegment,
        NonFiniteSample,
        PositionLimitExceeded,
        VelocityLimitExceeded,
        AccelerationLimitExceeded,
        JerkLimitExceeded
    };

    struct JointTrajectoryEvaluationStatus
    {
        static constexpr std::size_t NoJoint = 6;

        JointTrajectoryEvaluationResult result{JointTrajectoryEvaluationResult::Success};

        std::size_t failed_joint{NoJoint};
        double failure_value{0.0};

        bool succeeded() const noexcept
        {
            return result == JointTrajectoryEvaluationResult::Success;
        }
    };

    // Owns one scalar quintic segment for each of the six joints. Configuration
    // and evaluation are fixed-cost and allocation-free. Configuration validates
    // every mathematical position, velocity, acceleration, and jerk extremum over
    // the complete interval. Evaluation repeats the instantaneous checks as
    // defense in depth.
    class JointTrajectorySegment
    {
      public:
        explicit JointTrajectorySegment(const JointLimits &joint_limits) noexcept;

        JointTrajectoryConfigurationStatus configure(const JointTrajectoryBoundaryState &start,
                                                     const JointTrajectoryBoundaryState &end,
                                                     double duration_seconds) noexcept;

        JointTrajectoryEvaluationStatus evaluate(double elapsed_seconds,
                                                 JointTrajectorySample &sample) const noexcept;

        bool isConfigured() const noexcept;
        double durationSeconds() const noexcept;

      private:
        JointTrajectoryConfigurationStatus validateJointLimits() const noexcept;

        JointTrajectoryConfigurationStatus validateBoundary(
            const JointTrajectoryBoundaryState &boundary,
            double boundary_time_seconds) const noexcept;

        JointLimits joint_limits_;
        std::array<QuinticSegment, JointCount> segments_{};
        double duration_seconds_{0.0};
        bool configured_{false};
    };
}
