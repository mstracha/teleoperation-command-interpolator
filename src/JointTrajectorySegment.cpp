#include "teleoperation/JointTrajectorySegment.hpp"

#include "teleoperation/QuinticValidator.hpp"

#include <cmath>

namespace teleoperation
{
    namespace
    {
        bool isFinite(const JointLimit &limit) noexcept
        {
            return std::isfinite(limit.minimum_position) && std::isfinite(limit.maximum_position) &&
                   std::isfinite(limit.minimum_velocity) && std::isfinite(limit.maximum_velocity) &&
                   std::isfinite(limit.minimum_acceleration) &&
                   std::isfinite(limit.maximum_acceleration) && std::isfinite(limit.minimum_jerk) &&
                   std::isfinite(limit.maximum_jerk);
        }

        bool hasOrderedBounds(const JointLimit &limit) noexcept
        {
            return limit.minimum_position <= limit.maximum_position &&
                   limit.minimum_velocity <= limit.maximum_velocity &&
                   limit.minimum_acceleration <= limit.maximum_acceleration &&
                   limit.minimum_jerk <= limit.maximum_jerk;
        }

        bool isWithin(double value, double minimum, double maximum) noexcept
        {
            return value >= minimum && value <= maximum;
        }

        JointTrajectoryConfigurationResult mapConfigurationResult(
            QuinticConfigurationResult result) noexcept
        {
            switch (result)
            {
                case QuinticConfigurationResult::Success:
                    return JointTrajectoryConfigurationResult::Success;

                case QuinticConfigurationResult::InvalidDuration:
                    return JointTrajectoryConfigurationResult::InvalidDuration;

                case QuinticConfigurationResult::NonFiniteBoundary:
                    return JointTrajectoryConfigurationResult::NonFiniteBoundary;

                case QuinticConfigurationResult::NonFiniteCoefficients:
                    return JointTrajectoryConfigurationResult::NonFiniteCoefficients;
            }

            return JointTrajectoryConfigurationResult::NonFiniteCoefficients;
        }

        JointTrajectoryEvaluationResult mapEvaluationResult(QuinticEvaluationResult result) noexcept
        {
            switch (result)
            {
                case QuinticEvaluationResult::Success:
                    return JointTrajectoryEvaluationResult::Success;

                case QuinticEvaluationResult::NotConfigured:
                    return JointTrajectoryEvaluationResult::NotConfigured;

                case QuinticEvaluationResult::NonFiniteTime:
                    return JointTrajectoryEvaluationResult::NonFiniteTime;

                case QuinticEvaluationResult::TimeBeforeSegment:
                    return JointTrajectoryEvaluationResult::TimeBeforeSegment;

                case QuinticEvaluationResult::TimeAfterSegment:
                    return JointTrajectoryEvaluationResult::TimeAfterSegment;

                case QuinticEvaluationResult::NonFiniteSample:
                    return JointTrajectoryEvaluationResult::NonFiniteSample;
            }

            return JointTrajectoryEvaluationResult::NonFiniteSample;
        }

        JointTrajectoryConfigurationResult mapValidationResult(
            QuinticValidationResult result) noexcept
        {
            switch (result)
            {
                case QuinticValidationResult::Success:
                    return JointTrajectoryConfigurationResult::Success;

                case QuinticValidationResult::PositionLimitExceeded:
                    return JointTrajectoryConfigurationResult::PositionLimitExceeded;

                case QuinticValidationResult::VelocityLimitExceeded:
                    return JointTrajectoryConfigurationResult::VelocityLimitExceeded;

                case QuinticValidationResult::AccelerationLimitExceeded:
                    return JointTrajectoryConfigurationResult::AccelerationLimitExceeded;

                case QuinticValidationResult::JerkLimitExceeded:
                    return JointTrajectoryConfigurationResult::JerkLimitExceeded;

                case QuinticValidationResult::NotConfigured:
                case QuinticValidationResult::InvalidLimits:
                case QuinticValidationResult::NumericalFailure:
                    return JointTrajectoryConfigurationResult::ValidationNumericalFailure;
            }

            return JointTrajectoryConfigurationResult::ValidationNumericalFailure;
        }
    }

    JointTrajectorySegment::JointTrajectorySegment(const JointLimits &joint_limits) noexcept
        : joint_limits_(joint_limits)
    {
    }

    JointTrajectoryConfigurationStatus JointTrajectorySegment::configure(
        const JointTrajectoryBoundaryState &start, const JointTrajectoryBoundaryState &end,
        double duration_seconds) noexcept
    {
        configured_ = false;
        duration_seconds_ = 0.0;

        const JointTrajectoryConfigurationStatus limits_status = validateJointLimits();

        if (!limits_status.succeeded())
        {
            return limits_status;
        }

        if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0)
        {
            return {JointTrajectoryConfigurationResult::InvalidDuration,
                    JointTrajectoryConfigurationStatus::NoJoint};
        }

        const JointTrajectoryConfigurationStatus start_status = validateBoundary(start, 0.0);

        if (!start_status.succeeded())
        {
            return start_status;
        }

        const JointTrajectoryConfigurationStatus end_status =
            validateBoundary(end, duration_seconds);

        if (!end_status.succeeded())
        {
            return end_status;
        }

        std::array<QuinticSegment, JointCount> configured_segments{};

        for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
        {
            const QuinticBoundaryState joint_start{start.positions[joint_index],
                                                   start.velocities[joint_index],
                                                   start.accelerations[joint_index]};

            const QuinticBoundaryState joint_end{end.positions[joint_index],
                                                 end.velocities[joint_index],
                                                 end.accelerations[joint_index]};

            const QuinticConfigurationResult result = configured_segments[joint_index].configure(
                joint_start, joint_end, duration_seconds);

            if (result != QuinticConfigurationResult::Success)
            {
                return {mapConfigurationResult(result), joint_index};
            }

            const QuinticValidationStatus validation_status = QuinticValidator::validate(
                configured_segments[joint_index], joint_limits_[joint_index]);

            if (!validation_status.succeeded())
            {
                return {mapValidationResult(validation_status.result), joint_index,
                        validation_status.time_seconds, validation_status.value};
            }
        }

        segments_ = configured_segments;
        duration_seconds_ = duration_seconds;
        configured_ = true;

        return {JointTrajectoryConfigurationResult::Success,
                JointTrajectoryConfigurationStatus::NoJoint};
    }

    JointTrajectoryEvaluationStatus JointTrajectorySegment::evaluate(
        double elapsed_seconds, JointTrajectorySample &sample) const noexcept
    {
        if (!configured_)
        {
            return {JointTrajectoryEvaluationResult::NotConfigured,
                    JointTrajectoryEvaluationStatus::NoJoint};
        }

        JointTrajectorySample evaluated{};

        for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
        {
            QuinticSample joint_sample{};
            const QuinticEvaluationResult result =
                segments_[joint_index].evaluate(elapsed_seconds, joint_sample);

            if (result != QuinticEvaluationResult::Success)
            {
                const JointTrajectoryEvaluationResult mapped_result = mapEvaluationResult(result);

                const std::size_t failed_joint =
                    mapped_result == JointTrajectoryEvaluationResult::NonFiniteSample
                        ? joint_index
                        : JointTrajectoryEvaluationStatus::NoJoint;

                return {mapped_result, failed_joint};
            }

            const JointLimit &limit = joint_limits_[joint_index];

            if (!isWithin(joint_sample.position, limit.minimum_position, limit.maximum_position))
            {
                return {JointTrajectoryEvaluationResult::PositionLimitExceeded, joint_index,
                        joint_sample.position};
            }

            if (!isWithin(joint_sample.velocity, limit.minimum_velocity, limit.maximum_velocity))
            {
                return {JointTrajectoryEvaluationResult::VelocityLimitExceeded, joint_index,
                        joint_sample.velocity};
            }

            if (!isWithin(joint_sample.acceleration, limit.minimum_acceleration,
                          limit.maximum_acceleration))
            {
                return {JointTrajectoryEvaluationResult::AccelerationLimitExceeded, joint_index,
                        joint_sample.acceleration};
            }

            if (!isWithin(joint_sample.jerk, limit.minimum_jerk, limit.maximum_jerk))
            {
                return {JointTrajectoryEvaluationResult::JerkLimitExceeded, joint_index,
                        joint_sample.jerk};
            }

            evaluated.positions[joint_index] = joint_sample.position;
            evaluated.velocities[joint_index] = joint_sample.velocity;
            evaluated.accelerations[joint_index] = joint_sample.acceleration;
            evaluated.jerks[joint_index] = joint_sample.jerk;
        }

        sample = evaluated;

        return {JointTrajectoryEvaluationResult::Success, JointTrajectoryEvaluationStatus::NoJoint};
    }

    bool JointTrajectorySegment::isConfigured() const noexcept
    {
        return configured_;
    }

    double JointTrajectorySegment::durationSeconds() const noexcept
    {
        return duration_seconds_;
    }

    JointTrajectoryConfigurationStatus JointTrajectorySegment::validateJointLimits() const noexcept
    {
        for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
        {
            const JointLimit &limit = joint_limits_[joint_index];

            if (!isFinite(limit) || !hasOrderedBounds(limit))
            {
                return {JointTrajectoryConfigurationResult::InvalidJointLimits, joint_index};
            }
        }

        return {JointTrajectoryConfigurationResult::Success,
                JointTrajectoryConfigurationStatus::NoJoint};
    }

    JointTrajectoryConfigurationStatus JointTrajectorySegment::validateBoundary(
        const JointTrajectoryBoundaryState &boundary, double boundary_time_seconds) const noexcept
    {
        for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
        {
            const double position = boundary.positions[joint_index];
            const double velocity = boundary.velocities[joint_index];
            const double acceleration = boundary.accelerations[joint_index];

            if (!std::isfinite(position))
            {
                return {JointTrajectoryConfigurationResult::NonFiniteBoundary, joint_index,
                        boundary_time_seconds, position};
            }

            if (!std::isfinite(velocity))
            {
                return {JointTrajectoryConfigurationResult::NonFiniteBoundary, joint_index,
                        boundary_time_seconds, velocity};
            }

            if (!std::isfinite(acceleration))
            {
                return {JointTrajectoryConfigurationResult::NonFiniteBoundary, joint_index,
                        boundary_time_seconds, acceleration};
            }

            const JointLimit &limit = joint_limits_[joint_index];

            if (!isWithin(position, limit.minimum_position, limit.maximum_position))
            {
                return {JointTrajectoryConfigurationResult::BoundaryPositionOutOfRange, joint_index,
                        boundary_time_seconds, position};
            }

            if (!isWithin(velocity, limit.minimum_velocity, limit.maximum_velocity))
            {
                return {JointTrajectoryConfigurationResult::BoundaryVelocityOutOfRange, joint_index,
                        boundary_time_seconds, velocity};
            }

            if (!isWithin(acceleration, limit.minimum_acceleration, limit.maximum_acceleration))
            {
                return {JointTrajectoryConfigurationResult::BoundaryAccelerationOutOfRange,
                        joint_index, boundary_time_seconds, acceleration};
            }
        }

        return {JointTrajectoryConfigurationResult::Success,
                JointTrajectoryConfigurationStatus::NoJoint};
    }
}
