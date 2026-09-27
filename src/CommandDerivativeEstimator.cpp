#include "teleoperation/CommandDerivativeEstimator.hpp"

#include <cmath>

namespace teleoperation
{
    namespace
    {
        std::uint64_t positiveDifference(std::int64_t later, std::int64_t earlier) noexcept
        {
            return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
        }

        double nanosecondsToSeconds(std::uint64_t nanoseconds) noexcept
        {
            constexpr double SecondsPerNanosecond = 1.0e-9;
            return static_cast<double>(nanoseconds) * SecondsPerNanosecond;
        }
    }

    DerivativeEstimationStatus CommandDerivativeEstimator::estimate(
        const std::array<TargetCommand, CommandCount> &commands,
        EstimatedJointTrajectorySegment &estimate) noexcept
    {
        for (std::size_t command_index = 1; command_index < CommandCount; ++command_index)
        {
            const std::int64_t previous_time =
                commands[command_index - 1].sender_time.nanoseconds_since_sender_epoch;

            const std::int64_t current_time =
                commands[command_index].sender_time.nanoseconds_since_sender_epoch;

            if (current_time <= previous_time)
            {
                return {DerivativeEstimationResult::InvalidTimestampOrder, command_index,
                        DerivativeEstimationStatus::NoJoint};
            }
        }

        for (std::size_t command_index = 0; command_index < CommandCount; ++command_index)
        {
            for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
            {
                if (!std::isfinite(commands[command_index].positions[joint_index]))
                {
                    return {DerivativeEstimationResult::NonFinitePosition, command_index,
                            joint_index};
                }
            }
        }

        const std::int64_t time_0 = commands[0].sender_time.nanoseconds_since_sender_epoch;

        const std::int64_t time_1 = commands[1].sender_time.nanoseconds_since_sender_epoch;

        const std::int64_t time_2 = commands[2].sender_time.nanoseconds_since_sender_epoch;

        const std::int64_t time_3 = commands[3].sender_time.nanoseconds_since_sender_epoch;

        const double interval_01 = nanosecondsToSeconds(positiveDifference(time_1, time_0));

        const double interval_12 = nanosecondsToSeconds(positiveDifference(time_2, time_1));

        const double interval_23 = nanosecondsToSeconds(positiveDifference(time_3, time_2));

        EstimatedJointTrajectorySegment calculated{};
        calculated.duration_seconds = interval_12;
        calculated.start.positions = commands[1].positions;
        calculated.end.positions = commands[2].positions;

        for (std::size_t joint_index = 0; joint_index < JointCount; ++joint_index)
        {
            const double position_0 = commands[0].positions[joint_index];
            const double position_1 = commands[1].positions[joint_index];
            const double position_2 = commands[2].positions[joint_index];
            const double position_3 = commands[3].positions[joint_index];

            const double slope_01 = (position_1 - position_0) / interval_01;

            const double slope_12 = (position_2 - position_1) / interval_12;

            const double slope_23 = (position_3 - position_2) / interval_23;

            // These are the first and second derivatives of the nonuniform
            // three-point quadratic at its middle sample.
            const double start_velocity =
                (interval_12 * slope_01 + interval_01 * slope_12) / (interval_01 + interval_12);

            const double start_acceleration =
                2.0 * (slope_12 - slope_01) / (interval_01 + interval_12);

            const double end_velocity =
                (interval_23 * slope_12 + interval_12 * slope_23) / (interval_12 + interval_23);

            const double end_acceleration =
                2.0 * (slope_23 - slope_12) / (interval_12 + interval_23);

            if (!std::isfinite(start_velocity) || !std::isfinite(start_acceleration))
            {
                return {DerivativeEstimationResult::NonFiniteEstimate, std::size_t{1}, joint_index};
            }

            if (!std::isfinite(end_velocity) || !std::isfinite(end_acceleration))
            {
                return {DerivativeEstimationResult::NonFiniteEstimate, std::size_t{2}, joint_index};
            }

            calculated.start.velocities[joint_index] = start_velocity;
            calculated.start.accelerations[joint_index] = start_acceleration;
            calculated.end.velocities[joint_index] = end_velocity;
            calculated.end.accelerations[joint_index] = end_acceleration;
        }

        estimate = calculated;

        return {DerivativeEstimationResult::Success, DerivativeEstimationStatus::NoCommand,
                DerivativeEstimationStatus::NoJoint};
    }
}
