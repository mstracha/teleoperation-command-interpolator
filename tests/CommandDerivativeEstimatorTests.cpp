#include "teleoperation/CommandDerivativeEstimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    constexpr double NanosecondsPerSecond = 1.0e9;

    template <typename Actual, typename Expected>
    void expectEqual(Actual actual, Expected expected, const char *test_name)
    {
        if (actual != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectNear(double actual, double expected, double tolerance, const char *test_name)
    {
        const double scale = std::max(1.0, std::abs(expected));

        if (!std::isfinite(actual) || !std::isfinite(expected) ||
            std::abs(actual - expected) > tolerance * scale)
        {
            std::cerr << "FAILED: " << test_name << " expected " << expected << " but received "
                      << actual << '\n';
            ++failure_count;
        }
    }

    double senderSeconds(std::int64_t sender_nanoseconds)
    {
        return static_cast<double>(sender_nanoseconds) / NanosecondsPerSecond;
    }

    TargetCommand makeQuadraticCommand(std::int64_t sender_nanoseconds)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_nanoseconds};
        const double time = senderSeconds(sender_nanoseconds);

        for (std::size_t joint_index = 0; joint_index < command.positions.size(); ++joint_index)
        {
            const double initial_position = 0.25 * static_cast<double>(joint_index);

            const double initial_velocity = 1.0 + 0.1 * static_cast<double>(joint_index);

            const double acceleration = -0.5 + 0.2 * static_cast<double>(joint_index);

            command.positions[joint_index] =
                initial_position + initial_velocity * time + 0.5 * acceleration * time * time;
        }

        return command;
    }

    TargetCommand makeCubicCommand(std::int64_t sender_nanoseconds)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_nanoseconds};
        const double time = senderSeconds(sender_nanoseconds);

        for (std::size_t joint_index = 0; joint_index < command.positions.size(); ++joint_index)
        {
            const double scale = 1.0 + static_cast<double>(joint_index);

            command.positions[joint_index] =
                scale * (0.5 + time - 2.0 * time * time + 3.0 * time * time * time);
        }

        return command;
    }

    void expectArrayValue(const JointPositions &values, double expected, const char *test_name)
    {
        for (double value : values)
        {
            expectNear(value, expected, 0.0, test_name);
        }
    }
}

int main()
{
    constexpr std::int64_t Millisecond = 1'000'000;

    // The negative first timestamps confirm that calculations depend only on
    // sender-time intervals, not on the sender clock's arbitrary epoch.
    const std::array<std::int64_t, 4> quadratic_times{-20 * Millisecond, -5 * Millisecond,
                                                      15 * Millisecond, 50 * Millisecond};

    std::array<TargetCommand, 4> quadratic_commands{};

    for (std::size_t command_index = 0; command_index < quadratic_commands.size(); ++command_index)
    {
        quadratic_commands[command_index] = makeQuadraticCommand(quadratic_times[command_index]);
    }

    EstimatedJointTrajectorySegment quadratic_estimate{};

    const DerivativeEstimationStatus quadratic_status =
        CommandDerivativeEstimator::estimate(quadratic_commands, quadratic_estimate);

    expectEqual(quadratic_status.result, DerivativeEstimationResult::Success,
                "nonuniform quadratic window is estimated");

    expectEqual(quadratic_status.failed_command, DerivativeEstimationStatus::NoCommand,
                "successful estimate has no failed command");

    expectEqual(quadratic_status.failed_joint, DerivativeEstimationStatus::NoJoint,
                "successful estimate has no failed joint");

    expectNear(quadratic_estimate.duration_seconds, 0.020, 1.0e-15,
               "middle sender-time interval becomes segment duration");

    const double start_time = senderSeconds(quadratic_times[1]);
    const double end_time = senderSeconds(quadratic_times[2]);

    for (std::size_t joint_index = 0; joint_index < CommandDerivativeEstimator::JointCount;
         ++joint_index)
    {
        const double initial_velocity = 1.0 + 0.1 * static_cast<double>(joint_index);

        const double acceleration = -0.5 + 0.2 * static_cast<double>(joint_index);

        expectNear(quadratic_estimate.start.positions[joint_index],
                   quadratic_commands[1].positions[joint_index], 0.0,
                   "start position is copied from first middle command");

        expectNear(quadratic_estimate.end.positions[joint_index],
                   quadratic_commands[2].positions[joint_index], 0.0,
                   "end position is copied from second middle command");

        expectNear(quadratic_estimate.start.velocities[joint_index],
                   initial_velocity + acceleration * start_time, 1.0e-12,
                   "start velocity matches nonuniform quadratic derivative");

        expectNear(quadratic_estimate.end.velocities[joint_index],
                   initial_velocity + acceleration * end_time, 1.0e-12,
                   "end velocity matches nonuniform quadratic derivative");

        expectNear(quadratic_estimate.start.accelerations[joint_index], acceleration, 1.0e-10,
                   "start acceleration matches nonuniform quadratic derivative");

        expectNear(quadratic_estimate.end.accelerations[joint_index], acceleration, 1.0e-10,
                   "end acceleration matches nonuniform quadratic derivative");
    }

    const std::array<std::int64_t, 5> sliding_times{0, 10 * Millisecond, 25 * Millisecond,
                                                    45 * Millisecond, 70 * Millisecond};

    std::array<TargetCommand, 5> sliding_commands{};

    for (std::size_t command_index = 0; command_index < sliding_commands.size(); ++command_index)
    {
        sliding_commands[command_index] = makeCubicCommand(sliding_times[command_index]);
    }

    const std::array<TargetCommand, 4> first_window{sliding_commands[0], sliding_commands[1],
                                                    sliding_commands[2], sliding_commands[3]};

    const std::array<TargetCommand, 4> second_window{sliding_commands[1], sliding_commands[2],
                                                     sliding_commands[3], sliding_commands[4]};

    EstimatedJointTrajectorySegment first_estimate{};
    EstimatedJointTrajectorySegment second_estimate{};

    expectEqual(CommandDerivativeEstimator::estimate(first_window, first_estimate).result,
                DerivativeEstimationResult::Success, "first sliding window is estimated");

    expectEqual(CommandDerivativeEstimator::estimate(second_window, second_estimate).result,
                DerivativeEstimationResult::Success, "second sliding window is estimated");

    for (std::size_t joint_index = 0; joint_index < CommandDerivativeEstimator::JointCount;
         ++joint_index)
    {
        expectNear(first_estimate.end.positions[joint_index],
                   second_estimate.start.positions[joint_index], 0.0,
                   "sliding windows share boundary position");

        expectNear(first_estimate.end.velocities[joint_index],
                   second_estimate.start.velocities[joint_index], 0.0,
                   "sliding windows share boundary velocity");

        expectNear(first_estimate.end.accelerations[joint_index],
                   second_estimate.start.accelerations[joint_index], 0.0,
                   "sliding windows share boundary acceleration");
    }

    std::array<TargetCommand, 4> invalid_commands = quadratic_commands;
    invalid_commands[2].sender_time = invalid_commands[1].sender_time;

    EstimatedJointTrajectorySegment unchanged_estimate{};
    unchanged_estimate.duration_seconds = 4.0;
    unchanged_estimate.start.positions.fill(7.0);
    unchanged_estimate.start.velocities.fill(8.0);
    unchanged_estimate.start.accelerations.fill(9.0);
    unchanged_estimate.end.positions.fill(10.0);
    unchanged_estimate.end.velocities.fill(11.0);
    unchanged_estimate.end.accelerations.fill(12.0);

    const DerivativeEstimationStatus timestamp_status =
        CommandDerivativeEstimator::estimate(invalid_commands, unchanged_estimate);

    expectEqual(timestamp_status.result, DerivativeEstimationResult::InvalidTimestampOrder,
                "duplicate timestamp is rejected");

    expectEqual(timestamp_status.failed_command, std::size_t{2},
                "timestamp failure identifies its command");

    expectNear(unchanged_estimate.duration_seconds, 4.0, 0.0,
               "failed estimate leaves duration unchanged");

    expectArrayValue(unchanged_estimate.start.positions, 7.0,
                     "failed estimate leaves start positions unchanged");

    expectArrayValue(unchanged_estimate.start.velocities, 8.0,
                     "failed estimate leaves start velocities unchanged");

    expectArrayValue(unchanged_estimate.start.accelerations, 9.0,
                     "failed estimate leaves start accelerations unchanged");

    expectArrayValue(unchanged_estimate.end.positions, 10.0,
                     "failed estimate leaves end positions unchanged");

    expectArrayValue(unchanged_estimate.end.velocities, 11.0,
                     "failed estimate leaves end velocities unchanged");

    expectArrayValue(unchanged_estimate.end.accelerations, 12.0,
                     "failed estimate leaves end accelerations unchanged");

    invalid_commands = quadratic_commands;
    invalid_commands[0].positions[4] = std::numeric_limits<double>::quiet_NaN();

    const DerivativeEstimationStatus position_status =
        CommandDerivativeEstimator::estimate(invalid_commands, unchanged_estimate);

    expectEqual(position_status.result, DerivativeEstimationResult::NonFinitePosition,
                "non-finite position is rejected");

    expectEqual(position_status.failed_command, std::size_t{0},
                "position failure identifies its command");

    expectEqual(position_status.failed_joint, std::size_t{4},
                "position failure identifies its joint");

    invalid_commands = quadratic_commands;
    invalid_commands[0].positions[3] = -std::numeric_limits<double>::max();
    invalid_commands[1].positions[3] = std::numeric_limits<double>::max();

    const DerivativeEstimationStatus estimate_status =
        CommandDerivativeEstimator::estimate(invalid_commands, unchanged_estimate);

    expectEqual(estimate_status.result, DerivativeEstimationResult::NonFiniteEstimate,
                "non-finite derivative estimate is rejected");

    expectEqual(estimate_status.failed_command, std::size_t{1},
                "estimate failure identifies its boundary command");

    expectEqual(estimate_status.failed_joint, std::size_t{3},
                "estimate failure identifies its joint");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " derivative estimator test(s) failed\n";
        return 1;
    }

    std::cout << "All command derivative estimator tests passed\n";
    return 0;
}
