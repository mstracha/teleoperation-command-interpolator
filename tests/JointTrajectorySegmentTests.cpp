#include "teleoperation/JointTrajectorySegment.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    JointLimits makeUniformLimits(const JointLimit &limit)
    {
        return JointLimits{{limit, limit, limit, limit, limit, limit}};
    }

    JointTrajectoryBoundaryState makeBoundary(double position, double velocity, double acceleration)
    {
        JointTrajectoryBoundaryState boundary{};
        boundary.positions.fill(position);
        boundary.velocities.fill(velocity);
        boundary.accelerations.fill(acceleration);
        return boundary;
    }

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

    void expectSampleValue(const JointPositions &values, double expected, const char *test_name)
    {
        for (double value : values)
        {
            expectNear(value, expected, 1.0e-12, test_name);
        }
    }
}

int main()
{
    const JointLimits generous_limits =
        makeUniformLimits(JointLimit{-2.0, 2.0, -10.0, 10.0, -20.0, 20.0, -100.0, 100.0});

    const JointTrajectoryBoundaryState zero_rest = makeBoundary(0.0, 0.0, 0.0);

    const JointTrajectoryBoundaryState one_rest = makeBoundary(1.0, 0.0, 0.0);

    JointTrajectorySegment trajectory(generous_limits);

    const JointTrajectoryConfigurationStatus configuration_status =
        trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(configuration_status.result, JointTrajectoryConfigurationResult::Success,
                "six-joint trajectory configures");

    expectEqual(configuration_status.failed_joint, JointTrajectoryConfigurationStatus::NoJoint,
                "successful configuration has no failed joint");

    expectEqual(trajectory.isConfigured(), true, "successful trajectory reports configured");

    expectNear(trajectory.durationSeconds(), 1.0, 0.0, "trajectory duration is preserved");

    JointTrajectorySample sample{};
    const JointTrajectoryEvaluationStatus evaluation_status = trajectory.evaluate(0.5, sample);

    expectEqual(evaluation_status.result, JointTrajectoryEvaluationResult::Success,
                "six-joint trajectory evaluates");

    expectEqual(evaluation_status.failed_joint, JointTrajectoryEvaluationStatus::NoJoint,
                "successful evaluation has no failed joint");

    expectSampleValue(sample.positions, 0.5, "midpoint positions");
    expectSampleValue(sample.velocities, 1.875, "midpoint velocities");
    expectSampleValue(sample.accelerations, 0.0, "midpoint accelerations");
    expectSampleValue(sample.jerks, -30.0, "midpoint jerks");

    JointTrajectoryBoundaryState invalid_boundary = zero_rest;
    invalid_boundary.positions[4] = 3.0;

    const JointTrajectoryConfigurationStatus position_boundary_status =
        trajectory.configure(invalid_boundary, one_rest, 1.0);

    expectEqual(position_boundary_status.result,
                JointTrajectoryConfigurationResult::BoundaryPositionOutOfRange,
                "out-of-range boundary position is rejected");

    expectEqual(position_boundary_status.failed_joint, std::size_t{4},
                "boundary failure identifies its joint");

    expectNear(position_boundary_status.failure_time_seconds, 0.0, 0.0,
               "start-boundary failure reports start time");

    expectNear(position_boundary_status.failure_value, 3.0, 0.0,
               "boundary failure reports the rejected value");

    expectEqual(trajectory.isConfigured(), false,
                "failed reconfiguration invalidates the old trajectory");

    invalid_boundary = zero_rest;
    invalid_boundary.velocities[3] = 11.0;

    expectEqual(trajectory.configure(invalid_boundary, one_rest, 1.0).result,
                JointTrajectoryConfigurationResult::BoundaryVelocityOutOfRange,
                "out-of-range boundary velocity is rejected");

    invalid_boundary = zero_rest;
    invalid_boundary.accelerations[2] = -21.0;

    expectEqual(trajectory.configure(invalid_boundary, one_rest, 1.0).result,
                JointTrajectoryConfigurationResult::BoundaryAccelerationOutOfRange,
                "out-of-range boundary acceleration is rejected");

    invalid_boundary = zero_rest;
    invalid_boundary.positions[1] = std::numeric_limits<double>::quiet_NaN();

    expectEqual(trajectory.configure(invalid_boundary, one_rest, 1.0).result,
                JointTrajectoryConfigurationResult::NonFiniteBoundary,
                "non-finite boundary is rejected");

    expectEqual(trajectory.configure(zero_rest, one_rest, 0.0).result,
                JointTrajectoryConfigurationResult::InvalidDuration,
                "invalid duration is rejected");

    expectEqual(
        trajectory.configure(zero_rest, one_rest, std::numeric_limits<double>::min()).result,
        JointTrajectoryConfigurationResult::NonFiniteCoefficients,
        "numerically unsafe duration is rejected");

    JointLimits invalid_limits = generous_limits;
    invalid_limits[2].minimum_jerk = 5.0;
    invalid_limits[2].maximum_jerk = -5.0;
    JointTrajectorySegment invalid_limits_trajectory(invalid_limits);

    const JointTrajectoryConfigurationStatus invalid_limits_status =
        invalid_limits_trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(invalid_limits_status.result,
                JointTrajectoryConfigurationResult::InvalidJointLimits,
                "unordered limits are rejected");

    expectEqual(invalid_limits_status.failed_joint, std::size_t{2},
                "invalid limits identify their joint");

    JointLimits non_finite_limits = generous_limits;
    non_finite_limits[5].maximum_acceleration = std::numeric_limits<double>::infinity();
    JointTrajectorySegment non_finite_limits_trajectory(non_finite_limits);

    const JointTrajectoryConfigurationStatus non_finite_limits_status =
        non_finite_limits_trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(non_finite_limits_status.result,
                JointTrajectoryConfigurationResult::InvalidJointLimits,
                "non-finite limits are rejected");

    expectEqual(non_finite_limits_status.failed_joint, std::size_t{5},
                "non-finite limits identify their joint");

    JointLimits velocity_limits = generous_limits;
    velocity_limits[4].maximum_velocity = 1.8;

    JointTrajectorySegment velocity_limited_trajectory(velocity_limits);

    const JointTrajectoryConfigurationStatus velocity_status =
        velocity_limited_trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(velocity_status.result, JointTrajectoryConfigurationResult::VelocityLimitExceeded,
                "continuous velocity violation rejects configuration");

    expectEqual(velocity_status.failed_joint, std::size_t{4},
                "continuous velocity violation identifies its joint");

    expectNear(velocity_status.failure_time_seconds, 0.5, 1.0e-10,
               "continuous velocity violation reports its extremum time");

    expectNear(velocity_status.failure_value, 1.875, 1.0e-10,
               "continuous velocity violation reports its extremum value");

    expectEqual(velocity_limited_trajectory.isConfigured(), false,
                "continuous validation failure rejects the whole trajectory");

    expectEqual(velocity_limited_trajectory.evaluate(0.5, sample).result,
                JointTrajectoryEvaluationResult::NotConfigured,
                "rejected trajectory cannot be evaluated");

    JointTrajectorySegment evaluation_trajectory(generous_limits);

    expectEqual(evaluation_trajectory.configure(zero_rest, one_rest, 1.0).result,
                JointTrajectoryConfigurationResult::Success,
                "evaluation-failure test trajectory configures");

    JointTrajectorySample unchanged_sample{};
    unchanged_sample.positions.fill(7.0);
    unchanged_sample.velocities.fill(8.0);
    unchanged_sample.accelerations.fill(9.0);
    unchanged_sample.jerks.fill(10.0);

    expectEqual(evaluation_trajectory.evaluate(-0.001, unchanged_sample).result,
                JointTrajectoryEvaluationResult::TimeBeforeSegment,
                "time before the trajectory is rejected");

    expectSampleValue(unchanged_sample.positions, 7.0,
                      "failed evaluation leaves output positions unchanged");

    expectSampleValue(unchanged_sample.velocities, 8.0,
                      "failed evaluation leaves output velocities unchanged");

    expectSampleValue(unchanged_sample.accelerations, 9.0,
                      "failed evaluation leaves output accelerations unchanged");

    expectSampleValue(unchanged_sample.jerks, 10.0,
                      "failed evaluation leaves output jerks unchanged");

    expectEqual(evaluation_trajectory.evaluate(1.001, sample).result,
                JointTrajectoryEvaluationResult::TimeAfterSegment,
                "time after the trajectory is rejected");

    expectEqual(
        evaluation_trajectory.evaluate(std::numeric_limits<double>::quiet_NaN(), sample).result,
        JointTrajectoryEvaluationResult::NonFiniteTime, "non-finite evaluation time is rejected");

    JointLimits acceleration_limits = generous_limits;
    acceleration_limits[3].maximum_acceleration = 5.7;

    JointTrajectorySegment acceleration_limited_trajectory(acceleration_limits);

    const JointTrajectoryConfigurationStatus acceleration_status =
        acceleration_limited_trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(acceleration_status.result,
                JointTrajectoryConfigurationResult::AccelerationLimitExceeded,
                "continuous acceleration violation rejects configuration");

    expectEqual(acceleration_status.failed_joint, std::size_t{3},
                "continuous acceleration violation identifies its joint");

    JointLimits jerk_limits = generous_limits;
    jerk_limits[2].minimum_jerk = -29.0;

    JointTrajectorySegment jerk_limited_trajectory(jerk_limits);

    const JointTrajectoryConfigurationStatus jerk_status =
        jerk_limited_trajectory.configure(zero_rest, one_rest, 1.0);

    expectEqual(jerk_status.result, JointTrajectoryConfigurationResult::JerkLimitExceeded,
                "continuous jerk violation rejects configuration");

    expectEqual(jerk_status.failed_joint, std::size_t{2},
                "continuous jerk violation identifies its joint");

    const JointLimits overshoot_limits =
        makeUniformLimits(JointLimit{-1.0, 1.0, -20.0, 20.0, -100.0, 100.0, -1000.0, 1000.0});

    JointTrajectorySegment overshoot_trajectory(overshoot_limits);
    const JointTrajectoryBoundaryState high_velocity_boundary = makeBoundary(0.0, 10.0, 0.0);

    const JointTrajectoryConfigurationStatus overshoot_status =
        overshoot_trajectory.configure(high_velocity_boundary, high_velocity_boundary, 1.0);

    expectEqual(overshoot_status.result, JointTrajectoryConfigurationResult::PositionLimitExceeded,
                "continuous position overshoot rejects configuration");

    expectEqual(overshoot_status.failure_value > 1.0, true,
                "continuous position failure reports the overshoot value");

    expectEqual(trajectory.evaluate(0.5, sample).result,
                JointTrajectoryEvaluationResult::NotConfigured,
                "failed configuration prevents evaluation");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " joint trajectory test(s) failed\n";
        return 1;
    }

    std::cout << "All joint trajectory segment tests passed\n";
    return 0;
}
