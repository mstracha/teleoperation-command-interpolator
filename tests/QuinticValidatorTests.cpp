#include "teleoperation/QuinticValidator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

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
}

int main()
{
    const QuinticBoundaryState zero_rest{0.0, 0.0, 0.0};
    const QuinticBoundaryState one_rest{1.0, 0.0, 0.0};
    QuinticSegment smooth_step;

    expectEqual(smooth_step.configure(zero_rest, one_rest, 1.0),
                QuinticConfigurationResult::Success, "smooth-step segment configures");

    const JointLimit safe_limits{-0.1, 1.1, -2.0, 2.0, -6.0, 6.0, -61.0, 61.0};

    expectEqual(QuinticValidator::validate(smooth_step, safe_limits).result,
                QuinticValidationResult::Success, "safe complete interval validates");

    const JointLimit velocity_limits{-0.1, 1.1, -2.0, 1.8, -6.0, 6.0, -61.0, 61.0};

    const QuinticValidationStatus velocity_status =
        QuinticValidator::validate(smooth_step, velocity_limits);

    expectEqual(velocity_status.result, QuinticValidationResult::VelocityLimitExceeded,
                "interior velocity maximum is detected");

    expectNear(velocity_status.time_seconds, 0.5, 1.0e-10, "velocity maximum time is identified");

    expectNear(velocity_status.value, 1.875, 1.0e-10, "velocity maximum value is identified");

    QuinticSegment short_segment;

    expectEqual(short_segment.configure(zero_rest, one_rest, 0.2),
                QuinticConfigurationResult::Success, "short segment configures");

    const JointLimit short_segment_limits{-0.1, 1.1, -10.0, 9.0, -200.0, 200.0, -10000.0, 10000.0};

    const QuinticValidationStatus short_segment_status =
        QuinticValidator::validate(short_segment, short_segment_limits);

    expectEqual(short_segment_status.result, QuinticValidationResult::VelocityLimitExceeded,
                "normalized-time search detects short-segment velocity maximum");

    expectNear(short_segment_status.time_seconds, 0.1, 1.0e-10,
               "normalized extremum is mapped to physical time");

    expectNear(short_segment_status.value, 9.375, 1.0e-10,
               "short-segment physical velocity is reported");

    const JointLimit acceleration_limits{-0.1, 1.1, -2.0, 2.0, -5.7, 5.7, -61.0, 61.0};

    const QuinticValidationStatus acceleration_status =
        QuinticValidator::validate(smooth_step, acceleration_limits);

    expectEqual(acceleration_status.result, QuinticValidationResult::AccelerationLimitExceeded,
                "interior acceleration maximum is detected");

    expectNear(acceleration_status.time_seconds, 0.5 - std::sqrt(3.0) / 6.0, 1.0e-10,
               "acceleration maximum time is identified");

    expectNear(acceleration_status.value, 10.0 / std::sqrt(3.0), 1.0e-10,
               "acceleration maximum value is identified");

    const JointLimit jerk_limits{-0.1, 1.1, -2.0, 2.0, -6.0, 6.0, -29.0, 61.0};

    const QuinticValidationStatus jerk_status =
        QuinticValidator::validate(smooth_step, jerk_limits);

    expectEqual(jerk_status.result, QuinticValidationResult::JerkLimitExceeded,
                "interior jerk minimum is detected");

    expectNear(jerk_status.time_seconds, 0.5, 1.0e-10, "jerk minimum time is identified");

    expectNear(jerk_status.value, -30.0, 1.0e-10, "jerk minimum value is identified");

    QuinticSegment overshoot_segment;
    const QuinticBoundaryState high_velocity_boundary{0.0, 10.0, 0.0};

    expectEqual(overshoot_segment.configure(high_velocity_boundary, high_velocity_boundary, 1.0),
                QuinticConfigurationResult::Success, "position-overshoot segment configures");

    const JointLimit overshoot_limits{-1.0, 1.0, -100.0, 100.0, -1000.0, 1000.0, -10000.0, 10000.0};

    const QuinticValidationStatus overshoot_status =
        QuinticValidator::validate(overshoot_segment, overshoot_limits);

    expectEqual(overshoot_status.result, QuinticValidationResult::PositionLimitExceeded,
                "position overshoot between endpoints is detected");

    expectEqual(overshoot_status.value > overshoot_limits.maximum_position, true,
                "reported position is outside its limit");

    QuinticSegment constant_segment;
    const QuinticBoundaryState constant_boundary{0.0, 0.0, 0.0};

    expectEqual(constant_segment.configure(constant_boundary, constant_boundary, 1.0),
                QuinticConfigurationResult::Success, "constant segment configures");

    const JointLimit exact_zero_limits{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    expectEqual(QuinticValidator::validate(constant_segment, exact_zero_limits).result,
                QuinticValidationResult::Success, "degenerate constant polynomial validates");

    QuinticSegment unconfigured_segment;

    expectEqual(QuinticValidator::validate(unconfigured_segment, safe_limits).result,
                QuinticValidationResult::NotConfigured, "unconfigured segment is rejected");

    JointLimit invalid_limits = safe_limits;
    invalid_limits.minimum_acceleration = 2.0;
    invalid_limits.maximum_acceleration = -2.0;

    expectEqual(QuinticValidator::validate(smooth_step, invalid_limits).result,
                QuinticValidationResult::InvalidLimits, "unordered limits are rejected");

    invalid_limits = safe_limits;
    invalid_limits.maximum_jerk = std::numeric_limits<double>::infinity();

    expectEqual(QuinticValidator::validate(smooth_step, invalid_limits).result,
                QuinticValidationResult::InvalidLimits, "non-finite limits are rejected");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " quintic validator test(s) failed\n";
        return 1;
    }

    std::cout << "All quintic validator tests passed\n";
    return 0;
}
