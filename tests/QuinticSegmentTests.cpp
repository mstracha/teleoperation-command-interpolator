#include "teleoperation/QuinticSegment.hpp"

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

    void expectBoundary(const QuinticSample &sample, const QuinticBoundaryState &boundary,
                        const char *test_name)
    {
        constexpr double Tolerance = 1.0e-11;

        expectNear(sample.position, boundary.position, Tolerance, test_name);
        expectNear(sample.velocity, boundary.velocity, Tolerance, test_name);
        expectNear(sample.acceleration, boundary.acceleration, Tolerance, test_name);
    }
}

int main()
{
    QuinticSegment smooth_step;
    const QuinticBoundaryState zero_rest{0.0, 0.0, 0.0};
    const QuinticBoundaryState one_rest{1.0, 0.0, 0.0};

    expectEqual(smooth_step.configure(zero_rest, one_rest, 1.0),
                QuinticConfigurationResult::Success, "unit smooth-step segment configures");

    expectEqual(smooth_step.isConfigured(), true, "successful segment reports configured");

    expectNear(smooth_step.durationSeconds(), 1.0, 0.0, "duration is preserved");

    QuinticSample sample{};

    expectEqual(smooth_step.evaluate(0.0, sample), QuinticEvaluationResult::Success,
                "unit segment evaluates at start");

    expectBoundary(sample, zero_rest, "unit segment start boundary");
    expectNear(sample.jerk, 60.0, 1.0e-12, "unit segment start jerk");

    expectEqual(smooth_step.evaluate(0.5, sample), QuinticEvaluationResult::Success,
                "unit segment evaluates at midpoint");

    expectNear(sample.position, 0.5, 1.0e-12, "midpoint position");
    expectNear(sample.velocity, 1.875, 1.0e-12, "midpoint velocity");
    expectNear(sample.acceleration, 0.0, 1.0e-12, "midpoint acceleration");
    expectNear(sample.jerk, -30.0, 1.0e-12, "midpoint jerk");

    expectEqual(smooth_step.evaluate(1.0, sample), QuinticEvaluationResult::Success,
                "unit segment evaluates at end");

    expectBoundary(sample, one_rest, "unit segment end boundary");
    expectNear(sample.jerk, 60.0, 1.0e-12, "unit segment end jerk");

    QuinticSegment arbitrary_segment;
    const QuinticBoundaryState arbitrary_start{2.0, -0.5, 0.25};
    const QuinticBoundaryState arbitrary_end{-1.0, 0.75, -0.4};

    expectEqual(arbitrary_segment.configure(arbitrary_start, arbitrary_end, 2.0),
                QuinticConfigurationResult::Success, "arbitrary-boundary segment configures");

    expectEqual(arbitrary_segment.evaluate(0.0, sample), QuinticEvaluationResult::Success,
                "arbitrary segment evaluates at start");

    expectBoundary(sample, arbitrary_start, "arbitrary start boundary");

    expectEqual(arbitrary_segment.evaluate(2.0, sample), QuinticEvaluationResult::Success,
                "arbitrary segment evaluates at end");

    expectBoundary(sample, arbitrary_end, "arbitrary end boundary");

    const QuinticBoundaryState shared_boundary{1.0, 0.5, -0.2};
    QuinticSegment first_joined_segment;
    QuinticSegment second_joined_segment;

    expectEqual(first_joined_segment.configure(zero_rest, shared_boundary, 1.25),
                QuinticConfigurationResult::Success, "first joined segment configures");

    expectEqual(second_joined_segment.configure(shared_boundary,
                                                QuinticBoundaryState{1.5, -0.1, 0.3}, 0.75),
                QuinticConfigurationResult::Success, "second joined segment configures");

    QuinticSample first_join_sample{};
    QuinticSample second_join_sample{};

    expectEqual(first_joined_segment.evaluate(1.25, first_join_sample),
                QuinticEvaluationResult::Success, "first segment evaluates at join");

    expectEqual(second_joined_segment.evaluate(0.0, second_join_sample),
                QuinticEvaluationResult::Success, "second segment evaluates at join");

    expectNear(first_join_sample.position, second_join_sample.position, 1.0e-11,
               "joined position is continuous");

    expectNear(first_join_sample.velocity, second_join_sample.velocity, 1.0e-11,
               "joined velocity is continuous");

    expectNear(first_join_sample.acceleration, second_join_sample.acceleration, 1.0e-11,
               "joined acceleration is continuous");

    QuinticSegment invalid_segment;
    QuinticSample unchanged_sample{7.0, 8.0, 9.0, 10.0};

    expectEqual(invalid_segment.evaluate(0.0, unchanged_sample),
                QuinticEvaluationResult::NotConfigured, "unconfigured segment cannot evaluate");

    expectNear(unchanged_sample.position, 7.0, 0.0, "failed evaluation leaves output unchanged");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, 0.0),
                QuinticConfigurationResult::InvalidDuration, "zero duration is rejected");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, -1.0),
                QuinticConfigurationResult::InvalidDuration, "negative duration is rejected");

    expectEqual(
        invalid_segment.configure(zero_rest, one_rest, std::numeric_limits<double>::infinity()),
        QuinticConfigurationResult::InvalidDuration, "infinite duration is rejected");

    expectEqual(invalid_segment.configure(
                    QuinticBoundaryState{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                    one_rest, 1.0),
                QuinticConfigurationResult::NonFiniteBoundary, "non-finite boundary is rejected");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, std::numeric_limits<double>::min()),
                QuinticConfigurationResult::NonFiniteCoefficients,
                "numerically unsafe duration is rejected");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, 1.0),
                QuinticConfigurationResult::Success,
                "segment can recover through valid reconfiguration");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, 0.0),
                QuinticConfigurationResult::InvalidDuration, "failed reconfiguration is reported");

    expectEqual(invalid_segment.isConfigured(), false,
                "failed reconfiguration invalidates the old segment");

    expectEqual(invalid_segment.evaluate(0.5, unchanged_sample),
                QuinticEvaluationResult::NotConfigured,
                "old coefficients cannot be evaluated after failed reconfiguration");

    expectEqual(invalid_segment.configure(zero_rest, one_rest, 1.0),
                QuinticConfigurationResult::Success, "segment can configure again after failure");

    expectEqual(invalid_segment.evaluate(-0.001, unchanged_sample),
                QuinticEvaluationResult::TimeBeforeSegment, "time before segment is rejected");

    expectEqual(invalid_segment.evaluate(1.001, unchanged_sample),
                QuinticEvaluationResult::TimeAfterSegment, "time after segment is rejected");

    expectEqual(
        invalid_segment.evaluate(std::numeric_limits<double>::quiet_NaN(), unchanged_sample),
        QuinticEvaluationResult::NonFiniteTime, "non-finite evaluation time is rejected");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " quintic test(s) failed\n";
        return 1;
    }

    std::cout << "All quintic segment tests passed\n";
    return 0;
}
