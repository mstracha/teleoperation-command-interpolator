#include "teleoperation/QuinticSegment.hpp"

#include <cmath>

namespace teleoperation
{
    namespace
    {
        bool isFinite(const QuinticBoundaryState &state) noexcept
        {
            return std::isfinite(state.position) && std::isfinite(state.velocity) &&
                   std::isfinite(state.acceleration);
        }

        bool isFinite(const QuinticSample &sample) noexcept
        {
            return std::isfinite(sample.position) && std::isfinite(sample.velocity) &&
                   std::isfinite(sample.acceleration) && std::isfinite(sample.jerk);
        }
    }

    QuinticConfigurationResult QuinticSegment::configure(const QuinticBoundaryState &start,
                                                         const QuinticBoundaryState &end,
                                                         double duration_seconds) noexcept
    {
        configured_ = false;
        duration_seconds_ = 0.0;

        if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0)
        {
            return QuinticConfigurationResult::InvalidDuration;
        }

        if (!isFinite(start) || !isFinite(end))
        {
            return QuinticConfigurationResult::NonFiniteBoundary;
        }

        const double duration_squared = duration_seconds * duration_seconds;
        const double duration_cubed = duration_squared * duration_seconds;
        const double duration_fourth = duration_cubed * duration_seconds;
        const double duration_fifth = duration_fourth * duration_seconds;

        const double position_remainder =
            end.position - (start.position + start.velocity * duration_seconds +
                            0.5 * start.acceleration * duration_squared);

        const double velocity_remainder =
            end.velocity - (start.velocity + start.acceleration * duration_seconds);

        const double acceleration_remainder = end.acceleration - start.acceleration;

        std::array<double, 6> coefficients{};
        coefficients[0] = start.position;
        coefficients[1] = start.velocity;
        coefficients[2] = 0.5 * start.acceleration;

        coefficients[3] = (10.0 * position_remainder - 4.0 * velocity_remainder * duration_seconds +
                           0.5 * acceleration_remainder * duration_squared) /
                          duration_cubed;

        coefficients[4] =
            (-15.0 * position_remainder + 7.0 * velocity_remainder * duration_seconds -
             acceleration_remainder * duration_squared) /
            duration_fourth;

        coefficients[5] = (6.0 * position_remainder - 3.0 * velocity_remainder * duration_seconds +
                           0.5 * acceleration_remainder * duration_squared) /
                          duration_fifth;

        for (double coefficient : coefficients)
        {
            if (!std::isfinite(coefficient))
            {
                return QuinticConfigurationResult::NonFiniteCoefficients;
            }
        }

        coefficients_ = coefficients;
        duration_seconds_ = duration_seconds;
        configured_ = true;
        return QuinticConfigurationResult::Success;
    }

    QuinticEvaluationResult QuinticSegment::evaluate(double elapsed_seconds,
                                                     QuinticSample &sample) const noexcept
    {
        if (!configured_)
        {
            return QuinticEvaluationResult::NotConfigured;
        }

        if (!std::isfinite(elapsed_seconds))
        {
            return QuinticEvaluationResult::NonFiniteTime;
        }

        if (elapsed_seconds < 0.0)
        {
            return QuinticEvaluationResult::TimeBeforeSegment;
        }

        if (elapsed_seconds > duration_seconds_)
        {
            return QuinticEvaluationResult::TimeAfterSegment;
        }

        const double time = elapsed_seconds;
        const double a0 = coefficients_[0];
        const double a1 = coefficients_[1];
        const double a2 = coefficients_[2];
        const double a3 = coefficients_[3];
        const double a4 = coefficients_[4];
        const double a5 = coefficients_[5];

        QuinticSample evaluated{};

        evaluated.position = ((((a5 * time + a4) * time + a3) * time + a2) * time + a1) * time + a0;

        evaluated.velocity =
            (((5.0 * a5 * time + 4.0 * a4) * time + 3.0 * a3) * time + 2.0 * a2) * time + a1;

        evaluated.acceleration =
            ((20.0 * a5 * time + 12.0 * a4) * time + 6.0 * a3) * time + 2.0 * a2;

        evaluated.jerk = (60.0 * a5 * time + 24.0 * a4) * time + 6.0 * a3;

        if (!isFinite(evaluated))
        {
            return QuinticEvaluationResult::NonFiniteSample;
        }

        sample = evaluated;
        return QuinticEvaluationResult::Success;
    }

    bool QuinticSegment::isConfigured() const noexcept
    {
        return configured_;
    }

    double QuinticSegment::durationSeconds() const noexcept
    {
        return duration_seconds_;
    }
}
