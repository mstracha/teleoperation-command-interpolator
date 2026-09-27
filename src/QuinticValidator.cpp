#include "teleoperation/QuinticValidator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace teleoperation
{
    namespace
    {
        constexpr std::size_t MaximumPolynomialDegree = 5;
        constexpr std::size_t MaximumRootCount = MaximumPolynomialDegree;
        constexpr std::size_t BisectionIterations = 80;

        struct Polynomial
        {
            // Coefficients are stored in ascending power order.
            std::array<double, MaximumPolynomialDegree + 1> coefficients{};
            std::size_t degree{0};
        };

        struct RootSet
        {
            std::array<double, MaximumRootCount> values{};
            std::size_t count{0};
            bool succeeded{true};
        };

        bool normalize(Polynomial &polynomial) noexcept
        {
            while (polynomial.degree > 0 && polynomial.coefficients[polynomial.degree] == 0.0)
            {
                --polynomial.degree;
            }

            double maximum_magnitude = 0.0;

            for (std::size_t coefficient_index = 0; coefficient_index <= polynomial.degree;
                 ++coefficient_index)
            {
                const double coefficient = polynomial.coefficients[coefficient_index];

                if (!std::isfinite(coefficient))
                {
                    return false;
                }

                maximum_magnitude = std::max(maximum_magnitude, std::abs(coefficient));
            }

            if (maximum_magnitude == 0.0)
            {
                polynomial.degree = 0;
                polynomial.coefficients.fill(0.0);
                return true;
            }

            for (std::size_t coefficient_index = 0; coefficient_index <= polynomial.degree;
                 ++coefficient_index)
            {
                polynomial.coefficients[coefficient_index] /= maximum_magnitude;
            }

            return true;
        }

        double evaluate(const Polynomial &polynomial, double value) noexcept
        {
            double result = polynomial.coefficients[polynomial.degree];

            for (std::size_t coefficient_index = polynomial.degree; coefficient_index > 0;
                 --coefficient_index)
            {
                result = result * value + polynomial.coefficients[coefficient_index - 1];
            }

            return result;
        }

        Polynomial derivativeOf(const Polynomial &polynomial) noexcept
        {
            Polynomial derivative{};

            if (polynomial.degree == 0)
            {
                return derivative;
            }

            derivative.degree = polynomial.degree - 1;

            for (std::size_t coefficient_index = 1; coefficient_index <= polynomial.degree;
                 ++coefficient_index)
            {
                derivative.coefficients[coefficient_index - 1] =
                    static_cast<double>(coefficient_index) *
                    polynomial.coefficients[coefficient_index];
            }

            return derivative;
        }

        double zeroTolerance(const Polynomial &polynomial) noexcept
        {
            constexpr double ToleranceMultiplier = 256.0;

            return ToleranceMultiplier * std::numeric_limits<double>::epsilon() *
                   static_cast<double>(polynomial.degree + 1);
        }

        int classifiedSign(double value, double tolerance) noexcept
        {
            if (std::abs(value) <= tolerance)
            {
                return 0;
            }

            return value < 0.0 ? -1 : 1;
        }

        bool addRoot(RootSet &roots, double root) noexcept
        {
            constexpr double MergeTolerance = 256.0 * std::numeric_limits<double>::epsilon();

            const double bounded_root = std::clamp(root, 0.0, 1.0);

            if (roots.count > 0 &&
                std::abs(roots.values[roots.count - 1] - bounded_root) <= MergeTolerance)
            {
                return true;
            }

            if (roots.count >= roots.values.size())
            {
                return false;
            }

            roots.values[roots.count] = bounded_root;
            ++roots.count;
            return true;
        }

        bool bisectRoot(const Polynomial &polynomial, double left, double right, double left_value,
                        double tolerance, double &root) noexcept
        {
            int left_sign = classifiedSign(left_value, tolerance);

            if (left_sign == 0)
            {
                root = left;
                return true;
            }

            for (std::size_t iteration = 0; iteration < BisectionIterations; ++iteration)
            {
                const double midpoint = left + 0.5 * (right - left);
                const double midpoint_value = evaluate(polynomial, midpoint);

                if (!std::isfinite(midpoint_value))
                {
                    return false;
                }

                const int midpoint_sign = classifiedSign(midpoint_value, tolerance);

                if (midpoint_sign == 0)
                {
                    root = midpoint;
                    return true;
                }

                if (midpoint_sign == left_sign)
                {
                    left = midpoint;
                    left_sign = midpoint_sign;
                }
                else
                {
                    right = midpoint;
                }
            }

            root = left + 0.5 * (right - left);
            return std::isfinite(root);
        }

        RootSet findRootsOnUnitInterval(Polynomial polynomial) noexcept
        {
            RootSet roots{};

            if (!normalize(polynomial))
            {
                roots.succeeded = false;
                return roots;
            }

            if (polynomial.degree == 0)
            {
                return roots;
            }

            if (polynomial.degree == 1)
            {
                const double root = -polynomial.coefficients[0] / polynomial.coefficients[1];

                if (!std::isfinite(root))
                {
                    // A non-finite linear root lies outside the finite interval.
                    return roots;
                }

                constexpr double IntervalTolerance = 256.0 * std::numeric_limits<double>::epsilon();

                if (root >= -IntervalTolerance && root <= 1.0 + IntervalTolerance)
                {
                    roots.succeeded = addRoot(roots, root);
                }

                return roots;
            }

            const RootSet critical_points = findRootsOnUnitInterval(derivativeOf(polynomial));

            if (!critical_points.succeeded)
            {
                roots.succeeded = false;
                return roots;
            }

            std::array<double, MaximumRootCount + 2> partition_points{};
            std::size_t partition_count = 0;
            partition_points[partition_count++] = 0.0;

            for (std::size_t root_index = 0; root_index < critical_points.count; ++root_index)
            {
                const double critical_point = critical_points.values[root_index];

                if (critical_point > 0.0 && critical_point < 1.0)
                {
                    partition_points[partition_count++] = critical_point;
                }
            }

            partition_points[partition_count++] = 1.0;
            const double tolerance = zeroTolerance(polynomial);

            for (std::size_t point_index = 0; point_index < partition_count; ++point_index)
            {
                const double point = partition_points[point_index];
                const double point_value = evaluate(polynomial, point);

                if (!std::isfinite(point_value))
                {
                    roots.succeeded = false;
                    return roots;
                }

                const int point_sign = classifiedSign(point_value, tolerance);

                if (point_sign == 0 && !addRoot(roots, point))
                {
                    roots.succeeded = false;
                    return roots;
                }

                if (point_index + 1 >= partition_count)
                {
                    continue;
                }

                const double next_point = partition_points[point_index + 1];
                const double next_value = evaluate(polynomial, next_point);

                if (!std::isfinite(next_value))
                {
                    roots.succeeded = false;
                    return roots;
                }

                const int next_sign = classifiedSign(next_value, tolerance);

                if (point_sign == 0 || next_sign == 0 || point_sign == next_sign)
                {
                    continue;
                }

                double root = 0.0;

                if (!bisectRoot(polynomial, point, next_point, point_value, tolerance, root) ||
                    !addRoot(roots, root))
                {
                    roots.succeeded = false;
                    return roots;
                }
            }

            return roots;
        }

        bool isFinite(const JointLimit &limits) noexcept
        {
            return std::isfinite(limits.minimum_position) &&
                   std::isfinite(limits.maximum_position) &&
                   std::isfinite(limits.minimum_velocity) &&
                   std::isfinite(limits.maximum_velocity) &&
                   std::isfinite(limits.minimum_acceleration) &&
                   std::isfinite(limits.maximum_acceleration) &&
                   std::isfinite(limits.minimum_jerk) && std::isfinite(limits.maximum_jerk);
        }

        bool hasOrderedBounds(const JointLimit &limits) noexcept
        {
            return limits.minimum_position <= limits.maximum_position &&
                   limits.minimum_velocity <= limits.maximum_velocity &&
                   limits.minimum_acceleration <= limits.maximum_acceleration &&
                   limits.minimum_jerk <= limits.maximum_jerk;
        }

        enum class Quantity : std::uint8_t
        {
            Position,
            Velocity,
            Acceleration,
            Jerk
        };

        double quantityValue(const QuinticSample &sample, Quantity quantity) noexcept
        {
            switch (quantity)
            {
                case Quantity::Position:
                    return sample.position;

                case Quantity::Velocity:
                    return sample.velocity;

                case Quantity::Acceleration:
                    return sample.acceleration;

                case Quantity::Jerk:
                    return sample.jerk;
            }

            return std::numeric_limits<double>::quiet_NaN();
        }

        QuinticValidationResult violationResult(Quantity quantity) noexcept
        {
            switch (quantity)
            {
                case Quantity::Position:
                    return QuinticValidationResult::PositionLimitExceeded;

                case Quantity::Velocity:
                    return QuinticValidationResult::VelocityLimitExceeded;

                case Quantity::Acceleration:
                    return QuinticValidationResult::AccelerationLimitExceeded;

                case Quantity::Jerk:
                    return QuinticValidationResult::JerkLimitExceeded;
            }

            return QuinticValidationResult::NumericalFailure;
        }

        QuinticValidationStatus validateQuantity(const QuinticSegment &segment,
                                                 const Polynomial &extrema_polynomial,
                                                 Quantity quantity, double minimum,
                                                 double maximum) noexcept
        {
            const RootSet extrema = findRootsOnUnitInterval(extrema_polynomial);

            if (!extrema.succeeded)
            {
                return {QuinticValidationResult::NumericalFailure, 0.0, 0.0};
            }

            std::array<double, MaximumRootCount + 2> candidates{};
            std::size_t candidate_count = 0;
            candidates[candidate_count++] = 0.0;

            for (std::size_t root_index = 0; root_index < extrema.count; ++root_index)
            {
                const double normalized_time = extrema.values[root_index];

                if (normalized_time > 0.0 && normalized_time < 1.0)
                {
                    candidates[candidate_count++] = normalized_time;
                }
            }

            candidates[candidate_count++] = 1.0;

            for (std::size_t candidate_index = 0; candidate_index < candidate_count;
                 ++candidate_index)
            {
                const double time_seconds = candidates[candidate_index] * segment.durationSeconds();

                QuinticSample sample{};

                if (segment.evaluate(time_seconds, sample) != QuinticEvaluationResult::Success)
                {
                    return {QuinticValidationResult::NumericalFailure, time_seconds, 0.0};
                }

                const double value = quantityValue(sample, quantity);

                if (!std::isfinite(value))
                {
                    return {QuinticValidationResult::NumericalFailure, time_seconds, value};
                }

                if (value < minimum || value > maximum)
                {
                    return {violationResult(quantity), time_seconds, value};
                }
            }

            return {QuinticValidationResult::Success, 0.0, 0.0};
        }
    }

    QuinticValidationStatus QuinticValidator::validate(const QuinticSegment &segment,
                                                       const JointLimit &limits) noexcept
    {
        if (!segment.configured_)
        {
            return {QuinticValidationResult::NotConfigured, 0.0, 0.0};
        }

        if (!isFinite(limits) || !hasOrderedBounds(limits))
        {
            return {QuinticValidationResult::InvalidLimits, 0.0, 0.0};
        }

        Polynomial normalized_position{};
        normalized_position.degree = MaximumPolynomialDegree;
        double duration_power = 1.0;

        for (std::size_t coefficient_index = 0; coefficient_index <= MaximumPolynomialDegree;
             ++coefficient_index)
        {
            normalized_position.coefficients[coefficient_index] =
                segment.coefficients_[coefficient_index] * duration_power;

            if (coefficient_index < MaximumPolynomialDegree)
            {
                duration_power *= segment.duration_seconds_;
            }
        }

        if (!normalize(normalized_position))
        {
            return {QuinticValidationResult::NumericalFailure, 0.0, 0.0};
        }

        const Polynomial normalized_velocity = derivativeOf(normalized_position);

        const Polynomial normalized_acceleration = derivativeOf(normalized_velocity);

        const Polynomial normalized_jerk = derivativeOf(normalized_acceleration);

        const Polynomial normalized_snap = derivativeOf(normalized_jerk);

        QuinticValidationStatus status =
            validateQuantity(segment, normalized_velocity, Quantity::Position,
                             limits.minimum_position, limits.maximum_position);

        if (!status.succeeded())
        {
            return status;
        }

        status = validateQuantity(segment, normalized_acceleration, Quantity::Velocity,
                                  limits.minimum_velocity, limits.maximum_velocity);

        if (!status.succeeded())
        {
            return status;
        }

        status = validateQuantity(segment, normalized_jerk, Quantity::Acceleration,
                                  limits.minimum_acceleration, limits.maximum_acceleration);

        if (!status.succeeded())
        {
            return status;
        }

        return validateQuantity(segment, normalized_snap, Quantity::Jerk, limits.minimum_jerk,
                                limits.maximum_jerk);
    }
}
