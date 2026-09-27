#pragma once

#include <array>
#include <cstdint>

namespace teleoperation
{
    class QuinticValidator;

    struct QuinticBoundaryState
    {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
    };

    struct QuinticSample
    {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double jerk{0.0};
    };

    enum class QuinticConfigurationResult : std::uint8_t
    {
        Success,
        InvalidDuration,
        NonFiniteBoundary,
        NonFiniteCoefficients
    };

    enum class QuinticEvaluationResult : std::uint8_t
    {
        Success,
        NotConfigured,
        NonFiniteTime,
        TimeBeforeSegment,
        TimeAfterSegment,
        NonFiniteSample
    };

    // One-dimensional quintic polynomial using seconds as its time unit.
    // Configuration and evaluation are fixed-cost and allocation-free.
    class QuinticSegment
    {
      public:
        QuinticConfigurationResult configure(const QuinticBoundaryState &start,
                                             const QuinticBoundaryState &end,
                                             double duration_seconds) noexcept;

        QuinticEvaluationResult evaluate(double elapsed_seconds,
                                         QuinticSample &sample) const noexcept;

        bool isConfigured() const noexcept;
        double durationSeconds() const noexcept;

      private:
        friend class QuinticValidator;

        std::array<double, 6> coefficients_{};
        double duration_seconds_{0.0};
        bool configured_{false};
    };
}
