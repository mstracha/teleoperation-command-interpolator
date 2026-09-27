#pragma once

#include "teleoperation/JointLimits.hpp"
#include "teleoperation/QuinticSegment.hpp"

#include <cstdint>

namespace teleoperation
{
    enum class QuinticValidationResult : std::uint8_t
    {
        Success,
        NotConfigured,
        InvalidLimits,
        NumericalFailure,
        PositionLimitExceeded,
        VelocityLimitExceeded,
        AccelerationLimitExceeded,
        JerkLimitExceeded
    };

    struct QuinticValidationStatus
    {
        QuinticValidationResult result{QuinticValidationResult::Success};
        double time_seconds{0.0};
        double value{0.0};

        bool succeeded() const noexcept
        {
            return result == QuinticValidationResult::Success;
        }
    };

    // Validates the complete continuous interval, including all extrema between
    // controller samples. It isolates polynomial roots using fixed-size storage
    // and bounded bisection; no allocation or unbounded iteration is performed.
    class QuinticValidator
    {
      public:
        static QuinticValidationStatus validate(const QuinticSegment &segment,
                                                const JointLimit &limits) noexcept;
    };
}
