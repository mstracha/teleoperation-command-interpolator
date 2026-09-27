#pragma once

#include "teleoperation/CommandTypes.hpp"

#include <array>

namespace teleoperation
{
    struct JointLimit
    {
        constexpr JointLimit(double minimum_position_value, double maximum_position_value,
                             double minimum_velocity_value, double maximum_velocity_value,
                             double minimum_acceleration_value, double maximum_acceleration_value,
                             double minimum_jerk_value, double maximum_jerk_value)
            : minimum_position(minimum_position_value), maximum_position(maximum_position_value),
              minimum_velocity(minimum_velocity_value), maximum_velocity(maximum_velocity_value),
              minimum_acceleration(minimum_acceleration_value),
              maximum_acceleration(maximum_acceleration_value), minimum_jerk(minimum_jerk_value),
              maximum_jerk(maximum_jerk_value)
        {
        }

        double minimum_position;
        double maximum_position;

        double minimum_velocity;
        double maximum_velocity;

        double minimum_acceleration;
        double maximum_acceleration;

        double minimum_jerk;
        double maximum_jerk;
    };

    using JointLimits = std::array<JointLimit, JointCount>;

    // Fictional test-only limits for six revolute joints. Units are radians,
    // radians/second, radians/second^2, and radians/second^3. These values are not
    // safe specifications for hardware.
    inline constexpr JointLimits DefaultToyJointLimits{
        {JointLimit{-2.80, 2.80, -2.00, 2.00, -5.00, 5.00, -20.00, 20.00},
         JointLimit{-2.40, 2.40, -1.80, 1.80, -4.50, 4.50, -18.00, 18.00},
         JointLimit{-2.00, 2.00, -1.60, 1.60, -4.00, 4.00, -16.00, 16.00},
         JointLimit{-3.00, 3.00, -2.20, 2.20, -6.00, 6.00, -24.00, 24.00},
         JointLimit{-2.20, 2.20, -1.50, 1.50, -3.50, 3.50, -14.00, 14.00},
         JointLimit{-3.14, 3.14, -2.50, 2.50, -7.00, 7.00, -28.00, 28.00}}};
}
