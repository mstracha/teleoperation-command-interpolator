#pragma once

#include <chrono>

namespace teleoperation
{
    struct InterpolatorConfiguration
    {
        // Toy values for the test harness; these are not hardware specifications.
        std::chrono::nanoseconds minimum_message_interval{std::chrono::milliseconds{10}};

        std::chrono::nanoseconds maximum_sender_interval{std::chrono::milliseconds{100}};

        std::chrono::nanoseconds hold_timeout{std::chrono::milliseconds{150}};

        std::chrono::nanoseconds fault_timeout{std::chrono::milliseconds{500}};

        constexpr bool isValid() const noexcept
        {
            return minimum_message_interval.count() >= 0 &&
                   maximum_sender_interval > minimum_message_interval && hold_timeout.count() > 0 &&
                   fault_timeout > hold_timeout;
        }
    };
}
