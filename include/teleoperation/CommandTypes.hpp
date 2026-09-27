#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace teleoperation
{
    inline constexpr std::size_t JointCount = 6;

    using JointPositions = std::array<double, JointCount>;

    using ControllerClock = std::chrono::steady_clock;

    struct SenderTime
    {
        std::int64_t nanoseconds_since_sender_epoch;
    };

    // The controller loop supplies time in its local monotonic domain.
    // It has the same signed-nanosecond representation as SenderTime, but remains
    // a distinct type because the two clocks do not share an epoch.
    struct ControllerTime
    {
        std::int64_t nanoseconds_since_controller_epoch;

        static ControllerTime now() noexcept
        {
            const auto elapsed = ControllerClock::now().time_since_epoch();

            return ControllerTime{
                std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()};
        }
    };

    // Apply different time types for sender and receiver to avoid confusion and
    // potential bugs. We assume the sender timestamps a command before sending it,
    // and the network teleoperation thread adds the receive timestamp.
    struct TargetCommand
    {
        SenderTime sender_time;
        ControllerTime received_time;
        JointPositions positions;
    };
}
