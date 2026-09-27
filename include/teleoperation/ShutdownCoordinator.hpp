#pragma once

#include "teleoperation/FaultReason.hpp"

#include <atomic>
#include <cstdint>

namespace teleoperation
{
    enum class ShutdownPhase : std::uint8_t
    {
        Running,
        StopRequested,
        DrainingLogs,
        Complete
    };

    // Coordinates process lifecycle without owning or forcibly terminating any
    // thread. Each thread observes the phase, completes its responsibility, and
    // reports completion to the coordinator.
    class ShutdownCoordinator
    {
      public:
        // Requests an orderly non-fault shutdown. Returns true only for the call
        // that changes Running to StopRequested.
        bool requestStop() noexcept;

        // Requests fault-driven shutdown and latches the first non-None reason.
        bool requestStop(FaultReason reason) noexcept;

        bool stopRequested() const noexcept;
        ShutdownPhase phase() const noexcept;
        FaultReason faultReason() const noexcept;

        void markControllerStopped() noexcept;
        void markNetworkStopped() noexcept;
        bool producersStopped() const noexcept;

        // The logger may begin draining as soon as a stop is requested. It should
        // call markLoggingComplete only after both telemetry buffers are empty.
        bool beginLogDrain() noexcept;
        bool markLoggingComplete() noexcept;

      private:
        std::atomic<ShutdownPhase> phase_{ShutdownPhase::Running};
        std::atomic<FaultReason> fault_reason_{FaultReason::None};
        std::atomic<bool> controller_stopped_{false};
        std::atomic<bool> network_stopped_{false};

        static_assert(std::atomic<ShutdownPhase>::is_always_lock_free,
                      "Shutdown phase transitions must be lock-free");

        static_assert(std::atomic<FaultReason>::is_always_lock_free,
                      "Shutdown fault latching must be lock-free");

        static_assert(std::atomic<bool>::is_always_lock_free,
                      "Shutdown completion flags must be lock-free");
    };
}
