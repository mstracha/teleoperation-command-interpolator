#include "teleoperation/ShutdownCoordinator.hpp"

namespace teleoperation
{
    bool ShutdownCoordinator::requestStop() noexcept
    {
        ShutdownPhase expected_phase = ShutdownPhase::Running;

        return phase_.compare_exchange_strong(expected_phase, ShutdownPhase::StopRequested,
                                              std::memory_order_acq_rel, std::memory_order_acquire);
    }

    bool ShutdownCoordinator::requestStop(FaultReason reason) noexcept
    {
        if (reason == FaultReason::None)
        {
            return false;
        }

        FaultReason expected_reason = FaultReason::None;

        static_cast<void>(fault_reason_.compare_exchange_strong(
            expected_reason, reason, std::memory_order_acq_rel, std::memory_order_acquire));

        return requestStop();
    }

    bool ShutdownCoordinator::stopRequested() const noexcept
    {
        return phase_.load(std::memory_order_acquire) != ShutdownPhase::Running;
    }

    ShutdownPhase ShutdownCoordinator::phase() const noexcept
    {
        return phase_.load(std::memory_order_acquire);
    }

    FaultReason ShutdownCoordinator::faultReason() const noexcept
    {
        return fault_reason_.load(std::memory_order_acquire);
    }

    void ShutdownCoordinator::markControllerStopped() noexcept
    {
        controller_stopped_.store(true, std::memory_order_release);
    }

    void ShutdownCoordinator::markNetworkStopped() noexcept
    {
        network_stopped_.store(true, std::memory_order_release);
    }

    bool ShutdownCoordinator::producersStopped() const noexcept
    {
        return controller_stopped_.load(std::memory_order_acquire) &&
               network_stopped_.load(std::memory_order_acquire);
    }

    bool ShutdownCoordinator::beginLogDrain() noexcept
    {
        ShutdownPhase expected_phase = ShutdownPhase::StopRequested;

        return phase_.compare_exchange_strong(expected_phase, ShutdownPhase::DrainingLogs,
                                              std::memory_order_acq_rel, std::memory_order_acquire);
    }

    bool ShutdownCoordinator::markLoggingComplete() noexcept
    {
        if (!producersStopped())
        {
            return false;
        }

        ShutdownPhase expected_phase = ShutdownPhase::DrainingLogs;

        return phase_.compare_exchange_strong(expected_phase, ShutdownPhase::Complete,
                                              std::memory_order_acq_rel, std::memory_order_acquire);
    }
}
