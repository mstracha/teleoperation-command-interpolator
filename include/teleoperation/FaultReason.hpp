#pragma once

#include <cstdint>

namespace teleoperation
{
    enum class FaultReason : std::uint8_t
    {
        None,
        ExcessiveSenderGap,
        ReceiveTimeout,
        ControllerTimeRegression,
        ControllerTimeBeforeLatestReceive,
        CommandBufferInconsistent,
        InvalidSegmentTiming,
        InvalidStateTransition,
        MissingHoldTarget,
        AdvanceLimitExceeded,
        DerivativeEstimationFailed,
        TrajectoryConfigurationFailed,
        TrajectoryPositionLimitExceeded,
        TrajectoryVelocityLimitExceeded,
        TrajectoryAccelerationLimitExceeded,
        TrajectoryJerkLimitExceeded,
        TrajectoryEvaluationFailed
    };
}
