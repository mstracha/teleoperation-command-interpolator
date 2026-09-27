#pragma once

namespace teleoperation
{
    enum class CommandResult
    {
        Accepted,
        RejectedPositionNotFinite,
        RejectedPositionOutOfRange,
        RejectedDuplicateOrOutOfOrder,
        RejectedIntervalTooSmall,
        RejectedExcessiveSenderGap,
        RejectedBufferFull
    };

    enum class ControlResult
    {
        Buffering,
        TargetProduced,
        Holding,
        Faulted
    };
}
