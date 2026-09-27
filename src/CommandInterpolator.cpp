#include "teleoperation/CommandInterpolator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace teleoperation
{
    namespace
    {
        std::uint64_t positiveDifference(std::int64_t later, std::int64_t earlier) noexcept
        {
            return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
        }
    }

    CommandInterpolator::CommandInterpolator(const JointLimits &joint_limits)
        : joint_limits_(joint_limits)
    {
    }

    CommandInterpolator::CommandInterpolator(const JointLimits &joint_limits,
                                             const InterpolatorConfiguration &configuration)
        : joint_limits_(joint_limits), configuration_(configuration)
    {
        if (!configuration_.isValid())
        {
            throw std::invalid_argument("Invalid interpolator configuration");
        }
    }

    CommandResult CommandInterpolator::submitCommand(TargetCommand cmd)
    {
        cmd.received_time = ControllerTime::now();
        const CommandResult result = acceptCommand(cmd);
        recordInputTelemetry(cmd, result);
        return result;
    }

    CommandResult CommandInterpolator::submitCommand(TargetCommand cmd,
                                                     ControllerTime received_time)
    {
        cmd.received_time = received_time;
        const CommandResult result = acceptCommand(cmd);
        recordInputTelemetry(cmd, result);
        return result;
    }

    CommandResult CommandInterpolator::acceptCommand(const TargetCommand &command)
    {
        const auto sender_time = command.sender_time.nanoseconds_since_sender_epoch;

        const CommandResult validation_result = validateCommand(command);

        if (validation_result != CommandResult::Accepted)
        {
            return validation_result;
        }

        // If the network has reordered messages, discard the out-of-order
        // message. A matching sender time is treated as a duplicate.
        if (has_newest_sender_time_ && sender_time <= newest_sender_time_)
        {
            return CommandResult::RejectedDuplicateOrOutOfOrder;
        }

        if (has_newest_sender_time_)
        {
            // The signed timestamps are already known to be ordered. Unsigned
            // subtraction represents their full non-negative separation without
            // risking signed overflow at the int64 boundaries.
            const std::uint64_t sender_interval = static_cast<std::uint64_t>(sender_time) -
                                                  static_cast<std::uint64_t>(newest_sender_time_);

            const auto minimum_interval =
                static_cast<std::uint64_t>(configuration_.minimum_message_interval.count());

            const auto maximum_interval =
                static_cast<std::uint64_t>(configuration_.maximum_sender_interval.count());

            if (sender_interval <= minimum_interval)
            {
                return CommandResult::RejectedIntervalTooSmall;
            }

            if (sender_interval > maximum_interval)
            {
                FaultReason expected_reason = FaultReason::None;

                static_cast<void>(pending_fault_reason_.compare_exchange_strong(
                    expected_reason, FaultReason::ExcessiveSenderGap, std::memory_order_release,
                    std::memory_order_relaxed));

                return CommandResult::RejectedExcessiveSenderGap;
            }
        }

        if (!command_queue_.tryPush(command))
        {
            return CommandResult::RejectedBufferFull;
        }

        // Ordering is measured against the newest command successfully
        // published to the consumer, not against a command that was dropped.
        newest_sender_time_ = sender_time;
        has_newest_sender_time_ = true;

        latest_receive_time_nanoseconds_.store(
            command.received_time.nanoseconds_since_controller_epoch, std::memory_order_release);

        has_accepted_receive_time_.store(true, std::memory_order_release);

        return CommandResult::Accepted;
    }

    ControlResult CommandInterpolator::updateTarget(ControllerTime current_time,
                                                    std::array<double, 6> &output_target)
    {
        const ControlResult result = calculateNextInterpolatedCommand(current_time, output_target);

        recordControlTelemetry(current_time, result, output_target);
        return result;
    }

    ControlResult CommandInterpolator::calculateNextInterpolatedCommand(
        ControllerTime current_time, std::array<double, 6> &output_target)
    {
        const FaultReason pending_fault_reason =
            pending_fault_reason_.exchange(FaultReason::None, std::memory_order_acquire);

        if (pending_fault_reason != FaultReason::None)
        {
            return enterFaultedState(pending_fault_reason);
        }

        if (state_machine_.state() == InterpolatorState::Faulted)
        {
            return ControlResult::Faulted;
        }

        const std::int64_t controller_time = current_time.nanoseconds_since_controller_epoch;

        if (has_last_controller_time_ && controller_time < last_controller_time_nanoseconds_)
        {
            return enterFaultedState(FaultReason::ControllerTimeRegression);
        }

        last_controller_time_nanoseconds_ = controller_time;
        has_last_controller_time_ = true;

        if (has_accepted_receive_time_.load(std::memory_order_acquire))
        {
            const std::int64_t latest_receive_time =
                latest_receive_time_nanoseconds_.load(std::memory_order_acquire);

            // The network producer can publish a receive timestamp after the
            // controller sampled current_time but before this load. That is a
            // normal concurrent arrival, so its age for this tick is zero.
            const std::uint64_t time_since_latest_receive =
                controller_time < latest_receive_time
                    ? std::uint64_t{0}
                    : positiveDifference(controller_time, latest_receive_time);

            const auto fault_timeout =
                static_cast<std::uint64_t>(configuration_.fault_timeout.count());

            if (time_since_latest_receive >= fault_timeout)
            {
                return enterFaultedState(FaultReason::ReceiveTimeout);
            }

            const auto hold_timeout =
                static_cast<std::uint64_t>(configuration_.hold_timeout.count());

            if (state_machine_.state() != InterpolatorState::Buffering &&
                time_since_latest_receive >= hold_timeout)
            {
                if (!has_last_target_)
                {
                    return enterFaultedState(FaultReason::MissingHoldTarget);
                }

                if (state_machine_.transitionTo(InterpolatorState::Holding) ==
                    StateTransitionResult::Rejected)
                {
                    return enterFaultedState(FaultReason::InvalidStateTransition);
                }

                publishTarget(last_target_positions_, output_target);
                return ControlResult::Holding;
            }
        }

        if (!timeline_initialized_)
        {
            if (!command_queue_.hasInterpolationWindow())
            {
                return ControlResult::Buffering;
            }

            if (!command_queue_.tryGetInterpolationSet(interpolation_window_))
            {
                return enterFaultedState(FaultReason::CommandBufferInconsistent);
            }

            const FaultReason trajectory_fault = configureActiveTrajectory();

            if (trajectory_fault != FaultReason::None)
            {
                return enterFaultedState(trajectory_fault);
            }

            const std::int64_t segment_start_time =
                interpolation_window_[SegmentStartIndex].sender_time.nanoseconds_since_sender_epoch;

            const std::int64_t segment_end_time =
                interpolation_window_[SegmentEndIndex].sender_time.nanoseconds_since_sender_epoch;

            if (segment_end_time <= segment_start_time)
            {
                return enterFaultedState(FaultReason::InvalidSegmentTiming);
            }

            // Keep the unrelated clock epochs separate. Only elapsed intervals
            // from this calibration pair are compared after initialization.
            timeline_controller_origin_nanoseconds_ = controller_time;
            timeline_sender_origin_nanoseconds_ = segment_start_time;

            timeline_initialized_ = true;

            if (state_machine_.transitionTo(InterpolatorState::Running) ==
                StateTransitionResult::Rejected)
            {
                return enterFaultedState(FaultReason::InvalidStateTransition);
            }
        }

        const std::uint64_t controller_elapsed_nanoseconds =
            positiveDifference(controller_time, timeline_controller_origin_nanoseconds_);

        // A controller-time jump may cross more than one buffered segment.
        // CommandBufferCapacity provides a fixed upper bound for this loop.
        for (std::size_t advance_count = 0; advance_count < CommandBufferCapacity; ++advance_count)
        {
            const TargetCommand &segment_start = interpolation_window_[SegmentStartIndex];

            const TargetCommand &segment_end = interpolation_window_[SegmentEndIndex];

            const std::int64_t segment_start_time =
                segment_start.sender_time.nanoseconds_since_sender_epoch;

            const std::int64_t segment_end_time =
                segment_end.sender_time.nanoseconds_since_sender_epoch;

            if (segment_end_time <= segment_start_time)
            {
                return enterFaultedState(FaultReason::InvalidSegmentTiming);
            }

            if (segment_start_time < timeline_sender_origin_nanoseconds_)
            {
                return enterFaultedState(FaultReason::InvalidSegmentTiming);
            }

            const std::uint64_t segment_start_elapsed_nanoseconds =
                positiveDifference(segment_start_time, timeline_sender_origin_nanoseconds_);

            const std::uint64_t segment_end_elapsed_nanoseconds =
                positiveDifference(segment_end_time, timeline_sender_origin_nanoseconds_);

            const std::uint64_t segment_duration_nanoseconds =
                segment_end_elapsed_nanoseconds - segment_start_elapsed_nanoseconds;

            if (segment_duration_nanoseconds >
                static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
            {
                return enterFaultedState(FaultReason::InvalidSegmentTiming);
            }

            current_segment_duration_nanoseconds_ =
                static_cast<std::int64_t>(segment_duration_nanoseconds);

            if (controller_elapsed_nanoseconds < segment_start_elapsed_nanoseconds)
            {
                current_segment_elapsed_nanoseconds_ = 0;

                if (state_machine_.transitionTo(InterpolatorState::Running) ==
                    StateTransitionResult::Rejected)
                {
                    return enterFaultedState(FaultReason::InvalidStateTransition);
                }

                return evaluateActiveTrajectory(current_segment_elapsed_nanoseconds_,
                                                output_target);
            }

            if (controller_elapsed_nanoseconds < segment_end_elapsed_nanoseconds)
            {
                const std::uint64_t segment_elapsed_nanoseconds =
                    controller_elapsed_nanoseconds - segment_start_elapsed_nanoseconds;

                if (segment_elapsed_nanoseconds >
                    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                {
                    return enterFaultedState(FaultReason::InvalidSegmentTiming);
                }

                current_segment_elapsed_nanoseconds_ =
                    static_cast<std::int64_t>(segment_elapsed_nanoseconds);

                if (state_machine_.transitionTo(InterpolatorState::Running) ==
                    StateTransitionResult::Rejected)
                {
                    return enterFaultedState(FaultReason::InvalidStateTransition);
                }

                return evaluateActiveTrajectory(current_segment_elapsed_nanoseconds_,
                                                output_target);
            }

            if (!command_queue_.tryAdvanceInterpolationSet())
            {
                // No complete successor window is available yet. Hold the current
                // endpoint; a richer status will later distinguish this underrun
                // from normal interpolation.
                current_segment_elapsed_nanoseconds_ = current_segment_duration_nanoseconds_;

                publishTarget(segment_end.positions, output_target);

                if (state_machine_.transitionTo(InterpolatorState::Holding) ==
                    StateTransitionResult::Rejected)
                {
                    return enterFaultedState(FaultReason::InvalidStateTransition);
                }

                return ControlResult::Holding;
            }

            if (!command_queue_.tryGetInterpolationSet(interpolation_window_))
            {
                return enterFaultedState(FaultReason::CommandBufferInconsistent);
            }

            const FaultReason trajectory_fault = configureActiveTrajectory();

            if (trajectory_fault != FaultReason::None)
            {
                return enterFaultedState(trajectory_fault);
            }
        }

        // Normally unreachable: the loop returns after finding the active
        // segment or the newest available endpoint.
        return enterFaultedState(FaultReason::AdvanceLimitExceeded);
    }

    FaultReason CommandInterpolator::configureActiveTrajectory() noexcept
    {
        trajectory_failed_command_ = ControlTelemetrySample::NoFailedCommand;
        trajectory_failed_joint_ = ControlTelemetrySample::NoFailedJoint;
        trajectory_failure_time_seconds_ = 0.0;
        trajectory_failure_value_ = 0.0;

        EstimatedJointTrajectorySegment estimate{};
        const DerivativeEstimationStatus estimation_status =
            CommandDerivativeEstimator::estimate(interpolation_window_, estimate);

        if (!estimation_status.succeeded())
        {
            trajectory_failed_command_ = estimation_status.failed_command;
            trajectory_failed_joint_ = estimation_status.failed_joint;
            return FaultReason::DerivativeEstimationFailed;
        }

        const JointTrajectoryConfigurationStatus configuration_status =
            active_trajectory_.configure(estimate.start, estimate.end, estimate.duration_seconds);

        if (!configuration_status.succeeded())
        {
            trajectory_failed_joint_ = configuration_status.failed_joint;
            trajectory_failure_time_seconds_ = configuration_status.failure_time_seconds;
            trajectory_failure_value_ = configuration_status.failure_value;
        }

        switch (configuration_status.result)
        {
            case JointTrajectoryConfigurationResult::Success:
                return FaultReason::None;

            case JointTrajectoryConfigurationResult::BoundaryPositionOutOfRange:
            case JointTrajectoryConfigurationResult::PositionLimitExceeded:
                return FaultReason::TrajectoryPositionLimitExceeded;

            case JointTrajectoryConfigurationResult::BoundaryVelocityOutOfRange:
            case JointTrajectoryConfigurationResult::VelocityLimitExceeded:
                return FaultReason::TrajectoryVelocityLimitExceeded;

            case JointTrajectoryConfigurationResult::BoundaryAccelerationOutOfRange:
            case JointTrajectoryConfigurationResult::AccelerationLimitExceeded:
                return FaultReason::TrajectoryAccelerationLimitExceeded;

            case JointTrajectoryConfigurationResult::JerkLimitExceeded:
                return FaultReason::TrajectoryJerkLimitExceeded;

            case JointTrajectoryConfigurationResult::InvalidJointLimits:
            case JointTrajectoryConfigurationResult::InvalidDuration:
            case JointTrajectoryConfigurationResult::NonFiniteBoundary:
            case JointTrajectoryConfigurationResult::NonFiniteCoefficients:
            case JointTrajectoryConfigurationResult::ValidationNumericalFailure:
                return FaultReason::TrajectoryConfigurationFailed;
        }

        return FaultReason::TrajectoryConfigurationFailed;
    }

    ControlResult CommandInterpolator::evaluateActiveTrajectory(
        std::int64_t elapsed_nanoseconds, std::array<double, 6> &output_target) noexcept
    {
        constexpr double SecondsPerNanosecond = 1.0e-9;
        const double elapsed_seconds =
            static_cast<double>(elapsed_nanoseconds) * SecondsPerNanosecond;

        JointTrajectorySample sample{};
        const JointTrajectoryEvaluationStatus evaluation_status =
            active_trajectory_.evaluate(elapsed_seconds, sample);

        if (!evaluation_status.succeeded())
        {
            trajectory_failed_command_ = ControlTelemetrySample::NoFailedCommand;
            trajectory_failed_joint_ = evaluation_status.failed_joint;
            trajectory_failure_time_seconds_ = elapsed_seconds;
            trajectory_failure_value_ = evaluation_status.failure_value;
        }

        switch (evaluation_status.result)
        {
            case JointTrajectoryEvaluationResult::Success:
                publishTarget(sample.positions, output_target);
                return ControlResult::TargetProduced;

            case JointTrajectoryEvaluationResult::PositionLimitExceeded:
                return enterFaultedState(FaultReason::TrajectoryPositionLimitExceeded);

            case JointTrajectoryEvaluationResult::VelocityLimitExceeded:
                return enterFaultedState(FaultReason::TrajectoryVelocityLimitExceeded);

            case JointTrajectoryEvaluationResult::AccelerationLimitExceeded:
                return enterFaultedState(FaultReason::TrajectoryAccelerationLimitExceeded);

            case JointTrajectoryEvaluationResult::JerkLimitExceeded:
                return enterFaultedState(FaultReason::TrajectoryJerkLimitExceeded);

            case JointTrajectoryEvaluationResult::NotConfigured:
            case JointTrajectoryEvaluationResult::NonFiniteTime:
            case JointTrajectoryEvaluationResult::TimeBeforeSegment:
            case JointTrajectoryEvaluationResult::TimeAfterSegment:
            case JointTrajectoryEvaluationResult::NonFiniteSample:
                return enterFaultedState(FaultReason::TrajectoryEvaluationFailed);
        }

        return enterFaultedState(FaultReason::TrajectoryEvaluationFailed);
    }

    CommandResult CommandInterpolator::validateCommand(const TargetCommand &command) const noexcept
    {
        for (std::size_t joint_index = 0; joint_index < command.positions.size(); ++joint_index)
        {
            const double position = command.positions[joint_index];

            if (!std::isfinite(position))
            {
                return CommandResult::RejectedPositionNotFinite;
            }

            const JointLimit &limit = joint_limits_[joint_index];

            if (position < limit.minimum_position || position > limit.maximum_position)
            {
                return CommandResult::RejectedPositionOutOfRange;
            }
        }

        return CommandResult::Accepted;
    }

    InterpolatorState CommandInterpolator::state() const noexcept
    {
        return state_machine_.state();
    }

    FaultReason CommandInterpolator::faultReason() const noexcept
    {
        return fault_reason_;
    }

    std::size_t CommandInterpolator::inputTelemetryAvailable() const noexcept
    {
        return input_telemetry_->available();
    }

    std::size_t CommandInterpolator::controlTelemetryAvailable() const noexcept
    {
        return control_telemetry_->available();
    }

    std::size_t CommandInterpolator::droppedInputTelemetryCount() const noexcept
    {
        return input_telemetry_->droppedSampleCount();
    }

    std::size_t CommandInterpolator::droppedControlTelemetryCount() const noexcept
    {
        return control_telemetry_->droppedSampleCount();
    }

    void CommandInterpolator::recordInputTelemetry(const TargetCommand &command,
                                                   CommandResult result) noexcept
    {
        InputTelemetrySample sample{};
        sample.received_time = command.received_time;
        sample.sender_time = command.sender_time;
        sample.commanded_positions = command.positions;
        sample.result = result;

        static_cast<void>(input_telemetry_->tryPush(sample));
    }

    void CommandInterpolator::recordControlTelemetry(
        ControllerTime current_time, ControlResult result,
        const std::array<double, 6> &output_target) noexcept
    {
        ControlTelemetrySample sample{};
        sample.controller_time = current_time;
        sample.segment_elapsed_nanoseconds = current_segment_elapsed_nanoseconds_;
        sample.segment_duration_nanoseconds = current_segment_duration_nanoseconds_;
        sample.result = result;
        sample.state = state_machine_.state();
        sample.fault_reason = fault_reason_;
        sample.failed_command = trajectory_failed_command_;
        sample.failed_joint = trajectory_failed_joint_;
        sample.trajectory_failure_time_seconds = trajectory_failure_time_seconds_;
        sample.trajectory_failure_value = trajectory_failure_value_;
        sample.has_target =
            result == ControlResult::TargetProduced || result == ControlResult::Holding;

        if (sample.has_target)
        {
            sample.target_positions = output_target;
        }

        static_cast<void>(control_telemetry_->tryPush(sample));
    }

    ControlResult CommandInterpolator::enterFaultedState(FaultReason reason) noexcept
    {
        if (fault_reason_ == FaultReason::None)
        {
            fault_reason_ = reason;
        }

        static_cast<void>(state_machine_.transitionTo(InterpolatorState::Faulted));

        return ControlResult::Faulted;
    }

    void CommandInterpolator::publishTarget(const std::array<double, 6> &target_positions,
                                            std::array<double, 6> &output_target) noexcept
    {
        output_target = target_positions;
        last_target_positions_ = target_positions;
        has_last_target_ = true;
    }
}
