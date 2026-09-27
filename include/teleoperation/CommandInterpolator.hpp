#pragma once

#include "teleoperation/CommandDerivativeEstimator.hpp"
#include "teleoperation/CommandRingBuffer.hpp"
#include "teleoperation/FaultReason.hpp"
#include "teleoperation/InterpolatorConfiguration.hpp"
#include "teleoperation/InterpolatorResults.hpp"
#include "teleoperation/InterpolatorStateMachine.hpp"
#include "teleoperation/JointLimits.hpp"
#include "teleoperation/TelemetryRingBuffer.hpp"
#include "teleoperation/TelemetrySamples.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace teleoperation
{
    class CommandInterpolator
    {
      public:
        CommandInterpolator() = default;

        explicit CommandInterpolator(const JointLimits &joint_limits);

        CommandInterpolator(const JointLimits &joint_limits,
                            const InterpolatorConfiguration &configuration);

        // Triggered by the jittery network teleoperation thread (15-30 Hz).
        CommandResult submitCommand(TargetCommand cmd);

        // Allows a network adapter or deterministic simulation to provide one
        // already-sampled time from the same local monotonic controller clock.
        CommandResult submitCommand(TargetCommand cmd, ControllerTime received_time);

        // Called by the controller thread, nominally at 500 Hz in the examples.
        // This method does not schedule that thread, estimate the robot's physical
        // state, or directly control hardware.
        ControlResult updateTarget(ControllerTime current_time, JointPositions &output_target);

        // Read from the controller thread only.
        InterpolatorState state() const noexcept;

        // The first fault remains latched for the lifetime of this interpolator.
        // Read from the controller thread only.
        FaultReason faultReason() const noexcept;

        // Called only by the single logger consumer.
        template <std::size_t BatchCapacity>
        std::size_t drainInputTelemetry(std::array<InputTelemetrySample, BatchCapacity> &samples,
                                        std::size_t requested_count = BatchCapacity)
        {
            return input_telemetry_->tryPopBatch(samples, requested_count);
        }

        // Called only by the single logger consumer.
        template <std::size_t BatchCapacity>
        std::size_t drainControlTelemetry(
            std::array<ControlTelemetrySample, BatchCapacity> &samples,
            std::size_t requested_count = BatchCapacity)
        {
            return control_telemetry_->tryPopBatch(samples, requested_count);
        }

        std::size_t inputTelemetryAvailable() const noexcept;
        std::size_t controlTelemetryAvailable() const noexcept;
        std::size_t droppedInputTelemetryCount() const noexcept;
        std::size_t droppedControlTelemetryCount() const noexcept;

      private:
        CommandResult acceptCommand(const TargetCommand &command);

        CommandResult validateCommand(const TargetCommand &command) const noexcept;

        ControlResult calculateNextInterpolatedCommand(ControllerTime current_time,
                                                       JointPositions &output_target);

        FaultReason configureActiveTrajectory() noexcept;

        ControlResult evaluateActiveTrajectory(std::int64_t elapsed_nanoseconds,
                                               JointPositions &output_target) noexcept;

        void recordInputTelemetry(const TargetCommand &command, CommandResult result) noexcept;

        void recordControlTelemetry(ControllerTime current_time, ControlResult result,
                                    const JointPositions &output_target) noexcept;

        ControlResult enterFaultedState(FaultReason reason) noexcept;

        void publishTarget(const JointPositions &target_positions,
                           JointPositions &output_target) noexcept;

        static constexpr std::size_t LookaheadWindow = 4;
        static constexpr std::size_t CommandBufferCapacity = 16;
        static constexpr std::size_t InputTelemetryCapacity = 256;
        static constexpr std::size_t ControlTelemetryCapacity = 4096;

        using InputTelemetryBuffer =
            TelemetryRingBuffer<InputTelemetrySample, InputTelemetryCapacity>;

        using ControlTelemetryBuffer =
            TelemetryRingBuffer<ControlTelemetrySample, ControlTelemetryCapacity>;

        // With four chronologically ordered commands, the middle two define the
        // active segment; the outer two provide derivative context.
        static constexpr std::size_t SegmentStartIndex = 1;
        static constexpr std::size_t SegmentEndIndex = 2;

        InterpolatorStateMachine state_machine_;
        CommandRingBuffer<CommandBufferCapacity, LookaheadWindow> command_queue_;

        // Allocate the large fixed-capacity stores once during construction.
        // No telemetry operation allocates memory on either real-time path.
        std::unique_ptr<InputTelemetryBuffer> input_telemetry_{
            std::make_unique<InputTelemetryBuffer>()};

        std::unique_ptr<ControlTelemetryBuffer> control_telemetry_{
            std::make_unique<ControlTelemetryBuffer>()};

        // Stable consumer-owned copy of the current interpolation window.
        std::array<TargetCommand, LookaheadWindow> interpolation_window_{};

        // Used for a position-preserving hold when communication becomes stale.
        JointPositions last_target_positions_{};
        bool has_last_target_{false};

        bool timeline_initialized_{false};
        std::int64_t timeline_controller_origin_nanoseconds_{0};
        std::int64_t timeline_sender_origin_nanoseconds_{0};
        std::int64_t current_segment_elapsed_nanoseconds_{0};
        std::int64_t current_segment_duration_nanoseconds_{0};
        bool has_last_controller_time_{false};
        std::int64_t last_controller_time_nanoseconds_{0};

        JointLimits joint_limits_{DefaultToyJointLimits};

        JointTrajectorySegment active_trajectory_{joint_limits_};

        InterpolatorConfiguration configuration_{};

        bool has_newest_sender_time_{false};
        std::int64_t newest_sender_time_{0};

        // The producer may request a fault, but only the controller thread applies
        // the state transition and latches the reason locally.
        std::atomic<FaultReason> pending_fault_reason_{FaultReason::None};
        FaultReason fault_reason_{FaultReason::None};
        std::size_t trajectory_failed_command_{ControlTelemetrySample::NoFailedCommand};
        std::size_t trajectory_failed_joint_{ControlTelemetrySample::NoFailedJoint};
        double trajectory_failure_time_seconds_{0.0};
        double trajectory_failure_value_{0.0};

        std::atomic<std::int64_t> latest_receive_time_nanoseconds_{0};
        std::atomic<bool> has_accepted_receive_time_{false};

        static_assert(std::atomic<bool>::is_always_lock_free,
                      "The real-time watchdog requires a lock-free presence flag");

        static_assert(std::atomic<FaultReason>::is_always_lock_free,
                      "The real-time fault handoff requires a lock-free reason atomic");

        static_assert(std::atomic<std::int64_t>::is_always_lock_free,
                      "The real-time watchdog requires a lock-free timestamp atomic");
    };
}
