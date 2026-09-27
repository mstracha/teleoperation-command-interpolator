#include "teleoperation/CommandInterpolator.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    constexpr std::int64_t Millisecond = 1'000'000;
    constexpr std::int64_t CommandInterval = 20 * Millisecond;
    constexpr double PositionIncrement = 0.01;

    TargetCommand makeCommand(std::int64_t sender_time, double position)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_time};
        command.positions.fill(position);
        return command;
    }

    ControllerTime addTime(ControllerTime time, std::int64_t nanoseconds)
    {
        return ControllerTime{time.nanoseconds_since_controller_epoch + nanoseconds};
    }

    template <typename Actual, typename Expected>
    void expectEqual(Actual actual, Expected expected, const char *test_name)
    {
        if (actual != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectPositions(const JointPositions &positions, double expected, const char *test_name)
    {
        for (double position : positions)
        {
            if (std::abs(position - expected) > 1.0e-12)
            {
                std::cerr << "FAILED: " << test_name << '\n';
                ++failure_count;
                return;
            }
        }
    }

    void expectNear(double actual, double expected, double tolerance, const char *test_name)
    {
        if (!std::isfinite(actual) || !std::isfinite(expected) ||
            std::abs(actual - expected) > tolerance)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }
}

int main()
{
    CommandInterpolator interpolator;
    JointPositions output{};
    const ControllerTime receive_start{1'000'000'000};

    expectEqual(interpolator.state(), InterpolatorState::Buffering,
                "interpolator starts Buffering");

    expectEqual(interpolator.updateTarget(ControllerTime{100}, output), ControlResult::Buffering,
                "controller reports Buffering before a complete window exists");

    for (std::int64_t index = 0; index < 4; ++index)
    {
        expectEqual(
            interpolator.submitCommand(makeCommand(index * CommandInterval,
                                                   static_cast<double>(index) * PositionIncrement),
                                       addTime(receive_start, index * Millisecond)),
            CommandResult::Accepted, "chronological command is accepted");
    }

    const ControllerTime playback_start = addTime(receive_start, 4 * Millisecond);

    expectEqual(interpolator.submitCommand(makeCommand(3 * CommandInterval, 0.3),
                                           addTime(receive_start, 5 * Millisecond)),
                CommandResult::RejectedDuplicateOrOutOfOrder, "duplicate command is rejected");

    expectEqual(interpolator.submitCommand(makeCommand(50 * Millisecond, 0.25),
                                           addTime(receive_start, 6 * Millisecond)),
                CommandResult::RejectedDuplicateOrOutOfOrder, "out-of-order command is rejected");

    expectEqual(interpolator.updateTarget(playback_start, output), ControlResult::TargetProduced,
                "complete window starts target production");

    expectEqual(interpolator.state(), InterpolatorState::Running,
                "target production transitions to Running");

    expectPositions(output, 0.01, "interpolation begins at the segment start");

    expectEqual(interpolator.updateTarget(addTime(playback_start, 10 * Millisecond), output),
                ControlResult::TargetProduced,
                "controller produces an interior interpolated target");

    expectPositions(output, 0.015, "interior target follows the validated trajectory");

    expectEqual(interpolator.updateTarget(addTime(playback_start, 21 * Millisecond), output),
                ControlResult::Holding, "missing successor transitions to Holding");

    expectEqual(interpolator.state(), InterpolatorState::Holding,
                "interpolator reports Holding state");

    expectPositions(output, 0.02, "Holding preserves the current segment endpoint");

    expectEqual(interpolator.submitCommand(makeCommand(4 * CommandInterval, 0.04),
                                           addTime(playback_start, 22 * Millisecond)),
                CommandResult::Accepted, "successor command is accepted while Holding");

    expectEqual(interpolator.updateTarget(addTime(playback_start, 23 * Millisecond), output),
                ControlResult::TargetProduced, "successor command resumes target production");

    expectEqual(interpolator.state(), InterpolatorState::Running,
                "successor command transitions Holding to Running");

    expectPositions(output, 0.0215, "resumed output evaluates the next segment at mapped time");

    std::array<InputTelemetrySample, 16> input_telemetry{};
    const std::size_t input_telemetry_count = interpolator.drainInputTelemetry(input_telemetry);

    expectEqual(input_telemetry_count, std::size_t{7},
                "every input call emits one telemetry sample");

    expectEqual(input_telemetry[0].result, CommandResult::Accepted,
                "input telemetry preserves the first accepted result");

    expectEqual(input_telemetry[4].result, CommandResult::RejectedDuplicateOrOutOfOrder,
                "input telemetry preserves duplicate rejection");

    expectEqual(input_telemetry[5].result, CommandResult::RejectedDuplicateOrOutOfOrder,
                "input telemetry preserves out-of-order rejection");

    expectEqual(input_telemetry[6].result, CommandResult::Accepted,
                "input telemetry preserves recovery command acceptance");

    expectEqual(input_telemetry[0].received_time.nanoseconds_since_controller_epoch,
                receive_start.nanoseconds_since_controller_epoch,
                "input telemetry preserves receive time");

    expectEqual(interpolator.inputTelemetryAvailable(), std::size_t{0},
                "input telemetry drain empties the queue");

    expectEqual(interpolator.droppedInputTelemetryCount(), std::size_t{0},
                "input telemetry did not overflow");

    std::array<ControlTelemetrySample, 16> control_telemetry{};
    const std::size_t control_telemetry_count =
        interpolator.drainControlTelemetry(control_telemetry);

    expectEqual(control_telemetry_count, std::size_t{5},
                "every control call emits one telemetry sample");

    expectEqual(control_telemetry[0].result, ControlResult::Buffering,
                "control telemetry preserves Buffering result");

    expectEqual(control_telemetry[0].has_target, false, "Buffering telemetry has no target");

    expectEqual(control_telemetry[1].state, InterpolatorState::Running,
                "control telemetry preserves Running state");

    expectEqual(control_telemetry[1].has_target, true,
                "target-production telemetry includes a target");

    expectEqual(control_telemetry[2].state, InterpolatorState::Running,
                "interior interpolation telemetry remains Running");

    expectEqual(control_telemetry[3].state, InterpolatorState::Holding,
                "control telemetry preserves Holding state");

    expectEqual(control_telemetry[4].state, InterpolatorState::Running,
                "control telemetry preserves resumed Running state");

    expectEqual(interpolator.controlTelemetryAvailable(), std::size_t{0},
                "control telemetry drain empties the queue");

    expectEqual(interpolator.droppedControlTelemetryCount(), std::size_t{0},
                "control telemetry did not overflow");

    CommandInterpolator full_buffer_interpolator;

    for (std::int64_t index = 0; index < 16; ++index)
    {
        expectEqual(full_buffer_interpolator.submitCommand(
                        makeCommand(index * CommandInterval, static_cast<double>(index) * 0.05)),
                    CommandResult::Accepted, "command fits in available buffer capacity");
    }

    expectEqual(full_buffer_interpolator.submitCommand(makeCommand(16 * CommandInterval, 0.8)),
                CommandResult::RejectedBufferFull,
                "command is rejected when the ring buffer is full");

    CommandInterpolator invalid_position_interpolator;

    expectEqual(invalid_position_interpolator.submitCommand(
                    makeCommand(0, std::numeric_limits<double>::quiet_NaN())),
                CommandResult::RejectedPositionNotFinite, "non-finite joint position is rejected");

    expectEqual(invalid_position_interpolator.submitCommand(
                    makeCommand(0, std::numeric_limits<double>::infinity())),
                CommandResult::RejectedPositionNotFinite, "infinite joint position is rejected");

    expectEqual(invalid_position_interpolator.submitCommand(makeCommand(0, 100.0)),
                CommandResult::RejectedPositionOutOfRange,
                "out-of-range joint position is rejected");

    CommandInterpolator interval_interpolator;

    expectEqual(interval_interpolator.submitCommand(makeCommand(0, 0.0)), CommandResult::Accepted,
                "first interval-test command is accepted");

    expectEqual(interval_interpolator.submitCommand(makeCommand(5 * Millisecond, 0.1)),
                CommandResult::RejectedIntervalTooSmall,
                "command inside the minimum interval is rejected");

    expectEqual(interval_interpolator.submitCommand(makeCommand(CommandInterval, 0.2)),
                CommandResult::Accepted,
                "later valid command is measured from the last accepted command");

    CommandInterpolator sender_gap_interpolator;

    expectEqual(sender_gap_interpolator.submitCommand(makeCommand(0, 0.0)), CommandResult::Accepted,
                "first sender-gap command is accepted");

    expectEqual(sender_gap_interpolator.submitCommand(makeCommand(101 * Millisecond, 0.1)),
                CommandResult::RejectedExcessiveSenderGap, "excessive sender gap is rejected");

    expectEqual(sender_gap_interpolator.state(), InterpolatorState::Buffering,
                "producer does not directly mutate controller state");

    expectEqual(sender_gap_interpolator.updateTarget(ControllerTime{0}, output),
                ControlResult::Faulted, "controller consumes the pending sender-gap fault");

    expectEqual(sender_gap_interpolator.state(), InterpolatorState::Faulted,
                "controller owns the transition to Faulted");

    expectEqual(sender_gap_interpolator.faultReason(), FaultReason::ExcessiveSenderGap,
                "sender-gap fault reason is preserved");

    std::array<InputTelemetrySample, 4> sender_gap_input_telemetry{};

    expectEqual(sender_gap_interpolator.drainInputTelemetry(sender_gap_input_telemetry),
                std::size_t{2}, "sender-gap input events are available to logger");

    expectEqual(sender_gap_input_telemetry[1].result, CommandResult::RejectedExcessiveSenderGap,
                "input telemetry preserves sender-gap rejection");

    std::array<ControlTelemetrySample, 4> sender_gap_control_telemetry{};

    expectEqual(sender_gap_interpolator.drainControlTelemetry(sender_gap_control_telemetry),
                std::size_t{1}, "terminal control event is available to logger");

    expectEqual(sender_gap_control_telemetry[0].fault_reason, FaultReason::ExcessiveSenderGap,
                "terminal telemetry preserves the latched fault reason");

    expectEqual(sender_gap_control_telemetry[0].has_target, false,
                "terminal telemetry does not claim a valid target");

    CommandInterpolator watchdog_interpolator;
    const ControllerTime watchdog_receive_start{2'000'000'000};

    for (std::int64_t index = 0; index < 4; ++index)
    {
        expectEqual(watchdog_interpolator.submitCommand(
                        makeCommand(index * CommandInterval,
                                    static_cast<double>(index) * PositionIncrement),
                        addTime(watchdog_receive_start, index * CommandInterval)),
                    CommandResult::Accepted, "watchdog setup command is accepted");
    }

    const ControllerTime watchdog_running_time =
        addTime(watchdog_receive_start, 3 * CommandInterval);

    expectEqual(watchdog_interpolator.updateTarget(watchdog_running_time, output),
                ControlResult::TargetProduced, "watchdog test enters Running");

    expectPositions(output, 0.01, "watchdog test records the last emitted target");

    const ControllerTime watchdog_hold_time = addTime(watchdog_running_time, 150 * Millisecond);

    expectEqual(watchdog_interpolator.updateTarget(watchdog_hold_time, output),
                ControlResult::Holding, "receive silence at hold timeout enters Holding");

    expectEqual(watchdog_interpolator.state(), InterpolatorState::Holding,
                "watchdog owns the Holding state transition");

    expectPositions(output, 0.01, "watchdog hold preserves the last emitted target");

    CommandInterpolator unsafe_trajectory_interpolator;
    const ControllerTime unsafe_receive_start{3'000'000'000};

    for (std::int64_t index = 0; index < 4; ++index)
    {
        expectEqual(unsafe_trajectory_interpolator.submitCommand(
                        makeCommand(index * CommandInterval, static_cast<double>(index) * 0.1),
                        addTime(unsafe_receive_start, index * Millisecond)),
                    CommandResult::Accepted,
                    "unsafe trajectory command passes position ingress checks");
    }

    output.fill(9.0);

    expectEqual(unsafe_trajectory_interpolator.updateTarget(
                    addTime(unsafe_receive_start, 4 * Millisecond), output),
                ControlResult::Faulted,
                "unsafe estimated trajectory faults before target publication");

    expectEqual(unsafe_trajectory_interpolator.faultReason(),
                FaultReason::TrajectoryVelocityLimitExceeded,
                "unsafe trajectory preserves its velocity-limit fault reason");

    expectPositions(output, 9.0, "trajectory configuration fault leaves caller output unchanged");

    std::array<ControlTelemetrySample, 2> unsafe_control_telemetry{};

    expectEqual(unsafe_trajectory_interpolator.drainControlTelemetry(unsafe_control_telemetry),
                std::size_t{1}, "trajectory fault emits one control telemetry record");

    expectEqual(unsafe_control_telemetry[0].fault_reason,
                FaultReason::TrajectoryVelocityLimitExceeded,
                "trajectory telemetry preserves the fault category");

    expectEqual(unsafe_control_telemetry[0].failed_joint, std::size_t{0},
                "trajectory telemetry identifies the failed joint");

    expectEqual(unsafe_control_telemetry[0].failed_command, ControlTelemetrySample::NoFailedCommand,
                "trajectory configuration fault is not assigned to one command");

    expectNear(unsafe_control_telemetry[0].trajectory_failure_time_seconds, 0.0, 0.0,
               "boundary velocity failure occurs at the segment start");

    expectNear(unsafe_control_telemetry[0].trajectory_failure_value, 5.0, 1.0e-12,
               "trajectory telemetry preserves the failed velocity value");

    CommandInterpolator distant_epoch_interpolator;
    constexpr std::int64_t SenderEpochStart =
        std::numeric_limits<std::int64_t>::min() + 1'000'000'000;
    constexpr std::int64_t ControllerEpochStart =
        std::numeric_limits<std::int64_t>::max() - 1'000'000'000;

    for (std::int64_t index = 0; index < 4; ++index)
    {
        expectEqual(distant_epoch_interpolator.submitCommand(
                        makeCommand(SenderEpochStart + index * CommandInterval,
                                    static_cast<double>(index) * PositionIncrement),
                        ControllerTime{ControllerEpochStart + index * Millisecond}),
                    CommandResult::Accepted, "distant-epoch command is accepted");
    }

    const ControllerTime distant_epoch_playback{ControllerEpochStart + 4 * Millisecond};

    const ControlResult distant_epoch_start_result =
        distant_epoch_interpolator.updateTarget(distant_epoch_playback, output);

    expectEqual(distant_epoch_start_result, ControlResult::TargetProduced,
                "unrelated extreme clock epochs initialize safely");

    expectPositions(output, 0.01, "distant epochs preserve segment-start output");

    expectEqual(distant_epoch_interpolator.updateTarget(
                    addTime(distant_epoch_playback, 10 * Millisecond), output),
                ControlResult::TargetProduced,
                "elapsed-time mapping works without epoch subtraction");

    expectPositions(output, 0.015, "distant epochs preserve interior interpolation");

    const ControllerTime watchdog_fault_time = addTime(watchdog_running_time, 500 * Millisecond);

    expectEqual(watchdog_interpolator.updateTarget(watchdog_fault_time, output),
                ControlResult::Faulted, "receive silence at fault timeout enters Faulted");

    expectEqual(watchdog_interpolator.state(), InterpolatorState::Faulted,
                "watchdog owns the Faulted state transition");

    expectEqual(watchdog_interpolator.faultReason(), FaultReason::ReceiveTimeout,
                "receive-timeout reason is preserved");

    CommandInterpolator backward_clock_interpolator;

    expectEqual(
        backward_clock_interpolator.submitCommand(makeCommand(0, 0.0), watchdog_receive_start),
        CommandResult::Accepted, "backward-clock setup command is accepted");

    expectEqual(
        backward_clock_interpolator.updateTarget(addTime(watchdog_receive_start, -1), output),
        ControlResult::Buffering,
        "receive newer than sampled controller tick is treated as age zero");

    expectEqual(backward_clock_interpolator.faultReason(), FaultReason::None,
                "concurrent receive ordering does not invent a clock fault");

    CommandInterpolator regressing_clock_interpolator;

    expectEqual(regressing_clock_interpolator.updateTarget(ControllerTime{100}, output),
                ControlResult::Buffering,
                "clock-regression test records its initial controller time");

    expectEqual(regressing_clock_interpolator.updateTarget(ControllerTime{99}, output),
                ControlResult::Faulted, "controller time regression enters Faulted");

    expectEqual(regressing_clock_interpolator.faultReason(), FaultReason::ControllerTimeRegression,
                "controller-regression reason is preserved");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " interpolator test(s) failed\n";
        return 1;
    }

    std::cout << "All interpolator tests passed\n";
    return 0;
}
