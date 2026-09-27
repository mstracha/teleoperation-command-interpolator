#pragma once

#include "teleoperation/InterpolatorConfiguration.hpp"
#include "teleoperation/JointLimits.hpp"
#include "teleoperation/ShutdownCoordinator.hpp"
#include "teleoperation/TelemetrySamples.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace teleoperation
{
    struct RealTimeScheduledCommand
    {
        std::chrono::nanoseconds delivery_offset{0};
        TargetCommand command{};
    };

    enum class ControllerWaitPreference : std::uint8_t
    {
        Automatic,
        PortableConditionVariable,
        WindowsHighResolutionWaitableTimer
    };

    struct RealTimeScenario
    {
        std::chrono::nanoseconds run_duration{std::chrono::milliseconds{300}};
        std::chrono::nanoseconds controller_period{std::chrono::milliseconds{2}};
        std::chrono::nanoseconds logger_poll_period{std::chrono::milliseconds{1}};
        std::chrono::nanoseconds startup_delay{std::chrono::milliseconds{10}};
        std::size_t maximum_controller_ticks{1'000'000};
        ControllerWaitPreference controller_wait_preference{
            ControllerWaitPreference::Automatic};
        std::optional<std::size_t> controller_logical_processor;
        std::vector<RealTimeScheduledCommand> commands;
    };

    enum class RealTimeRunOutcome : std::uint8_t
    {
        Completed,
        Faulted,
        InvalidRunDuration,
        InvalidControllerPeriod,
        InvalidLoggerPeriod,
        InvalidStartupDelay,
        DeliveryOffsetsOutOfOrder,
        ControllerTickLimitExceeded
    };

    enum class ControllerWaitStrategy : std::uint8_t
    {
        PortableConditionVariable,
        WindowsHighResolutionWaitableTimer
    };

    enum class ControllerAffinityResult : std::uint8_t
    {
        NotRequested,
        Applied,
        Unsupported,
        InvalidLogicalProcessor,
        ApplicationFailed
    };

    struct ControllerTimingSample
    {
        std::chrono::nanoseconds scheduled_offset{0};
        std::chrono::nanoseconds actual_start_offset{0};
        std::chrono::nanoseconds lateness{0};
        std::chrono::nanoseconds update_duration{0};
    };

    struct RealTimeRunResult
    {
        RealTimeRunOutcome outcome{RealTimeRunOutcome::Completed};
        InterpolatorState final_state{InterpolatorState::Buffering};
        FaultReason fault_reason{FaultReason::None};
        ShutdownPhase shutdown_phase{ShutdownPhase::Running};
        std::chrono::nanoseconds configured_run_duration{0};
        std::chrono::nanoseconds configured_controller_period{0};
        std::chrono::nanoseconds configured_logger_poll_period{0};
        std::chrono::nanoseconds configured_startup_delay{0};
        ControllerWaitPreference configured_controller_wait_preference{
            ControllerWaitPreference::Automatic};
        ControllerWaitStrategy controller_wait_strategy{
            ControllerWaitStrategy::PortableConditionVariable};
        std::optional<std::size_t> requested_controller_logical_processor;
        ControllerAffinityResult controller_affinity_result{
            ControllerAffinityResult::NotRequested};
        std::size_t scheduled_command_count{0};
        std::size_t delivered_command_count{0};
        std::size_t controller_tick_count{0};
        std::size_t skipped_controller_period_count{0};
        std::chrono::nanoseconds p50_controller_lateness{0};
        std::chrono::nanoseconds p95_controller_lateness{0};
        std::chrono::nanoseconds p99_controller_lateness{0};
        std::chrono::nanoseconds maximum_controller_lateness{0};
        std::chrono::nanoseconds maximum_control_call_duration{0};
        std::chrono::nanoseconds wall_time{0};
        std::size_t dropped_input_telemetry_count{0};
        std::size_t dropped_control_telemetry_count{0};
        std::vector<InputTelemetrySample> input_telemetry;
        std::vector<ControlTelemetrySample> control_telemetry;
        std::vector<ControllerTimingSample> controller_timing;
    };

    // Process-local integration harness with one network producer, one controller
    // producer, and one telemetry consumer. Its waits are interruptible through
    // ShutdownCoordinator; no thread is forcibly terminated.
    class RealTimeHarness
    {
      public:
        explicit RealTimeHarness(const JointLimits &joint_limits = DefaultToyJointLimits,
                                 const InterpolatorConfiguration &configuration = {});

        RealTimeRunResult run(const RealTimeScenario &scenario) const;

      private:
        JointLimits joint_limits_;
        InterpolatorConfiguration configuration_;
    };
}
