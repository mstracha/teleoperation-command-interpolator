#include "teleoperation/VirtualTimeHarness.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    constexpr std::int64_t Millisecond = 1'000'000;
    constexpr std::int64_t CommandInterval = 40 * Millisecond;
    constexpr double PositionIncrement = 0.01;

    template <typename Actual, typename Expected>
    void expectEqual(Actual actual, Expected expected, const char *test_name)
    {
        if (actual != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectNear(double actual, double expected, double tolerance, const char *test_name)
    {
        if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        {
            std::cerr << "FAILED: " << test_name << " expected " << expected << " but received "
                      << actual << '\n';
            ++failure_count;
        }
    }

    TargetCommand makeCommand(std::int64_t sender_time, double position)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_time};
        command.positions.fill(position);
        return command;
    }

    ScheduledTargetCommand makeScheduledCommand(std::int64_t sender_time,
                                                std::int64_t delivery_time, double position)
    {
        return ScheduledTargetCommand{ControllerTime{delivery_time},
                                      makeCommand(sender_time, position)};
    }
}

int main()
{
    VirtualTimeHarness harness;
    constexpr std::int64_t HealthyStart = 1'000'000'000;

    const std::array<std::int64_t, 12> delivery_offsets_milliseconds{0,   42,  79,  123, 158, 205,
                                                                     238, 282, 321, 363, 398, 443};

    VirtualTimeScenario healthy_scenario{};
    healthy_scenario.controller_start = ControllerTime{HealthyStart};
    healthy_scenario.controller_end = ControllerTime{HealthyStart + 460 * Millisecond};

    for (std::size_t command_index = 0; command_index < delivery_offsets_milliseconds.size();
         ++command_index)
    {
        healthy_scenario.commands.push_back(makeScheduledCommand(
            static_cast<std::int64_t>(command_index) * CommandInterval,
            HealthyStart + delivery_offsets_milliseconds[command_index] * Millisecond,
            static_cast<double>(command_index) * PositionIncrement));
    }

    const VirtualTimeRunResult healthy_result = harness.run(healthy_scenario);

    expectEqual(healthy_result.outcome, VirtualTimeRunOutcome::Completed,
                "healthy jittered scenario completes");

    expectEqual(healthy_result.final_state, InterpolatorState::Running,
                "healthy scenario finishes Running");

    expectEqual(healthy_result.fault_reason, FaultReason::None, "healthy scenario has no fault");

    expectEqual(healthy_result.shutdown_phase, ShutdownPhase::Running,
                "normal virtual completion does not imitate fault shutdown");

    expectEqual(healthy_result.delivered_command_count, std::size_t{12},
                "healthy scenario delivers every scheduled command");

    expectEqual(healthy_result.controller_tick_count, std::size_t{231},
                "460 milliseconds produces 231 inclusive 500 Hz ticks");

    expectEqual(healthy_result.input_telemetry.size(), std::size_t{12},
                "healthy scenario captures every input record");

    expectEqual(healthy_result.control_telemetry.size(), healthy_result.controller_tick_count,
                "healthy scenario captures every control record");

    expectEqual(healthy_result.dropped_input_telemetry_count, std::size_t{0},
                "healthy scenario drops no input telemetry");

    expectEqual(healthy_result.dropped_control_telemetry_count, std::size_t{0},
                "healthy scenario drops no control telemetry");

    const std::int64_t midpoint_controller_time = HealthyStart + 144 * Millisecond;

    const auto midpoint_sample = std::find_if(
        healthy_result.control_telemetry.begin(), healthy_result.control_telemetry.end(),
        [midpoint_controller_time](const ControlTelemetrySample &sample)
        {
            return sample.controller_time.nanoseconds_since_controller_epoch ==
                   midpoint_controller_time;
        });

    expectEqual(midpoint_sample != healthy_result.control_telemetry.end(), true,
                "healthy scenario contains the selected virtual tick");

    if (midpoint_sample != healthy_result.control_telemetry.end())
    {
        expectEqual(midpoint_sample->result, ControlResult::TargetProduced,
                    "selected virtual tick produces a target");

        expectNear(midpoint_sample->target_positions[0], 0.015, 1.0e-12,
                   "selected virtual tick has deterministic interpolation");
    }

    constexpr std::int64_t TimeoutStart = 2'000'000'000;
    VirtualTimeScenario timeout_scenario{};
    timeout_scenario.controller_start = ControllerTime{TimeoutStart};
    timeout_scenario.controller_end = ControllerTime{TimeoutStart + 600 * Millisecond};

    for (std::int64_t command_index = 0; command_index < 4; ++command_index)
    {
        timeout_scenario.commands.push_back(makeScheduledCommand(
            command_index * 20 * Millisecond, TimeoutStart + command_index * 20 * Millisecond,
            static_cast<double>(command_index) * PositionIncrement));
    }

    const VirtualTimeRunResult timeout_result = harness.run(timeout_scenario);

    expectEqual(timeout_result.outcome, VirtualTimeRunOutcome::Faulted,
                "receive-silence scenario faults");

    expectEqual(timeout_result.fault_reason, FaultReason::ReceiveTimeout,
                "receive-silence scenario preserves timeout reason");

    expectEqual(timeout_result.final_state, InterpolatorState::Faulted,
                "receive-silence scenario finishes Faulted");

    expectEqual(timeout_result.shutdown_phase, ShutdownPhase::Complete,
                "faulted virtual run drains logs and completes shutdown");

    expectEqual(timeout_result.delivered_command_count, std::size_t{4},
                "timeout scenario delivers all setup commands");

    expectEqual(timeout_result.controller_tick_count, std::size_t{281},
                "timeout occurs at the exact 500 millisecond boundary");

    expectEqual(timeout_result.control_telemetry.size(), timeout_result.controller_tick_count,
                "faulting control record is drained before completion");

    if (!timeout_result.control_telemetry.empty())
    {
        expectEqual(timeout_result.control_telemetry.back().fault_reason,
                    FaultReason::ReceiveTimeout, "final drained record preserves timeout reason");
    }

    VirtualTimeScenario invalid_scenario{};
    invalid_scenario.controller_start = ControllerTime{0};
    invalid_scenario.controller_end = ControllerTime{10 * Millisecond};
    invalid_scenario.commands.push_back(makeScheduledCommand(0, 5 * Millisecond, 0.0));
    invalid_scenario.commands.push_back(
        makeScheduledCommand(20 * Millisecond, 4 * Millisecond, 0.01));

    const VirtualTimeRunResult invalid_result = harness.run(invalid_scenario);

    expectEqual(invalid_result.outcome, VirtualTimeRunOutcome::DeliveryTimesOutOfOrder,
                "out-of-order delivery schedule is rejected");

    expectEqual(invalid_result.controller_tick_count, std::size_t{0},
                "invalid scenario executes no controller ticks");

    VirtualTimeScenario invalid_period_scenario{};
    invalid_period_scenario.controller_start = ControllerTime{0};
    invalid_period_scenario.controller_end = ControllerTime{Millisecond};
    invalid_period_scenario.controller_period = std::chrono::nanoseconds{0};

    expectEqual(harness.run(invalid_period_scenario).outcome,
                VirtualTimeRunOutcome::InvalidControllerPeriod,
                "nonpositive controller period is rejected");

    VirtualTimeScenario invalid_range_scenario{};
    invalid_range_scenario.controller_start = ControllerTime{Millisecond};
    invalid_range_scenario.controller_end = ControllerTime{0};

    expectEqual(harness.run(invalid_range_scenario).outcome,
                VirtualTimeRunOutcome::InvalidControllerRange,
                "reversed controller range is rejected");

    VirtualTimeScenario tick_limit_scenario{};
    tick_limit_scenario.controller_start = ControllerTime{0};
    tick_limit_scenario.controller_end = ControllerTime{10 * Millisecond};
    tick_limit_scenario.maximum_controller_ticks = 5;

    expectEqual(harness.run(tick_limit_scenario).outcome,
                VirtualTimeRunOutcome::ControllerTickLimitExceeded,
                "scenario exceeding its tick budget is rejected");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " virtual-time harness test(s) failed\n";
        return 1;
    }

    std::cout << "All virtual-time harness tests passed\n";
    return 0;
}
