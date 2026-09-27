#include "teleoperation/RealTimeHarness.hpp"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>
#include <vector>

using namespace teleoperation;

namespace
{
    constexpr std::int64_t Millisecond = 1'000'000;

    struct Experiment
    {
        std::string_view name;
        ControllerWaitPreference wait_preference;
        std::optional<std::size_t> logical_processor;
    };

    TargetCommand makeCommand(std::int64_t sender_time, double position)
    {
        TargetCommand command{};
        command.sender_time = SenderTime{sender_time};
        command.positions.fill(position);
        return command;
    }

    bool parseSize(std::string_view text, std::size_t &value)
    {
        const char *const begin = text.data();
        const char *const end = begin + text.size();
        const std::from_chars_result result = std::from_chars(begin, end, value);
        return result.ec == std::errc{} && result.ptr == end;
    }

    std::string_view toString(ControllerWaitStrategy strategy)
    {
        switch (strategy)
        {
            case ControllerWaitStrategy::PortableConditionVariable:
                return "portable_condition_variable";
            case ControllerWaitStrategy::WindowsHighResolutionWaitableTimer:
                return "windows_high_resolution_waitable_timer";
        }

        return "unknown";
    }

    std::string_view toString(ControllerAffinityResult result)
    {
        switch (result)
        {
            case ControllerAffinityResult::NotRequested:
                return "not_requested";
            case ControllerAffinityResult::Applied:
                return "applied";
            case ControllerAffinityResult::Unsupported:
                return "unsupported";
            case ControllerAffinityResult::InvalidLogicalProcessor:
                return "invalid_logical_processor";
            case ControllerAffinityResult::ApplicationFailed:
                return "application_failed";
        }

        return "unknown";
    }

    double observedFrequency(const RealTimeRunResult &result)
    {
        if (result.controller_timing.size() < 2)
        {
            return 0.0;
        }

        const std::chrono::nanoseconds observed_span =
            result.controller_timing.back().actual_start_offset -
            result.controller_timing.front().actual_start_offset;

        if (observed_span.count() <= 0)
        {
            return 0.0;
        }

        return static_cast<double>(result.controller_timing.size() - 1) * 1'000'000'000.0 /
               static_cast<double>(observed_span.count());
    }

    RealTimeScenario makeScenario(const Experiment &experiment)
    {
        RealTimeScenario scenario{};
        scenario.run_duration = std::chrono::seconds{1};
        scenario.controller_wait_preference = experiment.wait_preference;
        scenario.controller_logical_processor = experiment.logical_processor;

        for (std::int64_t command_index = 0; command_index < 7; ++command_index)
        {
            scenario.commands.push_back(
                RealTimeScheduledCommand{std::chrono::milliseconds{40 * command_index},
                                         makeCommand(40 * command_index * Millisecond,
                                                     0.01 * static_cast<double>(command_index))});
        }

        return scenario;
    }
}

int main(int argument_count, char **arguments)
{
    std::size_t trial_count = 10;
    std::optional<std::size_t> logical_processor;

    if (argument_count > 1 && !parseSize(arguments[1], trial_count))
    {
        std::cerr << "Trial count must be a positive integer\n";
        return 2;
    }

    if (trial_count == 0)
    {
        std::cerr << "Trial count must be greater than zero\n";
        return 2;
    }

    if (argument_count > 2)
    {
        std::size_t parsed_processor = 0;

        if (!parseSize(arguments[2], parsed_processor))
        {
            std::cerr << "Logical processor must be a nonnegative integer\n";
            return 2;
        }

        logical_processor = parsed_processor;
    }

    if (argument_count > 3)
    {
        std::cerr << "Usage: timing_qualification [trial_count] [logical_processor]\n";
        return 2;
    }

    std::vector<Experiment> experiments{
        {"portable", ControllerWaitPreference::PortableConditionVariable, std::nullopt},
        {"automatic", ControllerWaitPreference::Automatic, std::nullopt}};

    if (logical_processor.has_value())
    {
        experiments.push_back(
            {"automatic_pinned", ControllerWaitPreference::Automatic, logical_processor});
    }

    InterpolatorConfiguration relaxed_watchdog{};
    relaxed_watchdog.hold_timeout = std::chrono::seconds{1};
    relaxed_watchdog.fault_timeout = std::chrono::seconds{2};
    const RealTimeHarness harness(DefaultToyJointLimits, relaxed_watchdog);

    std::cout << "experiment,trial,actual_wait_strategy,requested_logical_processor,"
                 "affinity_result,controller_ticks,skipped_periods,observed_frequency_hz,"
                 "p50_lateness_us,p95_lateness_us,p99_lateness_us,maximum_lateness_us,"
                 "maximum_update_duration_us,wall_time_ms\n";
    std::cout << std::fixed << std::setprecision(3);

    for (std::size_t trial = 1; trial <= trial_count; ++trial)
    {
        // Rotate execution order so first-run and background-load effects are not
        // assigned to the same configuration in every trial.
        for (std::size_t experiment_offset = 0; experiment_offset < experiments.size();
             ++experiment_offset)
        {
            const std::size_t experiment_index =
                (experiment_offset + trial - 1) % experiments.size();
            const Experiment &experiment = experiments[experiment_index];
            const RealTimeRunResult result = harness.run(makeScenario(experiment));

            if (result.outcome != RealTimeRunOutcome::Completed)
            {
                std::cerr << "Experiment " << experiment.name << " trial " << trial
                          << " did not complete\n";
                return 1;
            }

            std::cout << experiment.name << ',' << trial << ','
                      << toString(result.controller_wait_strategy) << ',';

            if (experiment.logical_processor.has_value())
            {
                std::cout << *experiment.logical_processor;
            }

            std::cout << ',' << toString(result.controller_affinity_result) << ','
                      << result.controller_tick_count << ','
                      << result.skipped_controller_period_count << ','
                      << observedFrequency(result) << ','
                      << static_cast<double>(result.p50_controller_lateness.count()) / 1'000.0
                      << ','
                      << static_cast<double>(result.p95_controller_lateness.count()) / 1'000.0
                      << ','
                      << static_cast<double>(result.p99_controller_lateness.count()) / 1'000.0
                      << ','
                      << static_cast<double>(result.maximum_controller_lateness.count()) /
                             1'000.0
                      << ','
                      << static_cast<double>(result.maximum_control_call_duration.count()) /
                             1'000.0
                      << ',' << static_cast<double>(result.wall_time.count()) / 1'000'000.0
                      << '\n';
        }
    }

    return 0;
}
