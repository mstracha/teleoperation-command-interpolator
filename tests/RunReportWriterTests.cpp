#include "teleoperation/RunReportWriter.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    template <typename Actual, typename Expected>
    void expectEqual(Actual actual, Expected expected, const char *test_name)
    {
        if (actual != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectTrue(bool condition, const char *test_name)
    {
        if (!condition)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    std::string readFile(const std::filesystem::path &path)
    {
        std::ifstream input(path);
        std::ostringstream contents;
        contents << input.rdbuf();
        return contents.str();
    }

    struct TemporaryDirectory
    {
        explicit TemporaryDirectory(const std::filesystem::path &value) : path(value) {}

        ~TemporaryDirectory()
        {
            std::error_code ignored_error;
            static_cast<void>(std::filesystem::remove_all(path, ignored_error));
        }

        std::filesystem::path path;
    };
}

int main()
{
    std::error_code path_error;
    const std::filesystem::path temporary_root = std::filesystem::temp_directory_path(path_error);

    expectTrue(!path_error, "system temporary directory is available");

    const auto unique_value = std::chrono::steady_clock::now().time_since_epoch().count();

    TemporaryDirectory test_directory(
        temporary_root /
        ("teleoperation_command_interpolator_report_test_" + std::to_string(unique_value)));

    RealTimeRunResult run{};
    run.outcome = RealTimeRunOutcome::Faulted;
    run.final_state = InterpolatorState::Faulted;
    run.fault_reason = FaultReason::TrajectoryVelocityLimitExceeded;
    run.shutdown_phase = ShutdownPhase::Complete;
    run.configured_controller_wait_preference =
        ControllerWaitPreference::WindowsHighResolutionWaitableTimer;
    run.controller_wait_strategy = ControllerWaitStrategy::PortableConditionVariable;
    run.requested_controller_logical_processor = 3;
    run.controller_affinity_result = ControllerAffinityResult::Applied;
    run.configured_run_duration = std::chrono::milliseconds{300};
    run.configured_controller_period = std::chrono::milliseconds{2};
    run.configured_logger_poll_period = std::chrono::milliseconds{1};
    run.configured_startup_delay = std::chrono::milliseconds{10};
    run.scheduled_command_count = 2;
    run.delivered_command_count = 2;
    run.controller_tick_count = 2;
    run.skipped_controller_period_count = 1;
    run.p50_controller_lateness = std::chrono::nanoseconds{7};
    run.p95_controller_lateness = std::chrono::nanoseconds{19};
    run.p99_controller_lateness = std::chrono::nanoseconds{23};
    run.maximum_controller_lateness = std::chrono::nanoseconds{23};
    run.maximum_control_call_duration = std::chrono::nanoseconds{47};
    run.wall_time = std::chrono::milliseconds{81};

    InputTelemetrySample accepted_input{};
    accepted_input.received_time = ControllerTime{101};
    accepted_input.sender_time = SenderTime{11};
    accepted_input.commanded_positions = {{0.0, 0.1, 0.2, 0.3, 0.4, 0.5}};
    accepted_input.result = CommandResult::Accepted;
    run.input_telemetry.push_back(accepted_input);

    InputTelemetrySample rejected_input{};
    rejected_input.received_time = ControllerTime{102};
    rejected_input.sender_time = SenderTime{12};
    rejected_input.commanded_positions[0] = std::numeric_limits<double>::quiet_NaN();
    rejected_input.result = CommandResult::RejectedPositionNotFinite;
    run.input_telemetry.push_back(rejected_input);

    ControlTelemetrySample target_output{};
    target_output.controller_time = ControllerTime{201};
    target_output.target_positions = {{1.0, 1.1, 1.2, 1.3, 1.4, 1.5}};
    target_output.segment_elapsed_nanoseconds = 2;
    target_output.segment_duration_nanoseconds = 40;
    target_output.result = ControlResult::TargetProduced;
    target_output.state = InterpolatorState::Running;
    target_output.has_target = true;
    run.control_telemetry.push_back(target_output);

    ControlTelemetrySample fault_output{};
    fault_output.controller_time = ControllerTime{202};
    fault_output.result = ControlResult::Faulted;
    fault_output.state = InterpolatorState::Faulted;
    fault_output.fault_reason = FaultReason::TrajectoryVelocityLimitExceeded;
    fault_output.failed_command = 2;
    fault_output.failed_joint = 1;
    fault_output.trajectory_failure_time_seconds = 0.02;
    fault_output.trajectory_failure_value = 3.5;
    run.control_telemetry.push_back(fault_output);

    run.controller_timing.push_back(ControllerTimingSample{
        std::chrono::nanoseconds{0}, std::chrono::nanoseconds{7},
        std::chrono::nanoseconds{7}, std::chrono::nanoseconds{47}});
    run.controller_timing.push_back(ControllerTimingSample{
        std::chrono::nanoseconds{2'000'000}, std::chrono::nanoseconds{2'000'023},
        std::chrono::nanoseconds{23}, std::chrono::nanoseconds{41}});

    const RunReportWriter writer;

    expectEqual(writer.write(run, test_directory.path), ReportWriteResult::Written,
                "complete report is written");

    const std::filesystem::path summary_path = test_directory.path / "summary.csv";
    const std::filesystem::path input_path = test_directory.path / "input_telemetry.csv";
    const std::filesystem::path control_path = test_directory.path / "control_telemetry.csv";
    const std::filesystem::path timing_path = test_directory.path / "controller_timing.csv";

    expectTrue(std::filesystem::exists(summary_path), "summary file exists");
    expectTrue(std::filesystem::exists(input_path), "input telemetry file exists");
    expectTrue(std::filesystem::exists(control_path), "control telemetry file exists");
    expectTrue(std::filesystem::exists(timing_path), "controller timing file exists");
    expectTrue(!std::filesystem::exists(summary_path.string() + ".tmp"),
               "summary temporary file is finalized");
    expectTrue(!std::filesystem::exists(input_path.string() + ".tmp"),
               "input temporary file is finalized");
    expectTrue(!std::filesystem::exists(control_path.string() + ".tmp"),
               "control temporary file is finalized");
    expectTrue(!std::filesystem::exists(test_directory.path.string() + ".tmp"),
               "report staging directory is atomically finalized");

    const std::string summary = readFile(summary_path);
    const std::string input = readFile(input_path);
    const std::string control = readFile(control_path);
    const std::string timing = readFile(timing_path);

    expectTrue(summary.find("3,Faulted,Faulted,TrajectoryVelocityLimitExceeded,Complete,"
                            "WindowsHighResolutionWaitableTimer,PortableConditionVariable,"
                            "3,Applied,") != std::string::npos,
               "summary contains stable wait and affinity names");
    expectTrue(summary.find("300000000,2000000,1000000,10000000,2,2,1,1,2,1,") != std::string::npos,
               "summary contains configuration and aggregate counts");
    expectTrue(input.find("101,11,Accepted,0,0.10000000000000001") != std::string::npos,
               "input CSV preserves timestamps and double precision");
    expectTrue(input.find("102,12,RejectedPositionNotFinite,nan") != std::string::npos,
               "input CSV represents non-finite test data consistently");
    expectTrue(control.find("201,TargetProduced,Running,None,1,2,40,,,") != std::string::npos,
               "control CSV leaves absent failure indices empty");
    expectTrue(control.find("202,Faulted,Faulted,TrajectoryVelocityLimitExceeded,0,0,0,2,1,") !=
                   std::string::npos,
               "control CSV records terminal failure details");
    expectTrue(timing.find("0,7,7,47") != std::string::npos,
               "timing CSV records the first deadline observation");
    expectTrue(timing.find("2000000,2000023,23,41") != std::string::npos,
               "timing CSV records subsequent deadline observations");

    expectEqual(writer.write(run, test_directory.path), ReportWriteResult::OutputAlreadyExists,
                "existing report is never overwritten");

    expectEqual(writer.write(run, {}), ReportWriteResult::InvalidOutputDirectory,
                "empty report path is rejected");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " report writer test(s) failed\n";
        return 1;
    }

    std::cout << "All run report writer tests passed\n";
    return 0;
}
