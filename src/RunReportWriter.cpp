#include "teleoperation/RunReportWriter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <string_view>
#include <system_error>

namespace teleoperation
{
    namespace
    {
        constexpr std::string_view SummaryFileName = "summary.csv";
        constexpr std::string_view InputTelemetryFileName = "input_telemetry.csv";
        constexpr std::string_view ControlTelemetryFileName = "control_telemetry.csv";
        constexpr std::string_view ControllerTimingFileName = "controller_timing.csv";
        constexpr std::string_view TemporarySuffix = ".tmp";

        std::string_view toString(RealTimeRunOutcome outcome) noexcept
        {
            switch (outcome)
            {
                case RealTimeRunOutcome::Completed:
                    return "Completed";
                case RealTimeRunOutcome::Faulted:
                    return "Faulted";
                case RealTimeRunOutcome::InvalidRunDuration:
                    return "InvalidRunDuration";
                case RealTimeRunOutcome::InvalidControllerPeriod:
                    return "InvalidControllerPeriod";
                case RealTimeRunOutcome::InvalidLoggerPeriod:
                    return "InvalidLoggerPeriod";
                case RealTimeRunOutcome::InvalidStartupDelay:
                    return "InvalidStartupDelay";
                case RealTimeRunOutcome::DeliveryOffsetsOutOfOrder:
                    return "DeliveryOffsetsOutOfOrder";
                case RealTimeRunOutcome::ControllerTickLimitExceeded:
                    return "ControllerTickLimitExceeded";
            }

            return "Unknown";
        }

        std::string_view toString(ControllerWaitStrategy strategy) noexcept
        {
            switch (strategy)
            {
                case ControllerWaitStrategy::PortableConditionVariable:
                    return "PortableConditionVariable";
                case ControllerWaitStrategy::WindowsHighResolutionWaitableTimer:
                    return "WindowsHighResolutionWaitableTimer";
            }

            return "Unknown";
        }

        std::string_view toString(ControllerWaitPreference preference) noexcept
        {
            switch (preference)
            {
                case ControllerWaitPreference::Automatic:
                    return "Automatic";
                case ControllerWaitPreference::PortableConditionVariable:
                    return "PortableConditionVariable";
                case ControllerWaitPreference::WindowsHighResolutionWaitableTimer:
                    return "WindowsHighResolutionWaitableTimer";
            }

            return "Unknown";
        }

        std::string_view toString(ControllerAffinityResult result) noexcept
        {
            switch (result)
            {
                case ControllerAffinityResult::NotRequested:
                    return "NotRequested";
                case ControllerAffinityResult::Applied:
                    return "Applied";
                case ControllerAffinityResult::Unsupported:
                    return "Unsupported";
                case ControllerAffinityResult::InvalidLogicalProcessor:
                    return "InvalidLogicalProcessor";
                case ControllerAffinityResult::ApplicationFailed:
                    return "ApplicationFailed";
            }

            return "Unknown";
        }

        std::string_view toString(CommandResult result) noexcept
        {
            switch (result)
            {
                case CommandResult::Accepted:
                    return "Accepted";
                case CommandResult::RejectedPositionNotFinite:
                    return "RejectedPositionNotFinite";
                case CommandResult::RejectedPositionOutOfRange:
                    return "RejectedPositionOutOfRange";
                case CommandResult::RejectedDuplicateOrOutOfOrder:
                    return "RejectedDuplicateOrOutOfOrder";
                case CommandResult::RejectedIntervalTooSmall:
                    return "RejectedIntervalTooSmall";
                case CommandResult::RejectedExcessiveSenderGap:
                    return "RejectedExcessiveSenderGap";
                case CommandResult::RejectedBufferFull:
                    return "RejectedBufferFull";
            }

            return "Unknown";
        }

        std::string_view toString(ControlResult result) noexcept
        {
            switch (result)
            {
                case ControlResult::Buffering:
                    return "Buffering";
                case ControlResult::TargetProduced:
                    return "TargetProduced";
                case ControlResult::Holding:
                    return "Holding";
                case ControlResult::Faulted:
                    return "Faulted";
            }

            return "Unknown";
        }

        std::string_view toString(InterpolatorState state) noexcept
        {
            switch (state)
            {
                case InterpolatorState::Buffering:
                    return "Buffering";
                case InterpolatorState::Running:
                    return "Running";
                case InterpolatorState::Holding:
                    return "Holding";
                case InterpolatorState::Faulted:
                    return "Faulted";
            }

            return "Unknown";
        }

        std::string_view toString(FaultReason reason) noexcept
        {
            switch (reason)
            {
                case FaultReason::None:
                    return "None";
                case FaultReason::ExcessiveSenderGap:
                    return "ExcessiveSenderGap";
                case FaultReason::ReceiveTimeout:
                    return "ReceiveTimeout";
                case FaultReason::ControllerTimeRegression:
                    return "ControllerTimeRegression";
                case FaultReason::ControllerTimeBeforeLatestReceive:
                    return "ControllerTimeBeforeLatestReceive";
                case FaultReason::CommandBufferInconsistent:
                    return "CommandBufferInconsistent";
                case FaultReason::InvalidSegmentTiming:
                    return "InvalidSegmentTiming";
                case FaultReason::InvalidStateTransition:
                    return "InvalidStateTransition";
                case FaultReason::MissingHoldTarget:
                    return "MissingHoldTarget";
                case FaultReason::AdvanceLimitExceeded:
                    return "AdvanceLimitExceeded";
                case FaultReason::DerivativeEstimationFailed:
                    return "DerivativeEstimationFailed";
                case FaultReason::TrajectoryConfigurationFailed:
                    return "TrajectoryConfigurationFailed";
                case FaultReason::TrajectoryPositionLimitExceeded:
                    return "TrajectoryPositionLimitExceeded";
                case FaultReason::TrajectoryVelocityLimitExceeded:
                    return "TrajectoryVelocityLimitExceeded";
                case FaultReason::TrajectoryAccelerationLimitExceeded:
                    return "TrajectoryAccelerationLimitExceeded";
                case FaultReason::TrajectoryJerkLimitExceeded:
                    return "TrajectoryJerkLimitExceeded";
                case FaultReason::TrajectoryEvaluationFailed:
                    return "TrajectoryEvaluationFailed";
            }

            return "Unknown";
        }

        std::string_view toString(ShutdownPhase phase) noexcept
        {
            switch (phase)
            {
                case ShutdownPhase::Running:
                    return "Running";
                case ShutdownPhase::StopRequested:
                    return "StopRequested";
                case ShutdownPhase::DrainingLogs:
                    return "DrainingLogs";
                case ShutdownPhase::Complete:
                    return "Complete";
            }

            return "Unknown";
        }

        void writeDouble(std::ostream &output, double value)
        {
            if (std::isnan(value))
            {
                output << "nan";
                return;
            }

            if (std::isinf(value))
            {
                output << (value < 0.0 ? "-inf" : "inf");
                return;
            }

            output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        }

        bool writeSummary(const RealTimeRunResult &run, const std::filesystem::path &path)
        {
            std::ofstream output(path, std::ios::out | std::ios::trunc);

            if (!output)
            {
                return false;
            }

            output.imbue(std::locale::classic());

            const std::size_t accepted_command_count = static_cast<std::size_t>(
                std::count_if(run.input_telemetry.begin(), run.input_telemetry.end(),
                              [](const InputTelemetrySample &sample)
                              { return sample.result == CommandResult::Accepted; }));

            std::array<std::size_t, 4> control_result_counts{};

            for (const ControlTelemetrySample &sample : run.control_telemetry)
            {
                switch (sample.result)
                {
                    case ControlResult::Buffering:
                        ++control_result_counts[0];
                        break;
                    case ControlResult::TargetProduced:
                        ++control_result_counts[1];
                        break;
                    case ControlResult::Holding:
                        ++control_result_counts[2];
                        break;
                    case ControlResult::Faulted:
                        ++control_result_counts[3];
                        break;
                }
            }

            output << "schema_version,outcome,final_state,fault_reason,shutdown_phase,"
                   << "configured_controller_wait_preference,controller_wait_strategy,"
                   << "requested_controller_logical_processor,controller_affinity_result,"
                   << "configured_run_duration_ns,configured_controller_period_ns,"
                   << "configured_logger_poll_period_ns,configured_startup_delay_ns,"
                   << "scheduled_command_count,delivered_command_count,"
                   << "recorded_accepted_command_count,"
                   << "recorded_rejected_command_count,"
                   << "controller_tick_count,skipped_controller_period_count,"
                   << "recorded_buffering_tick_count,"
                   << "recorded_target_produced_tick_count,"
                   << "recorded_holding_tick_count,recorded_faulted_tick_count,"
                   << "controller_timing_sample_count,p50_controller_lateness_ns,"
                   << "p95_controller_lateness_ns,p99_controller_lateness_ns,"
                   << "maximum_controller_lateness_ns,"
                   << "maximum_control_call_duration_ns,wall_time_ns,"
                   << "dropped_input_telemetry_count,"
                   << "dropped_control_telemetry_count,input_telemetry_count,"
                   << "control_telemetry_count\n";

            output << RunReportSchemaVersion << ',' << toString(run.outcome) << ','
                   << toString(run.final_state) << ',' << toString(run.fault_reason) << ','
                   << toString(run.shutdown_phase) << ','
                   << toString(run.configured_controller_wait_preference) << ','
                   << toString(run.controller_wait_strategy) << ',';

            if (run.requested_controller_logical_processor.has_value())
            {
                output << *run.requested_controller_logical_processor;
            }

            output << ',' << toString(run.controller_affinity_result) << ','
                   << run.configured_run_duration.count() << ','
                   << run.configured_controller_period.count() << ','
                   << run.configured_logger_poll_period.count() << ','
                   << run.configured_startup_delay.count() << ',' << run.scheduled_command_count
                   << ',' << run.delivered_command_count << ',' << accepted_command_count << ','
                   << run.input_telemetry.size() - accepted_command_count << ','
                   << run.controller_tick_count << ',' << run.skipped_controller_period_count << ','
                   << control_result_counts[0] << ',' << control_result_counts[1] << ','
                   << control_result_counts[2] << ',' << control_result_counts[3] << ','
                   << run.controller_timing.size() << ','
                   << run.p50_controller_lateness.count() << ','
                   << run.p95_controller_lateness.count() << ','
                   << run.p99_controller_lateness.count() << ','
                   << run.maximum_controller_lateness.count() << ','
                   << run.maximum_control_call_duration.count() << ',' << run.wall_time.count()
                   << ',' << run.dropped_input_telemetry_count << ','
                   << run.dropped_control_telemetry_count << ',' << run.input_telemetry.size()
                   << ',' << run.control_telemetry.size() << '\n';

            output.flush();
            return output.good();
        }

        bool writeControllerTiming(const std::vector<ControllerTimingSample> &samples,
                                   const std::filesystem::path &path)
        {
            std::ofstream output(path, std::ios::out | std::ios::trunc);

            if (!output)
            {
                return false;
            }

            output.imbue(std::locale::classic());
            output << "scheduled_offset_ns,actual_start_offset_ns,lateness_ns,update_duration_ns\n";

            for (const ControllerTimingSample &sample : samples)
            {
                output << sample.scheduled_offset.count() << ','
                       << sample.actual_start_offset.count() << ',' << sample.lateness.count()
                       << ',' << sample.update_duration.count() << '\n';
            }

            output.flush();
            return output.good();
        }

        bool writeInputTelemetry(const std::vector<InputTelemetrySample> &samples,
                                 const std::filesystem::path &path)
        {
            std::ofstream output(path, std::ios::out | std::ios::trunc);

            if (!output)
            {
                return false;
            }

            output.imbue(std::locale::classic());

            output << "received_time_ns,sender_time_ns,command_result,"
                   << "joint_0_position,joint_1_position,joint_2_position,"
                   << "joint_3_position,joint_4_position,joint_5_position\n";

            for (const InputTelemetrySample &sample : samples)
            {
                output << sample.received_time.nanoseconds_since_controller_epoch << ','
                       << sample.sender_time.nanoseconds_since_sender_epoch << ','
                       << toString(sample.result);

                for (double position : sample.commanded_positions)
                {
                    output << ',';
                    writeDouble(output, position);
                }

                output << '\n';
            }

            output.flush();
            return output.good();
        }

        bool writeControlTelemetry(const std::vector<ControlTelemetrySample> &samples,
                                   const std::filesystem::path &path)
        {
            std::ofstream output(path, std::ios::out | std::ios::trunc);

            if (!output)
            {
                return false;
            }

            output.imbue(std::locale::classic());

            output << "controller_time_ns,control_result,state,fault_reason,has_target,"
                   << "segment_elapsed_ns,segment_duration_ns,failed_command_index,"
                   << "failed_joint_index,trajectory_failure_time_seconds,"
                   << "trajectory_failure_value,target_joint_0_position,"
                   << "target_joint_1_position,target_joint_2_position,"
                   << "target_joint_3_position,target_joint_4_position,"
                   << "target_joint_5_position\n";

            for (const ControlTelemetrySample &sample : samples)
            {
                output << sample.controller_time.nanoseconds_since_controller_epoch << ','
                       << toString(sample.result) << ',' << toString(sample.state) << ','
                       << toString(sample.fault_reason) << ',' << (sample.has_target ? 1 : 0) << ','
                       << sample.segment_elapsed_nanoseconds << ','
                       << sample.segment_duration_nanoseconds << ',';

                if (sample.failed_command != ControlTelemetrySample::NoFailedCommand)
                {
                    output << sample.failed_command;
                }

                output << ',';

                if (sample.failed_joint != ControlTelemetrySample::NoFailedJoint)
                {
                    output << sample.failed_joint;
                }

                output << ',';
                writeDouble(output, sample.trajectory_failure_time_seconds);
                output << ',';
                writeDouble(output, sample.trajectory_failure_value);

                for (double position : sample.target_positions)
                {
                    output << ',';
                    writeDouble(output, position);
                }

                output << '\n';
            }

            output.flush();
            return output.good();
        }

        void removeDirectory(const std::filesystem::path &path) noexcept
        {
            std::error_code ignored_error;
            static_cast<void>(std::filesystem::remove_all(path, ignored_error));
        }
    }

    ReportWriteResult RunReportWriter::write(const RealTimeRunResult &run,
                                             const std::filesystem::path &output_directory) const
    {
        if (output_directory.empty())
        {
            return ReportWriteResult::InvalidOutputDirectory;
        }

        std::error_code file_error;
        const bool output_exists = std::filesystem::exists(output_directory, file_error);

        if (file_error)
        {
            return ReportWriteResult::InvalidOutputDirectory;
        }

        if (output_exists)
        {
            const bool is_directory = std::filesystem::is_directory(output_directory, file_error);

            if (file_error || !is_directory)
            {
                return ReportWriteResult::InvalidOutputDirectory;
            }

            return ReportWriteResult::OutputAlreadyExists;
        }

        const std::filesystem::path staging_directory =
            output_directory.string() + std::string{TemporarySuffix};

        if (std::filesystem::exists(staging_directory, file_error))
        {
            return ReportWriteResult::OutputAlreadyExists;
        }

        if (file_error)
        {
            return ReportWriteResult::InvalidOutputDirectory;
        }

        const bool staging_directory_created =
            std::filesystem::create_directories(staging_directory, file_error);

        if (file_error)
        {
            return ReportWriteResult::DirectoryCreationFailed;
        }

        // Another writer may have won the race to claim this staging directory.
        if (!staging_directory_created)
        {
            return ReportWriteResult::OutputAlreadyExists;
        }

        const std::filesystem::path summary_path = staging_directory / SummaryFileName;
        const std::filesystem::path input_path = staging_directory / InputTelemetryFileName;
        const std::filesystem::path control_path = staging_directory / ControlTelemetryFileName;
        const std::filesystem::path controller_timing_path =
            staging_directory / ControllerTimingFileName;

        if (!writeSummary(run, summary_path))
        {
            removeDirectory(staging_directory);
            return ReportWriteResult::SummaryWriteFailed;
        }

        if (!writeInputTelemetry(run.input_telemetry, input_path))
        {
            removeDirectory(staging_directory);
            return ReportWriteResult::InputTelemetryWriteFailed;
        }

        if (!writeControlTelemetry(run.control_telemetry, control_path))
        {
            removeDirectory(staging_directory);
            return ReportWriteResult::ControlTelemetryWriteFailed;
        }

        if (!writeControllerTiming(run.controller_timing, controller_timing_path))
        {
            removeDirectory(staging_directory);
            return ReportWriteResult::ControllerTimingWriteFailed;
        }

        std::filesystem::rename(staging_directory, output_directory, file_error);

        if (file_error)
        {
            removeDirectory(staging_directory);
            return ReportWriteResult::FinalizationFailed;
        }

        return ReportWriteResult::Written;
    }
}
