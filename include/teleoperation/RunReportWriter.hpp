#pragma once

#include "teleoperation/RealTimeHarness.hpp"

#include <cstdint>
#include <filesystem>

namespace teleoperation
{
    inline constexpr std::uint32_t RunReportSchemaVersion = 3;

    enum class ReportWriteResult : std::uint8_t
    {
        Written,
        InvalidOutputDirectory,
        OutputAlreadyExists,
        DirectoryCreationFailed,
        SummaryWriteFailed,
        InputTelemetryWriteFailed,
        ControlTelemetryWriteFailed,
        ControllerTimingWriteFailed,
        FinalizationFailed
    };

    // Persists a completed real-time run after all producer threads have stopped.
    // Each report directory is write-once so an earlier run cannot be silently
    // replaced by a later test.
    class RunReportWriter
    {
      public:
        ReportWriteResult write(const RealTimeRunResult &run,
                                const std::filesystem::path &output_directory) const;
    };
}
