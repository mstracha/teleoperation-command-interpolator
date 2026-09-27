# Teleoperation Command Interpolator

The Teleoperation Command Interpolator is an independent educational C++17 demonstration of bounded command transfer, smooth six-joint target generation, explicit controller state, and evidence-preserving shutdown. It models a jittery 15–30 Hz producer feeding a controller on a nominal 500 Hz schedule without claiming to control a particular robot or transport.

The project is useful for studying real-time-oriented data ownership, SPSC ring buffers, strong clock-domain types, nonuniform derivative estimation, transactional quintic construction, continuous trajectory validation, and deterministic versus real-thread testing.

> **Safety boundary:** This software is not a certified robot controller. The included joint limits are fictional, and the project does not validate a hardware plant, drive, emergency-stop circuit, safety PLC, fieldbus, operating-system deadline, or formal safety case. Do not use it to command physical hardware without a complete system-level safety design and qualified review.

## Architecture

A producer calls `teleoperation::CommandInterpolator::submitCommand` with a time-ordered `TargetCommand`. Accepted commands pass through a bounded SPSC ring. Once four commands are available, the controller maps its monotonic clock to the sender timeline, estimates shared boundary derivatives, and transactionally constructs six quintic segments. `updateTarget` is designed for a nominal 500 Hz caller and publishes a target only after complete position, velocity, acceleration, and jerk validation. The library does not schedule that caller or claim that a general-purpose operating system meets every 2 ms deadline.

The lifecycle is explicit:

- `Buffering` waits for a usable four-command window.
- `Running` evaluates a validated trajectory.
- `Holding` preserves the last valid target during a temporary underrun.
- `Faulted` is terminal for the controller session and preserves diagnostic evidence.

Separate input and control telemetry rings allow a logger to drain both producer streams. The virtual-time harness provides exact replay, while the three-thread harness exercises command delivery, controller updates, log drain, and coordinated shutdown.

The real-thread harness also records one scheduling observation per controller tick. On Windows it can compare the portable condition-variable wait with a high-resolution waitable timer and can optionally pin only the controller thread to a caller-selected logical processor. These are measurement controls, not production scheduling guarantees.

## Requirements

- CMake 3.20 or newer
- A C++17 compiler
- Threads supported by the platform toolchain

The verified Windows baseline uses 64-bit MSVC 19.50.35720.0 in Release mode. Continuous integration also builds with GCC on Ubuntu.

## VS Code setup

Install and enable:

- Microsoft C/C++ (`ms-vscode.cpptools`)
- Microsoft CMake Tools (`ms-vscode.cmake-tools`)

Reload VS Code, open this repository as the workspace folder, select a 64-bit Visual Studio compiler kit, and choose the `MSVC x64 Release` configure preset.

## Build and test

On the documented Windows toolchain:

```powershell
cmake --preset msvc-release
cmake --build --preset msvc-release
ctest --preset msvc-release
```

For another single-configuration generator:

```sh
cmake -S . -B out/build/release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build out/build/release
ctest --test-dir out/build/release --output-on-failure
```

The expected baseline is ten passing CTest suites.

## Timing qualification

When `TELEOPERATION_BUILD_TOOLS` is enabled (the default), the build also creates `timing_qualification`. It performs repeated one-second trials using identical command streams and writes comparison-ready CSV to standard output.

```powershell
# Ten portable-versus-automatic trials.
./out/build/msvc-release/Release/timing_qualification.exe 10

# Add an automatic-wait trial pinned to logical processor 0.
./out/build/msvc-release/Release/timing_qualification.exe 10 0
```

Each row reports the actual wait strategy, affinity result, executed and skipped ticks, observed frequency, p50/p95/p99/maximum lateness, maximum update duration, and wall time. Configuration order rotates between trials to reduce systematic first-run bias. Redirect standard output to a file to retain an experiment. Processor affinity is deliberately opt-in because an unsuitable processor can make timing worse.

## Use as a CMake package

Install the library to a local prefix:

```sh
cmake --install out/build/release --prefix out/install
```

A consumer can then use:

```cmake
find_package(TeleoperationCommandInterpolator 1.0 REQUIRED)
target_link_libraries(your_target PRIVATE teleoperation::command_interpolator)
```

See `examples/consumer` for a minimal independent consumer.

## Documentation

- [Design, safety rationale, and verification record](docs/design/TeleoperationCommandInterpolatorDesign.md)
- [Printable design document](docs/design/exports/TeleoperationCommandInterpolatorDesign.pdf)
- [Run-report schema](docs/RunReportSchema.md)

The design package contains editable Mermaid sources and generated SVG, HTML, and PDF editions. It records the complete ten-suite behavioral inventory, class-level responsibilities, language choices, safety boundaries, and hypotheses for future adversarial testing.

## Repository structure

```text
include/teleoperation/   Public C++ headers
src/                     Library implementations
tests/                   Ten behavioral test programs
tools/                   Optional timing-qualification executable
examples/consumer/       Installed-package consumer example
docs/design/             Canonical design sources and generated editions
cmake/                   Installed-package configuration helpers
.github/workflows/       Windows/MSVC and Ubuntu/GCC verification
out/                     Ignored local build and verification output
```

## Current limitations

- No hardware feedback, plant model, drive status, or following-error policy
- No safety-rated stop output or independent safety controller
- No transport, serialization, ROS 2, DDS, shared-memory, or gRPC integration
- No clock-synchronization guarantee between sender and controller
- No continuous-angle topology or revolute-joint unwrapping
- No real-time operating-system, thread-priority, process-isolation, or memory-locking guarantee
- Optional Windows controller-thread affinity is diagnostic and does not make execution hard real time
- No dynamic retiming when a requested segment violates limits
- Jerk is bounded within each segment but is not guaranteed continuous across segment joins

See [SECURITY.md](SECURITY.md) for private vulnerability reporting guidance and [CONTRIBUTING.md](CONTRIBUTING.md) for change requirements.

## License and acknowledgments

Copyright © 2026 Mark Strachan. Released under the [MIT License](LICENSE).

The design, implementation, tests, and documentation were developed collaboratively with OpenAI Codex. See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md).
