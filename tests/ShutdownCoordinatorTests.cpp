#include "teleoperation/ShutdownCoordinator.hpp"

#include <iostream>

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
}

int main()
{
    ShutdownCoordinator coordinator;

    expectEqual(coordinator.phase(), ShutdownPhase::Running, "coordinator starts Running");

    expectEqual(coordinator.stopRequested(), false, "no stop is initially requested");

    expectEqual(coordinator.faultReason(), FaultReason::None, "no fault is initially latched");

    expectEqual(coordinator.requestStop(FaultReason::None), false,
                "None cannot request fault-driven shutdown");

    expectEqual(coordinator.requestStop(FaultReason::ReceiveTimeout), true,
                "first fault requests shutdown");

    expectEqual(coordinator.phase(), ShutdownPhase::StopRequested,
                "fault moves lifecycle to StopRequested");

    expectEqual(coordinator.stopRequested(), true, "threads can observe the stop request");

    expectEqual(coordinator.requestStop(FaultReason::ControllerTimeRegression), false,
                "later faults do not restart shutdown");

    expectEqual(coordinator.faultReason(), FaultReason::ReceiveTimeout,
                "first fault reason remains latched");

    expectEqual(coordinator.markLoggingComplete(), false,
                "logging cannot complete before drain begins");

    expectEqual(coordinator.beginLogDrain(), true, "logger begins draining after stop request");

    expectEqual(coordinator.phase(), ShutdownPhase::DrainingLogs, "lifecycle reports DrainingLogs");

    expectEqual(coordinator.markLoggingComplete(), false,
                "logging waits for both telemetry producers");

    coordinator.markControllerStopped();

    expectEqual(coordinator.producersStopped(), false, "network producer is still active");

    coordinator.markNetworkStopped();

    expectEqual(coordinator.producersStopped(), true, "both telemetry producers are stopped");

    expectEqual(coordinator.markLoggingComplete(), true,
                "logger completes after producers stop and queues are drained");

    expectEqual(coordinator.phase(), ShutdownPhase::Complete,
                "shutdown lifecycle reaches Complete");

    expectEqual(coordinator.markLoggingComplete(), false,
                "completed lifecycle cannot complete twice");

    ShutdownCoordinator early_producer_completion;
    early_producer_completion.markControllerStopped();
    early_producer_completion.markNetworkStopped();

    expectEqual(early_producer_completion.requestStop(FaultReason::ExcessiveSenderGap), true,
                "shutdown can begin after producers already stopped");

    expectEqual(early_producer_completion.beginLogDrain(), true,
                "pre-stopped producers still enter log drain");

    expectEqual(early_producer_completion.markLoggingComplete(), true,
                "pre-stopped producers allow logging completion");

    ShutdownCoordinator normal_completion;

    expectEqual(normal_completion.requestStop(), true,
                "normal completion can request orderly shutdown");

    expectEqual(normal_completion.faultReason(), FaultReason::None,
                "normal completion does not invent a fault reason");

    normal_completion.markControllerStopped();
    normal_completion.markNetworkStopped();

    expectEqual(normal_completion.beginLogDrain(), true, "normal completion enters log drain");

    expectEqual(normal_completion.markLoggingComplete(), true,
                "normal completion reaches terminal shutdown phase");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " shutdown test(s) failed\n";
        return 1;
    }

    std::cout << "All shutdown coordinator tests passed\n";
    return 0;
}
