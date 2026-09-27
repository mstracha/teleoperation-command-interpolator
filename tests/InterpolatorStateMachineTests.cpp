#include "teleoperation/InterpolatorStateMachine.hpp"

#include <iostream>

using namespace teleoperation;

namespace
{
    int failure_count = 0;

    void expectState(const InterpolatorStateMachine &state_machine, InterpolatorState expected,
                     const char *test_name)
    {
        if (state_machine.state() != expected)
        {
            std::cerr << "FAILED: " << test_name << '\n';
            ++failure_count;
        }
    }

    void expectTransition(InterpolatorStateMachine &state_machine, InterpolatorState next_state,
                          StateTransitionResult expected_result, InterpolatorState expected_state,
                          const char *test_name)
    {
        const StateTransitionResult result = state_machine.transitionTo(next_state);

        if (result != expected_result)
        {
            std::cerr << "FAILED result: " << test_name << '\n';
            ++failure_count;
        }

        expectState(state_machine, expected_state, test_name);
    }
}

int main()
{
    InterpolatorStateMachine state_machine;

    expectState(state_machine, InterpolatorState::Buffering, "initial state is Buffering");

    expectTransition(state_machine, InterpolatorState::Buffering, StateTransitionResult::Unchanged,
                     InterpolatorState::Buffering, "repeating the current state is harmless");

    expectTransition(state_machine, InterpolatorState::Holding, StateTransitionResult::Rejected,
                     InterpolatorState::Buffering, "Buffering cannot skip directly to Holding");

    expectTransition(state_machine, InterpolatorState::Running, StateTransitionResult::Transitioned,
                     InterpolatorState::Running, "Buffering transitions to Running");

    expectTransition(state_machine, InterpolatorState::Buffering, StateTransitionResult::Rejected,
                     InterpolatorState::Running, "Running cannot silently reset to Buffering");

    expectTransition(state_machine, InterpolatorState::Holding, StateTransitionResult::Transitioned,
                     InterpolatorState::Holding, "Running transitions to Holding");

    expectTransition(state_machine, InterpolatorState::Buffering, StateTransitionResult::Rejected,
                     InterpolatorState::Holding, "Holding cannot silently reset to Buffering");

    expectTransition(state_machine, InterpolatorState::Running, StateTransitionResult::Transitioned,
                     InterpolatorState::Running, "Holding resumes Running");

    expectTransition(state_machine, InterpolatorState::Faulted, StateTransitionResult::Transitioned,
                     InterpolatorState::Faulted, "Running transitions to Faulted");

    expectTransition(state_machine, InterpolatorState::Running, StateTransitionResult::Rejected,
                     InterpolatorState::Faulted, "Faulted cannot resume without reset");

    expectTransition(state_machine, InterpolatorState::Holding, StateTransitionResult::Rejected,
                     InterpolatorState::Faulted, "Faulted cannot enter Holding");

    expectTransition(state_machine, InterpolatorState::Buffering, StateTransitionResult::Rejected,
                     InterpolatorState::Faulted, "Faulted is terminal for this controller session");

    expectTransition(state_machine, InterpolatorState::Faulted, StateTransitionResult::Unchanged,
                     InterpolatorState::Faulted, "repeating terminal Faulted is harmless");

    InterpolatorStateMachine buffering_fault_state_machine;

    expectTransition(buffering_fault_state_machine, InterpolatorState::Faulted,
                     StateTransitionResult::Transitioned, InterpolatorState::Faulted,
                     "Buffering can fault before startup completes");

    InterpolatorStateMachine holding_fault_state_machine;

    expectTransition(holding_fault_state_machine, InterpolatorState::Running,
                     StateTransitionResult::Transitioned, InterpolatorState::Running,
                     "second machine starts Running sequence");

    expectTransition(holding_fault_state_machine, InterpolatorState::Holding,
                     StateTransitionResult::Transitioned, InterpolatorState::Holding,
                     "second machine enters Holding");

    expectTransition(holding_fault_state_machine, InterpolatorState::Faulted,
                     StateTransitionResult::Transitioned, InterpolatorState::Faulted,
                     "Holding transitions to Faulted");

    if (failure_count != 0)
    {
        std::cerr << failure_count << " state-machine test(s) failed\n";
        return 1;
    }

    std::cout << "All state-machine tests passed\n";
    return 0;
}
