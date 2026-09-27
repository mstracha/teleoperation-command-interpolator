#include "teleoperation/InterpolatorStateMachine.hpp"

namespace teleoperation
{
    InterpolatorState InterpolatorStateMachine::state() const noexcept
    {
        return state_;
    }

    StateTransitionResult InterpolatorStateMachine::transitionTo(
        InterpolatorState next_state) noexcept
    {
        if (next_state == state_)
        {
            return StateTransitionResult::Unchanged;
        }

        if (!isAllowed(state_, next_state))
        {
            return StateTransitionResult::Rejected;
        }

        state_ = next_state;
        return StateTransitionResult::Transitioned;
    }

    bool InterpolatorStateMachine::isAllowed(InterpolatorState current_state,
                                             InterpolatorState next_state) noexcept
    {
        switch (current_state)
        {
            case InterpolatorState::Buffering:
                return next_state == InterpolatorState::Running ||
                       next_state == InterpolatorState::Faulted;

            case InterpolatorState::Running:
                return next_state == InterpolatorState::Holding ||
                       next_state == InterpolatorState::Faulted;

            case InterpolatorState::Holding:
                return next_state == InterpolatorState::Running ||
                       next_state == InterpolatorState::Faulted;

            case InterpolatorState::Faulted:
                return false;
        }

        return false;
    }
}
