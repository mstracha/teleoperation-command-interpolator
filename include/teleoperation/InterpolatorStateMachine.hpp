#pragma once

namespace teleoperation
{
    enum class InterpolatorState
    {
        Buffering,
        Running,
        Holding,
        Faulted
    };

    enum class StateTransitionResult
    {
        Transitioned,
        Unchanged,
        Rejected
    };

    class InterpolatorStateMachine
    {
      public:
        // The controller thread owns this object. It deliberately contains
        // no locking or atomic state of its own.
        InterpolatorState state() const noexcept;

        StateTransitionResult transitionTo(InterpolatorState next_state) noexcept;

      private:
        static bool isAllowed(InterpolatorState current_state,
                              InterpolatorState next_state) noexcept;

        InterpolatorState state_{InterpolatorState::Buffering};
    };
}
