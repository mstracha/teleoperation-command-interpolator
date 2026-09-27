#include <teleoperation/CommandInterpolator.hpp>

int main()
{
    teleoperation::CommandInterpolator interpolator;
    return interpolator.state() == teleoperation::InterpolatorState::Buffering ? 0 : 1;
}
