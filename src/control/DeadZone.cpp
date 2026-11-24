#include "DeadZone.h"

#include <cmath>

namespace gyrostab
{

bool updateDeadZoneState(float absoluteErrorRad,
                         float enterThresholdRad,
                         float exitThresholdRad,
                         DeadZoneState& state)
{
    if (!std::isfinite(absoluteErrorRad) ||
        !std::isfinite(enterThresholdRad) ||
        !std::isfinite(exitThresholdRad) ||
        absoluteErrorRad < 0.0f ||
        enterThresholdRad < 0.0f ||
        exitThresholdRad <= enterThresholdRad)
    {
        return false;
    }

    bool candidateActive = state.active;
    if (candidateActive)
    {
        if (absoluteErrorRad > exitThresholdRad)
        {
            candidateActive = false;
        }
    }
    else if (absoluteErrorRad <= enterThresholdRad)
    {
        candidateActive = true;
    }

    state.active = candidateActive;
    return true;
}

} // namespace gyrostab
