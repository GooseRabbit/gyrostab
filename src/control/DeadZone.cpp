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

bool applyDeadZone(float positionalErrorRad,
                   const DeadZoneState& state,
                   float& outputErrorRad)
{
    if (!std::isfinite(positionalErrorRad))
    {
        return false;
    }

    const float candidateErrorRad = state.active ? 0.0f : positionalErrorRad;
    outputErrorRad = candidateErrorRad;
    return true;
}

void resetDeadZone(DeadZoneState& state)
{
    state.active = false;
}

} // namespace gyrostab
