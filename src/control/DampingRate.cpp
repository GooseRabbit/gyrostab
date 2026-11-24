#include "DampingRate.h"

#include "../geometry/GyroRateProjection.h"

namespace gyrostab
{

bool computeDampingRate(const ImuSample& sample,
                        const Vector3f& gyroBiasRadPerSec,
                        const AxisConfig& controlAxis,
                        float maximumAxisNormDeviation,
                        float& rateRadPerSec)
{
    if (!sample.gyroscopeValid)
    {
        return false;
    }

    return projectGyroRateOnControlAxis(sample.angularVelocityRadPerSec,
                                        gyroBiasRadPerSec,
                                        controlAxis,
                                        maximumAxisNormDeviation,
                                        rateRadPerSec);
}

} // namespace gyrostab
