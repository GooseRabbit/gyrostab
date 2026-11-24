#include "GyroRateProjection.h"

#include <cmath>
#include <limits>

namespace gyrostab
{

bool projectGyroRateOnControlAxis(const Vector3f& angularVelocityRadPerSec,
                                  const Vector3f& gyroBiasRadPerSec,
                                  const AxisConfig& controlAxis,
                                  float maximumAxisNormDeviation,
                                  float& projectedRateRadPerSec)
{
    if (!std::isfinite(maximumAxisNormDeviation) || maximumAxisNormDeviation < 0.0f ||
        !std::isfinite(angularVelocityRadPerSec.x) ||
        !std::isfinite(angularVelocityRadPerSec.y) ||
        !std::isfinite(angularVelocityRadPerSec.z) ||
        !std::isfinite(gyroBiasRadPerSec.x) ||
        !std::isfinite(gyroBiasRadPerSec.y) ||
        !std::isfinite(gyroBiasRadPerSec.z) ||
        !std::isfinite(controlAxis.axisInSensorFrame.x) ||
        !std::isfinite(controlAxis.axisInSensorFrame.y) ||
        !std::isfinite(controlAxis.axisInSensorFrame.z))
    {
        return false;
    }

    const double axisX = controlAxis.axisInSensorFrame.x;
    const double axisY = controlAxis.axisInSensorFrame.y;
    const double axisZ = controlAxis.axisInSensorFrame.z;
    const double axisNorm = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (!std::isfinite(axisNorm) || axisNorm <= 0.0 ||
        std::fabs(axisNorm - 1.0) > static_cast<double>(maximumAxisNormDeviation))
    {
        return false;
    }

    const double correctedX = static_cast<double>(angularVelocityRadPerSec.x) -
                              static_cast<double>(gyroBiasRadPerSec.x);
    const double correctedY = static_cast<double>(angularVelocityRadPerSec.y) -
                              static_cast<double>(gyroBiasRadPerSec.y);
    const double correctedZ = static_cast<double>(angularVelocityRadPerSec.z) -
                              static_cast<double>(gyroBiasRadPerSec.z);

    const double candidateRate = correctedX * axisX +
                                 correctedY * axisY +
                                 correctedZ * axisZ;
    if (!std::isfinite(candidateRate) ||
        std::fabs(candidateRate) > static_cast<double>(std::numeric_limits<float>::max()))
    {
        return false;
    }

    const float candidate = static_cast<float>(candidateRate);
    if (!std::isfinite(candidate))
    {
        return false;
    }

    projectedRateRadPerSec = candidate;
    return true;
}

} // namespace gyrostab
