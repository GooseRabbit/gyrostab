#include "ControlAxisProjection.h"

#include <cmath>

namespace gyrostab
{

bool projectErrorOnControlAxis(const Quaternionf& relativeQuaternion,
                               const AxisConfig& controlAxis,
                               float maximumNormDeviation,
                               float& errorRad)
{
    const OrientationEstimate relativeEstimate = {
        relativeQuaternion,
        0U,
        0U,
        true
    };
    if (!validateOrientation(relativeEstimate, maximumNormDeviation))
    {
        return false;
    }

    const double axisX = controlAxis.axisInSensorFrame.x;
    const double axisY = controlAxis.axisInSensorFrame.y;
    const double axisZ = controlAxis.axisInSensorFrame.z;
    if (!std::isfinite(axisX) || !std::isfinite(axisY) || !std::isfinite(axisZ))
    {
        return false;
    }

    const double axisNorm = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (!std::isfinite(axisNorm) || axisNorm <= 0.0 ||
        std::fabs(axisNorm - 1.0) > static_cast<double>(maximumNormDeviation))
    {
        return false;
    }

    double twistW = relativeQuaternion.w;
    const double projectedVector =
        static_cast<double>(relativeQuaternion.x) * axisX +
        static_cast<double>(relativeQuaternion.y) * axisY +
        static_cast<double>(relativeQuaternion.z) * axisZ;
    double twistVector = projectedVector;

    // q and -q describe the same rotation. Choose one representation consistently.
    // At exactly 180 degrees w is zero, so the projected component breaks the tie.
    if (twistW < 0.0 || (twistW == 0.0 && twistVector < 0.0))
    {
        twistW = -twistW;
        twistVector = -twistVector;
    }

    if (twistW == 0.0 && twistVector == 0.0)
    {
        return false;
    }

    const double candidateAngle = 2.0 * std::atan2(twistVector, twistW);
    if (!std::isfinite(candidateAngle))
    {
        return false;
    }

    const float candidate = static_cast<float>(candidateAngle);
    if (!std::isfinite(candidate))
    {
        return false;
    }

    errorRad = candidate;
    return true;
}

} // namespace gyrostab
