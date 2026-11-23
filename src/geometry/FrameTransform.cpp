#include "FrameTransform.h"

#include <cmath>
#include <limits>

namespace gyrostab
{

bool transformImuToPlatform(const Vector3f& vectorInSensorFrame,
                            const MountingConfig& mountingConfig,
                            Vector3f& vectorInPlatformFrame)
{
    if (!std::isfinite(vectorInSensorFrame.x) ||
        !std::isfinite(vectorInSensorFrame.y) ||
        !std::isfinite(vectorInSensorFrame.z))
    {
        return false;
    }

    const double sensorX = vectorInSensorFrame.x;
    const double sensorY = vectorInSensorFrame.y;
    const double sensorZ = vectorInSensorFrame.z;

    const double platformX =
        sensorX * mountingConfig.platformXInSensorFrame.x +
        sensorY * mountingConfig.platformXInSensorFrame.y +
        sensorZ * mountingConfig.platformXInSensorFrame.z;
    const double platformY =
        sensorX * mountingConfig.platformYInSensorFrame.x +
        sensorY * mountingConfig.platformYInSensorFrame.y +
        sensorZ * mountingConfig.platformYInSensorFrame.z;
    const double platformZ =
        sensorX * mountingConfig.platformZInSensorFrame.x +
        sensorY * mountingConfig.platformZInSensorFrame.y +
        sensorZ * mountingConfig.platformZInSensorFrame.z;

    const double maximumFloat = static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(platformX) || !std::isfinite(platformY) ||
        !std::isfinite(platformZ) || std::fabs(platformX) > maximumFloat ||
        std::fabs(platformY) > maximumFloat || std::fabs(platformZ) > maximumFloat)
    {
        return false;
    }

    const Vector3f candidate = {
        static_cast<float>(platformX),
        static_cast<float>(platformY),
        static_cast<float>(platformZ)
    };

    vectorInPlatformFrame = candidate;
    return true;
}

} // namespace gyrostab
