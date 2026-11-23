#include "MountingConfig.h"

#include <cmath>

namespace gyrostab
{

bool validateMountingConfig(const MountingConfig& config, float maximumDeviation)
{
    if (!std::isfinite(maximumDeviation) || maximumDeviation < 0.0f ||
        maximumDeviation >= 1.0f)
    {
        return false;
    }

    const double xx = config.platformXInSensorFrame.x;
    const double xy = config.platformXInSensorFrame.y;
    const double xz = config.platformXInSensorFrame.z;
    const double yx = config.platformYInSensorFrame.x;
    const double yy = config.platformYInSensorFrame.y;
    const double yz = config.platformYInSensorFrame.z;
    const double zx = config.platformZInSensorFrame.x;
    const double zy = config.platformZInSensorFrame.y;
    const double zz = config.platformZInSensorFrame.z;

    if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(xz) ||
        !std::isfinite(yx) || !std::isfinite(yy) || !std::isfinite(yz) ||
        !std::isfinite(zx) || !std::isfinite(zy) || !std::isfinite(zz))
    {
        return false;
    }

    const double xLength = std::sqrt(xx * xx + xy * xy + xz * xz);
    const double yLength = std::sqrt(yx * yx + yy * yy + yz * yz);
    const double zLength = std::sqrt(zx * zx + zy * zy + zz * zz);
    const double tolerance = static_cast<double>(maximumDeviation);

    if (std::fabs(xLength - 1.0) > tolerance ||
        std::fabs(yLength - 1.0) > tolerance ||
        std::fabs(zLength - 1.0) > tolerance)
    {
        return false;
    }

    const double xyDot = xx * yx + xy * yy + xz * yz;
    const double xzDot = xx * zx + xy * zy + xz * zz;
    const double yzDot = yx * zx + yy * zy + yz * zz;
    if (std::fabs(xyDot) > tolerance || std::fabs(xzDot) > tolerance ||
        std::fabs(yzDot) > tolerance)
    {
        return false;
    }

    // Отрицательное смешанное произведение означает левую систему координат.
    const double handedness =
        (xy * yz - xz * yy) * zx +
        (xz * yx - xx * yz) * zy +
        (xx * yy - xy * yx) * zz;
    return handedness > 0.0;
}

} // namespace gyrostab
