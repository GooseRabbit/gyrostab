#pragma once

#include "MountingConfig.h"

namespace gyrostab
{

// Преобразует конечный вектор из координат IMU в координаты платформы.
// mountingConfig должна быть предварительно проверена validateMountingConfig().
// При ошибке vectorInPlatformFrame остаётся без изменений.
bool transformImuToPlatform(const Vector3f& vectorInSensorFrame,
                            const MountingConfig& mountingConfig,
                            Vector3f& vectorInPlatformFrame);

} // namespace gyrostab
