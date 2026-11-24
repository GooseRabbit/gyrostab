#pragma once

#include "AxisConfig.h"
#include "../orientation/OrientationEstimator.h"

namespace gyrostab
{

// Извлекает signed twist error вокруг заранее нормированной оси в координатах IMU.
// maximumNormDeviation применяется к норме quaternion и длине оси.
// При ошибке errorRad остаётся без изменений.
bool projectErrorOnControlAxis(const Quaternionf& relativeQuaternion,
                               const AxisConfig& controlAxis,
                               float maximumNormDeviation,
                               float& errorRad);

} // namespace gyrostab
