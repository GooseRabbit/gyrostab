#pragma once

#include "../geometry/AxisConfig.h"
#include "../geometry/ReferenceOrientation.h"

namespace gyrostab
{

// Вычисляет signed-ошибку по текущей оценке, опорной ориентации и оси управления.
// maximumNormDeviation передаётся существующим проверкам quaternion и оси.
// При ошибке errorRad остаётся без изменений.
bool computeControlError(const OrientationEstimate& current,
                         const ReferenceOrientation& reference,
                         const AxisConfig& controlAxis,
                         float maximumNormDeviation,
                         float& errorRad);

} // namespace gyrostab
