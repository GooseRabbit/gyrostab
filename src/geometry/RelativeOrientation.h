#pragma once

#include "ReferenceOrientation.h"

namespace gyrostab
{

// Вычисляет current относительно reference: inverse(reference) * current.
// maximumNormDeviation передаётся вызывающим кодом.
// При ошибке relativeQuaternion остаётся без изменений.
bool computeRelativeOrientation(const OrientationEstimate& current,
                                const ReferenceOrientation& reference,
                                float maximumNormDeviation,
                                Quaternionf& relativeQuaternion);

} // namespace gyrostab
