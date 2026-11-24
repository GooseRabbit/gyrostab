#pragma once

#include "../orientation/OrientationEstimator.h"

namespace gyrostab
{

// Сохранённая опорная ориентация, захваченная по команде вызывающего кода.
struct ReferenceOrientation
{
    Quaternionf quaternion;
    bool valid;
};

// Копирует текущую оценку в опору, если она валидна и проходит проверку нормы.
// При отказе reference остаётся без изменений.
bool captureReferenceOrientation(const OrientationEstimate& estimate,
                                 float maximumNormDeviation,
                                 ReferenceOrientation& reference);

} // namespace gyrostab
