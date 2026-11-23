#pragma once

#include <stdint.h>

namespace gyrostab
{

// Кватернион ориентации. Компоненты записаны в порядке scalar-first: w, x, y, z.
struct Quaternionf
{
    float w;
    float x;
    float y;
    float z;
};

// Последняя оценка ориентации и метаданные использованного образца IMU.
// При valid == false quaternion содержит только нейтральное начальное значение.
struct OrientationEstimate
{
    Quaternionf quaternion;
    uint32_t timestampUs;
    uint32_t sequence;
    bool valid;
};

// Задаёт нейтральное начальное состояние, не объявляя его валидной оценкой.
void initializeEstimator(OrientationEstimate& estimate);

} // namespace gyrostab
