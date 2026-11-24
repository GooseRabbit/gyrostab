#pragma once

#include <stdint.h>

#include "../sensors/ImuDriver.h"

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

// Интегрирует bias-corrected gyro. maximumDeltaTimeUs задаётся вызывающим кодом.
// Первый валидный образец только привязывает время/sequence и начинает относительный отсчёт.
// Ориентация задаётся body-to-world кватернионом; инкремент домножается справа.
// Акселерометрическая коррекция и магнитометр здесь не используются.
bool updateOrientation(OrientationEstimate& estimate, const ImuSample& sample,
                       const Vector3f& gyroBiasRadPerSec,
                       uint32_t maximumDeltaTimeUs);


// Проверяет валидность оценки и отклонение нормы quaternion от единицы.
// maximumNormDeviation задаётся вызывающим кодом; состояние не изменяется.
bool validateOrientation(const OrientationEstimate& estimate,
                         float maximumNormDeviation);

// Возвращает копию оценки, если valid и quaternion проходит явный допуск нормы.
// Проверку возраста выполняет отдельно checkEstimatorHealth(); при false output не меняется.
bool getOrientation(const OrientationEstimate& latestEstimate,
                    float maximumNormDeviation,
                    OrientationEstimate& output);

} // namespace gyrostab
