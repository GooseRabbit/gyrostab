#pragma once

#include "AxisConfig.h"

namespace gyrostab
{

// Проецирует угловую скорость после вычитания gyro bias на управляемую ось IMU.
// Ось должна быть предварительно нормирована; допуск проверяет её длину.
// При ошибке projectedRateRadPerSec остаётся без изменений.
bool projectGyroRateOnControlAxis(const Vector3f& angularVelocityRadPerSec,
                                  const Vector3f& gyroBiasRadPerSec,
                                  const AxisConfig& controlAxis,
                                  float maximumAxisNormDeviation,
                                  float& projectedRateRadPerSec);

} // namespace gyrostab
