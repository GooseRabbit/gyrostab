#pragma once

#include "../geometry/AxisConfig.h"
#include "../sensors/ImuDriver.h"

namespace gyrostab
{

// Получает signed angular rate для D-составляющей по новому IMU sample.
// Свежесть sample проверяет вызывающий код; здесь проверяется gyroscopeValid.
// maximumAxisNormDeviation передаётся существующей проекции оси.
// При ошибке rateRadPerSec остаётся без изменений.
bool computeDampingRate(const ImuSample& sample,
                        const Vector3f& gyroBiasRadPerSec,
                        const AxisConfig& controlAxis,
                        float maximumAxisNormDeviation,
                        float& rateRadPerSec);

} // namespace gyrostab
