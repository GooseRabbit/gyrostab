#pragma once

#include "../sensors/ImuDriver.h"

namespace gyrostab
{

// Выводит один CSV-рекорд с измерениями переданного образца через Serial.
// Частоту вызова определяет вызывающий код; функция не читает IMU и не задерживает цикл.
void publishRawImuTelemetry(const ImuSample& sample);

} // namespace gyrostab
