#pragma once

#include "ImuDriver.h"

namespace gyrostab
{

// Проверяет обязательные каналы IMU и диапазоны, допустимые выбранным full-scale.
// Для проверки нужно передавать ту же ImuConfig, с которой настроен датчик.
// Невалидный магнитометр не отклоняет образец: этот канал пока необязателен.
bool validateSensorSample(const ImuSample& sample, const ImuConfig& config);

} // namespace gyrostab
