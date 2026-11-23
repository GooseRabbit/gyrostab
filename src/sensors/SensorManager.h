#pragma once

#include "ImuDriver.h"

namespace gyrostab
{

// Проверяет обязательные каналы IMU и диапазоны, допустимые выбранным full-scale.
// Для проверки нужно передавать ту же ImuConfig, с которой настроен датчик.
// Невалидный магнитометр не отклоняет образец: этот канал пока необязателен.
bool validateSensorSample(const ImuSample& sample, const ImuConfig& config);

// Проверяет, что получен образец с валидными каналами и его возраст не превышает maximumAgeUs.
// nowUs передаётся явно (обычно из micros()), чтобы проверка не зависела от глобального таймера.
// maximumAgeUs должен быть меньше 2^31 для однозначной арифметики времени uint32_t с rollover.
bool isSampleFresh(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs);

} // namespace gyrostab
