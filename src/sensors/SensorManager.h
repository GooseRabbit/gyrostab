#pragma once

#include "ImuDriver.h"

namespace gyrostab
{

// Снимок состояния драйвера и последнего проверенного образца.
// hasValidSample и sampleFresh независимы: кэш может содержать устаревшие данные.
struct SensorHealth
{
    ICM_20948_Status_e driverStatus;
    bool hasValidSample;
    bool sampleFresh;
    uint32_t lastSampleTimestampUs;
    uint32_t lastSampleSequence;
};

// Сбрасывает кэш образца и повторно инициализирует IMU по той же BoardConfig.
// После успешного сброса configureImu() нужно вызвать повторно.
bool resetImuDataPath(const BoardConfig& board);

// Проверяет обязательные каналы IMU и диапазоны, допустимые выбранным full-scale.
// Для проверки нужно передавать ту же ImuConfig, с которой настроен датчик.
// Невалидный магнитометр не отклоняет образец: этот канал пока необязателен.
bool validateSensorSample(const ImuSample& sample, const ImuConfig& config);

// Проверяет, что получен образец с валидными каналами и его возраст не превышает maximumAgeUs.
// nowUs передаётся явно (обычно из micros()), чтобы проверка не зависела от глобального таймера.
// maximumAgeUs должен быть меньше 2^31 для однозначной арифметики времени uint32_t с rollover.
bool isSampleFresh(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs);

// Получает один новый образец от драйвера, проверяет его и сохраняет как последний валидный.
// Возвращает false без замены сохранённого образца, если новых данных нет, чтение ошибочно
// или измерения не проходят validateSensorSample().
bool updateSensorData(const ImuConfig& config);

// Возвращает последний сохранённый образец, только если он ещё свежий.
// nowUs и maximumAgeUs задаются вызывающим кодом; при false sample не меняется.
bool getLatestSample(ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs);

// Возвращает состояние драйвера и кэша без чтения SPI и изменения сохранённого образца.
SensorHealth getSensorHealth(uint32_t nowUs, uint32_t maximumAgeUs);

} // namespace gyrostab
