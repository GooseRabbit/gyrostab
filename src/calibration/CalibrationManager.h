#pragma once

#include "../sensors/ImuDriver.h"

#include <stddef.h>
#include <stdint.h>

namespace gyrostab
{

// Пороги и требования к окну измерений задаются вызывающим кодом, без значений по умолчанию.
struct StationaryCriteria
{
    float maximumGyroscopeMagnitudeRadPerSec;
    float maximumAccelerationMagnitudeDeviationMps2;
    float maximumAccelerationVectorVariationMps2;
    uint32_t minimumWindowDurationUs;
    uint32_t maximumSampleGapUs;
    uint32_t maximumSampleAgeUs;
    size_t minimumSampleCount;
};

// Проверяет окно последовательных образцов на неподвижность платформы.
// Магнитометр не используется. Входные образцы не изменяются.
bool detectStationary(const ImuSample* samples, size_t sampleCount,
                      const StationaryCriteria& criteria, uint32_t nowUs);

// Вычисляет gyro bias как среднее значение угловой скорости по неподвижному окну.
// При false выходной вектор остаётся без изменений.
bool calibrateGyroBias(const ImuSample* samples, size_t sampleCount,
                       const StationaryCriteria& stationaryCriteria, uint32_t nowUs,
                       Vector3f& gyroBiasRadPerSec);

// Проверяет качество рассчитанного gyro bias на том же неподвижном окне.
// maximumGyroscopeDeviationFromBiasRadPerSec — допустимое максимальное
// евклидово отклонение отдельного измерения gyro от переданного bias.
bool validateCalibration(const ImuSample* samples, size_t sampleCount,
                         const StationaryCriteria& stationaryCriteria, uint32_t nowUs,
                         const Vector3f& gyroBiasRadPerSec,
                         float maximumGyroscopeDeviationFromBiasRadPerSec);

// Сбрасывает рассчитанный bias и caller-owned признак готовности калибровки.
// Окно измерений не хранится в CalibrationManager и здесь не изменяется.
void resetCalibration(Vector3f& gyroBiasRadPerSec, bool& calibrationValid);

} // namespace gyrostab
