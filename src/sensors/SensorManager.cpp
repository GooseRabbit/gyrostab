#include "SensorManager.h"

#include <cmath>

namespace gyrostab
{
namespace
{

// Кэш последнего успешно проверенного образца; его читает getLatestSample().
ImuSample latestSample = {};
bool hasLatestSample = false;

} // namespace

bool validateSensorSample(const ImuSample& sample, const ImuConfig& config)
{
    if (!sample.accelerometerValid || !sample.gyroscopeValid)
    {
        return false;
    }

    // Коэффициенты преобразования взяты из SparkFun ICM-20948 Library 1.3.2:
    // getAccMG() и getGyrDPS() в ICM_20948.cpp.
    constexpr float kAccelerometerCountsPerG[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    constexpr float kGyroscopeCountsPerDegreePerSecond[] = {131.0f, 65.5f, 32.8f, 16.4f};
    constexpr float kRawValueMagnitude = 32768.0f;
    constexpr float kStandardGravity = 9.80665f;
    constexpr float kDegreesToRadians = 0.01745329252f;
    constexpr float kMagnetometerMicroTeslaPerCount = 0.15f;

    const uint8_t accelerometerRange = config.fullScale.a;
    const uint8_t gyroscopeRange = config.fullScale.g;
    if (accelerometerRange >= 4U || gyroscopeRange >= 4U)
    {
        return false;
    }

    // Предельные значения выводятся из выбранного full-scale и диапазона int16_t,
    // а не задаются произвольными порогами для предполагаемого режима движения.
    const float maximumAcceleration =
        (kRawValueMagnitude / kAccelerometerCountsPerG[accelerometerRange]) * kStandardGravity;
    const float maximumAngularVelocity =
        (kRawValueMagnitude / kGyroscopeCountsPerDegreePerSecond[gyroscopeRange]) * kDegreesToRadians;

    if (!std::isfinite(sample.accelerationMps2.x) ||
        !std::isfinite(sample.accelerationMps2.y) ||
        !std::isfinite(sample.accelerationMps2.z) ||
        std::fabs(sample.accelerationMps2.x) > maximumAcceleration ||
        std::fabs(sample.accelerationMps2.y) > maximumAcceleration ||
        std::fabs(sample.accelerationMps2.z) > maximumAcceleration)
    {
        return false;
    }

    if (!std::isfinite(sample.angularVelocityRadPerSec.x) ||
        !std::isfinite(sample.angularVelocityRadPerSec.y) ||
        !std::isfinite(sample.angularVelocityRadPerSec.z) ||
        std::fabs(sample.angularVelocityRadPerSec.x) > maximumAngularVelocity ||
        std::fabs(sample.angularVelocityRadPerSec.y) > maximumAngularVelocity ||
        std::fabs(sample.angularVelocityRadPerSec.z) > maximumAngularVelocity)
    {
        return false;
    }

    if (sample.magnetometerValid)
    {
        const float maximumMagneticField = kRawValueMagnitude * kMagnetometerMicroTeslaPerCount;
        if (!std::isfinite(sample.magneticFieldMicroTesla.x) ||
            !std::isfinite(sample.magneticFieldMicroTesla.y) ||
            !std::isfinite(sample.magneticFieldMicroTesla.z) ||
            std::fabs(sample.magneticFieldMicroTesla.x) > maximumMagneticField ||
            std::fabs(sample.magneticFieldMicroTesla.y) > maximumMagneticField ||
            std::fabs(sample.magneticFieldMicroTesla.z) > maximumMagneticField)
        {
            return false;
        }
    }

    return true;
}

bool isSampleFresh(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs)
{
    // Отсутствие обязательных каналов означает, что опубликованного образца ещё нет.
    if (!sample.accelerometerValid || !sample.gyroscopeValid)
    {
        return false;
    }

    // При интервале в половину диапазона uint32_t и более порядок моментов неоднозначен.
    if (maximumAgeUs >= 0x80000000UL)
    {
        return false;
    }

    // Беззнаковое вычитание корректно обрабатывает rollover счётчика micros().
    const uint32_t ageUs = nowUs - sample.timestampUs;
    return ageUs <= maximumAgeUs;
}

bool updateSensorData(const ImuConfig& config)
{
    ImuSample candidate = {};
    if (!readImuSample(candidate))
    {
        return false;
    }

    if (!validateSensorSample(candidate, config))
    {
        return false;
    }

    latestSample = candidate;
    hasLatestSample = true;
    return true;
}

bool getLatestSample(ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs)
{
    if (!hasLatestSample || !isSampleFresh(latestSample, nowUs, maximumAgeUs))
    {
        return false;
    }

    sample = latestSample;
    return true;
}

} // namespace gyrostab
