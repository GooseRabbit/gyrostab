#include "CalibrationManager.h"

#include "../sensors/SensorManager.h"

#include <cmath>

namespace gyrostab
{
namespace
{

constexpr uint32_t kHalfTimestampRange = 0x80000000UL;
constexpr float kStandardGravityMps2 = 9.80665f;

} // namespace

bool detectStationary(const ImuSample* samples, size_t sampleCount,
                      const StationaryCriteria& criteria, uint32_t nowUs)
{
    if (samples == nullptr || criteria.minimumSampleCount < 2U ||
        sampleCount < criteria.minimumSampleCount ||
        criteria.maximumSampleGapUs == 0U ||
        criteria.maximumSampleGapUs >= kHalfTimestampRange ||
        criteria.minimumWindowDurationUs >= kHalfTimestampRange ||
        criteria.maximumSampleAgeUs >= kHalfTimestampRange)
    {
        return false;
    }

    if (!std::isfinite(criteria.maximumGyroscopeMagnitudeRadPerSec) ||
        !std::isfinite(criteria.maximumAccelerationMagnitudeDeviationMps2) ||
        !std::isfinite(criteria.maximumAccelerationVectorVariationMps2) ||
        criteria.maximumGyroscopeMagnitudeRadPerSec < 0.0f ||
        criteria.maximumAccelerationMagnitudeDeviationMps2 < 0.0f ||
        criteria.maximumAccelerationVectorVariationMps2 < 0.0f)
    {
        return false;
    }

    const ImuSample& latestSample = samples[sampleCount - 1U];
    if (!isSampleFresh(latestSample, nowUs, criteria.maximumSampleAgeUs))
    {
        return false;
    }

    double meanAccelerationX = 0.0;
    double meanAccelerationY = 0.0;
    double meanAccelerationZ = 0.0;

    for (size_t i = 0; i < sampleCount; ++i)
    {
        const ImuSample& sample = samples[i];
        if (!sample.accelerometerValid || !sample.gyroscopeValid ||
            !std::isfinite(sample.accelerationMps2.x) ||
            !std::isfinite(sample.accelerationMps2.y) ||
            !std::isfinite(sample.accelerationMps2.z) ||
            !std::isfinite(sample.angularVelocityRadPerSec.x) ||
            !std::isfinite(sample.angularVelocityRadPerSec.y) ||
            !std::isfinite(sample.angularVelocityRadPerSec.z))
        {
            return false;
        }

        const double gyroX = sample.angularVelocityRadPerSec.x;
        const double gyroY = sample.angularVelocityRadPerSec.y;
        const double gyroZ = sample.angularVelocityRadPerSec.z;
        const double gyroMagnitude = std::sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
        if (gyroMagnitude > criteria.maximumGyroscopeMagnitudeRadPerSec)
        {
            return false;
        }

        const double accelerationX = sample.accelerationMps2.x;
        const double accelerationY = sample.accelerationMps2.y;
        const double accelerationZ = sample.accelerationMps2.z;
        const double accelerationMagnitude =
            std::sqrt(accelerationX * accelerationX + accelerationY * accelerationY +
                      accelerationZ * accelerationZ);
        if (std::fabs(accelerationMagnitude - kStandardGravityMps2) >
            criteria.maximumAccelerationMagnitudeDeviationMps2)
        {
            return false;
        }

        meanAccelerationX += (accelerationX - meanAccelerationX) / static_cast<double>(i + 1U);
        meanAccelerationY += (accelerationY - meanAccelerationY) / static_cast<double>(i + 1U);
        meanAccelerationZ += (accelerationZ - meanAccelerationZ) / static_cast<double>(i + 1U);

        if (i > 0U)
        {
            const ImuSample& previousSample = samples[i - 1U];
            const uint32_t sampleGapUs = sample.timestampUs - previousSample.timestampUs;
            const uint32_t sequenceDifference = sample.sequence - previousSample.sequence;
            if (sampleGapUs == 0U || sampleGapUs > criteria.maximumSampleGapUs ||
                sampleGapUs >= kHalfTimestampRange || sequenceDifference != 1U)
            {
                return false;
            }
        }
    }

    const uint32_t windowDurationUs = latestSample.timestampUs - samples[0].timestampUs;
    if (windowDurationUs < criteria.minimumWindowDurationUs ||
        windowDurationUs >= kHalfTimestampRange)
    {
        return false;
    }

    for (size_t i = 0; i < sampleCount; ++i)
    {
        const double deltaX = samples[i].accelerationMps2.x - meanAccelerationX;
        const double deltaY = samples[i].accelerationMps2.y - meanAccelerationY;
        const double deltaZ = samples[i].accelerationMps2.z - meanAccelerationZ;
        const double accelerationVariation =
            std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        if (accelerationVariation > criteria.maximumAccelerationVectorVariationMps2)
        {
            return false;
        }
    }

    return true;
}

bool calibrateGyroBias(const ImuSample* samples, size_t sampleCount,
                       const StationaryCriteria& stationaryCriteria, uint32_t nowUs,
                       Vector3f& gyroBiasRadPerSec)
{
    if (!detectStationary(samples, sampleCount, stationaryCriteria, nowUs))
    {
        return false;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    double meanZ = 0.0;
    for (size_t i = 0; i < sampleCount; ++i)
    {
        meanX += samples[i].angularVelocityRadPerSec.x;
        meanY += samples[i].angularVelocityRadPerSec.y;
        meanZ += samples[i].angularVelocityRadPerSec.z;
    }

    const double sampleCountAsDouble = static_cast<double>(sampleCount);
    const Vector3f candidate = {
        static_cast<float>(meanX / sampleCountAsDouble),
        static_cast<float>(meanY / sampleCountAsDouble),
        static_cast<float>(meanZ / sampleCountAsDouble)
    };

    if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y) || !std::isfinite(candidate.z))
    {
        return false;
    }

    gyroBiasRadPerSec = candidate;
    return true;
}

bool validateCalibration(const ImuSample* samples, size_t sampleCount,
                         const StationaryCriteria& stationaryCriteria, uint32_t nowUs,
                         const Vector3f& gyroBiasRadPerSec,
                         float maximumGyroscopeDeviationFromBiasRadPerSec)
{
    if (!std::isfinite(gyroBiasRadPerSec.x) ||
        !std::isfinite(gyroBiasRadPerSec.y) ||
        !std::isfinite(gyroBiasRadPerSec.z) ||
        !std::isfinite(maximumGyroscopeDeviationFromBiasRadPerSec) ||
        maximumGyroscopeDeviationFromBiasRadPerSec < 0.0f ||
        !detectStationary(samples, sampleCount, stationaryCriteria, nowUs))
    {
        return false;
    }

    for (size_t i = 0; i < sampleCount; ++i)
    {
        const double deltaX =
            static_cast<double>(samples[i].angularVelocityRadPerSec.x) - gyroBiasRadPerSec.x;
        const double deltaY =
            static_cast<double>(samples[i].angularVelocityRadPerSec.y) - gyroBiasRadPerSec.y;
        const double deltaZ =
            static_cast<double>(samples[i].angularVelocityRadPerSec.z) - gyroBiasRadPerSec.z;
        const double deviation = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        if (deviation > maximumGyroscopeDeviationFromBiasRadPerSec)
        {
            return false;
        }
    }

    return true;
}

void resetCalibration(Vector3f& gyroBiasRadPerSec, bool& calibrationValid)
{
    gyroBiasRadPerSec = {0.0f, 0.0f, 0.0f};
    calibrationValid = false;
}

} // namespace gyrostab
