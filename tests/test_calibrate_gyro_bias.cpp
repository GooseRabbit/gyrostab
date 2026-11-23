#include "../src/calibration/CalibrationManager.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::ImuSample makeSample(uint32_t timestampUs, uint32_t sequence,
                               float gyroX, float gyroY, float gyroZ)
{
    gyrostab::ImuSample sample = {};
    sample.accelerationMps2 = {0.0f, 0.0f, 9.80665f};
    sample.angularVelocityRadPerSec = {gyroX, gyroY, gyroZ};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = false;
    return sample;
}

gyrostab::StationaryCriteria makeCriteria()
{
    gyrostab::StationaryCriteria criteria = {};
    criteria.maximumGyroscopeMagnitudeRadPerSec = 0.1f;
    criteria.maximumAccelerationMagnitudeDeviationMps2 = 0.1f;
    criteria.maximumAccelerationVectorVariationMps2 = 0.1f;
    criteria.minimumWindowDurationUs = 2000U;
    criteria.maximumSampleGapUs = 1000U;
    criteria.maximumSampleAgeUs = 1000U;
    criteria.minimumSampleCount = 3U;
    return criteria;
}

void assertBiasUnchanged(const gyrostab::Vector3f& bias)
{
    assert(bias.x == 7.0f);
    assert(bias.y == 8.0f);
    assert(bias.z == 9.0f);
}

} // namespace

namespace gyrostab
{

bool isSampleFresh(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs)
{
    if (!sample.accelerometerValid || !sample.gyroscopeValid || maximumAgeUs >= 0x80000000UL)
    {
        return false;
    }
    return (nowUs - sample.timestampUs) <= maximumAgeUs;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;

    ImuSample samples[] = {
        makeSample(1000U, 1U, 0.010f, -0.020f, 0.030f),
        makeSample(2000U, 2U, 0.012f, -0.018f, 0.028f),
        makeSample(3000U, 3U, 0.008f, -0.022f, 0.032f),
        makeSample(4000U, 4U, 0.010f, -0.020f, 0.030f)
    };
    const StationaryCriteria criteria = makeCriteria();

    Vector3f bias = {7.0f, 8.0f, 9.0f};
    assert(calibrateGyroBias(samples, 4U, criteria, 4500U, bias));
    assert(std::fabs(bias.x - 0.010f) < 0.000001f);
    assert(std::fabs(bias.y + 0.020f) < 0.000001f);
    assert(std::fabs(bias.z - 0.030f) < 0.000001f);

    // При движении калибровка отклоняется, а выход остаётся прежним.
    bias = {7.0f, 8.0f, 9.0f};
    samples[2].angularVelocityRadPerSec.z = 0.5f;
    assert(!calibrateGyroBias(samples, 4U, criteria, 4500U, bias));
    assertBiasUnchanged(bias);
    samples[2] = makeSample(3000U, 3U, 0.008f, -0.022f, 0.032f);

    // Недостаточное окно и устаревший последний образец не дают новой калибровки.
    assert(!calibrateGyroBias(samples, 2U, criteria, 4500U, bias));
    assertBiasUnchanged(bias);
    assert(!calibrateGyroBias(samples, 4U, criteria, 6000U, bias));
    assertBiasUnchanged(bias);

    // Невалидные значения/каналы блокируются проверкой неподвижности.
    samples[1].angularVelocityRadPerSec.x = std::numeric_limits<float>::quiet_NaN();
    assert(!calibrateGyroBias(samples, 4U, criteria, 4500U, bias));
    assertBiasUnchanged(bias);

    // Входные измерения остаются неизменными.
    assert(samples[0].timestampUs == 1000U);
    assert(samples[0].sequence == 1U);

    std::puts("calibrateGyroBias tests passed");
    return 0;
}
