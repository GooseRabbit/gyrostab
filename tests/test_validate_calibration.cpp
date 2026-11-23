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
    const Vector3f bias = {0.010f, -0.020f, 0.030f};

    assert(validateCalibration(samples, 4U, criteria, 4500U, bias, 0.004f));

    // Точное совпадение с пределом принимается (граница включительная).
    ImuSample boundarySamples[] = {
        makeSample(1000U, 10U, 0.0f, 0.0f, 0.0f),
        makeSample(2000U, 11U, 0.03125f, 0.0f, 0.0f),
        makeSample(3000U, 12U, 0.0f, 0.0f, 0.0f),
        makeSample(4000U, 13U, 0.0f, 0.0f, 0.0f)
    };
    const Vector3f boundaryBias = {0.0f, 0.0f, 0.0f};
    const float exactDeviation = 0.03125f;
    assert(validateCalibration(boundarySamples, 4U, criteria, 4500U,
                               boundaryBias, exactDeviation));

    // Выброс за пределом разброса отклоняет калибровку.
    boundarySamples[1].angularVelocityRadPerSec.x = 0.09f;
    assert(!validateCalibration(boundarySamples, 4U, criteria, 4500U,
                                boundaryBias, 0.01f));
    boundarySamples[1] = makeSample(2000U, 11U, 0.014f, -0.020f, 0.030f);

    // Bias должен быть конечным, а предел не может быть отрицательным или неконечным.
    Vector3f invalidBias = bias;
    invalidBias.y = std::numeric_limits<float>::quiet_NaN();
    assert(!validateCalibration(samples, 4U, criteria, 4500U, invalidBias, 0.01f));
    assert(!validateCalibration(samples, 4U, criteria, 4500U, bias, -0.01f));
    assert(!validateCalibration(samples, 4U, criteria, 4500U, bias,
                                std::numeric_limits<float>::infinity()));

    // Не проходит недостаточное или устаревшее окно, как и detectStationary().
    assert(!validateCalibration(samples, 2U, criteria, 4500U, bias, 0.01f));
    assert(!validateCalibration(samples, 4U, criteria, 6000U, bias, 0.01f));

    // При движении и невалидных данных проверка также отклоняет результат.
    samples[2].angularVelocityRadPerSec.z = 0.5f;
    assert(!validateCalibration(samples, 4U, criteria, 4500U, bias, 0.01f));
    samples[2] = makeSample(3000U, 3U, 0.008f, -0.022f, 0.032f);
    samples[1].gyroscopeValid = false;
    assert(!validateCalibration(samples, 4U, criteria, 4500U, bias, 0.01f));
    samples[1] = makeSample(2000U, 2U, 0.012f, -0.018f, 0.028f);

    // Функция не меняет входные измерения и bias.
    assert(samples[0].timestampUs == 1000U);
    assert(samples[0].sequence == 1U);
    assert(samples[0].angularVelocityRadPerSec.x == 0.010f);
    assert(bias.x == 0.010f && bias.y == -0.020f && bias.z == 0.030f);

    std::puts("validateCalibration tests passed");
    return 0;
}
