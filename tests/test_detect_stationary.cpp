#include "../src/calibration/CalibrationManager.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

using gyrostab::ImuSample;
using gyrostab::StationaryCriteria;

ImuSample makeSample(uint32_t timestampUs, uint32_t sequence)
{
    ImuSample sample = {};
    sample.accelerationMps2 = {0.0f, 0.0f, 9.80665f};
    sample.angularVelocityRadPerSec = {0.0f, 0.0f, 0.0f};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = false;
    return sample;
}

StationaryCriteria makeCriteria()
{
    StationaryCriteria criteria = {};
    criteria.maximumGyroscopeMagnitudeRadPerSec = 0.02f;
    criteria.maximumAccelerationMagnitudeDeviationMps2 = 0.2f;
    criteria.maximumAccelerationVectorVariationMps2 = 0.2f;
    criteria.minimumWindowDurationUs = 2000U;
    criteria.maximumSampleGapUs = 2000U;
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
        makeSample(1000U, 1U),
        makeSample(2000U, 2U),
        makeSample(3000U, 3U),
        makeSample(4000U, 4U)
    };
    StationaryCriteria criteria = makeCriteria();

    assert(detectStationary(samples, 4U, criteria, 4500U));

    // Отсутствующий буфер и недостаточное число измерений отклоняются.
    assert(!detectStationary(nullptr, 4U, criteria, 4500U));
    assert(!detectStationary(samples, 2U, criteria, 4500U));

    // Не хватает длительности окна или допустимый разрыв между образцами превышен.
    criteria.minimumWindowDurationUs = 4000U;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    criteria = makeCriteria();
    criteria.maximumSampleGapUs = 500U;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    criteria = makeCriteria();

    // Движение по гироскопу или ускорению отклоняет окно.
    samples[2].angularVelocityRadPerSec.z = 0.1f;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[2] = makeSample(3000U, 3U);
    samples[2].accelerationMps2.z = 10.5f;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[2] = makeSample(3000U, 3U);

    // Изменение направления вектора ускорения выявляет движение/вибрацию.
    samples[1].accelerationMps2 = {1.0f, 0.0f, 9.7555f};
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[1] = makeSample(2000U, 2U);

    // Невалидные каналы, NaN, устаревшие данные и повторные образцы отклоняются.
    samples[0].gyroscopeValid = false;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[0] = makeSample(1000U, 1U);
    samples[1].angularVelocityRadPerSec.x = std::numeric_limits<float>::quiet_NaN();
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[1] = makeSample(2000U, 2U);
    assert(!detectStationary(samples, 4U, criteria, 6000U));
    samples[2].sequence = samples[1].sequence;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    samples[2] = makeSample(3000U, 2U);
    assert(!detectStationary(samples, 4U, criteria, 4500U)); // sequence пошёл назад
    samples[2] = makeSample(3000U, 0x80000002UL);
    assert(!detectStationary(samples, 4U, criteria, 4500U)); // направление неоднозначно
    samples[2] = makeSample(3000U, 3U);
    samples[2].timestampUs = samples[1].timestampUs;
    assert(!detectStationary(samples, 4U, criteria, 4500U));

    // Параметры критериев не должны содержать NaN/Inf или отрицательные пороги.
    samples[2] = makeSample(3000U, 3U);
    criteria.maximumGyroscopeMagnitudeRadPerSec = -0.1f;
    assert(!detectStationary(samples, 4U, criteria, 4500U));
    criteria = makeCriteria();
    criteria.maximumAccelerationVectorVariationMps2 =
        std::numeric_limits<float>::infinity();
    assert(!detectStationary(samples, 4U, criteria, 4500U));

    // Окно корректно проходит через переполнение micros().
    ImuSample rolloverSamples[] = {
        makeSample(UINT32_MAX - 2999U, 10U),
        makeSample(UINT32_MAX - 1999U, 11U),
        makeSample(UINT32_MAX - 999U, 12U),
        makeSample(0U, 13U)
    };
    criteria = makeCriteria();
    assert(detectStationary(rolloverSamples, 4U, criteria, 500U));

    // Счётчик sequence также может корректно переполниться.
    ImuSample sequenceRolloverSamples[] = {
        makeSample(1000U, UINT32_MAX - 2U),
        makeSample(2000U, UINT32_MAX - 1U),
        makeSample(3000U, UINT32_MAX),
        makeSample(4000U, 0U)
    };
    assert(detectStationary(sequenceRolloverSamples, 4U, criteria, 4500U));

    std::puts("detectStationary tests passed");
    return 0;
}
