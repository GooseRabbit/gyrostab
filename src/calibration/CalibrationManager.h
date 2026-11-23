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

} // namespace gyrostab
