#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::ImuSample makeSample(uint32_t timestampUs)
{
    gyrostab::ImuSample sample = {};
    sample.timestampUs = timestampUs;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    return sample;
}

void testChecksFreshnessBoundary()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(gyrostab::isSampleFresh(sample, 150U, 50U));
    assert(!gyrostab::isSampleFresh(sample, 151U, 50U));
}

void testAllowsZeroAgeLimitOnlyAtSameTick()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(gyrostab::isSampleFresh(sample, 100U, 0U));
    assert(!gyrostab::isSampleFresh(sample, 101U, 0U));
}

void testHandlesMicrosRollover()
{
    const gyrostab::ImuSample sample = makeSample(UINT32_MAX - 4U);
    assert(gyrostab::isSampleFresh(sample, 5U, 10U));
    assert(!gyrostab::isSampleFresh(sample, 5U, 9U));
}

void testRejectsFutureTimestamp()
{
    const gyrostab::ImuSample sample = makeSample(101U);
    assert(!gyrostab::isSampleFresh(sample, 100U, 1000U));
}

void testRejectsUnpublishedSample()
{
    gyrostab::ImuSample sample = {};
    assert(!gyrostab::isSampleFresh(sample, 1U, 100U));

    sample = makeSample(1U);
    sample.gyroscopeValid = false;
    assert(!gyrostab::isSampleFresh(sample, 1U, 100U));
}

void testRejectsAmbiguousMaximumAge()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(!gyrostab::isSampleFresh(sample, 100U, 0x80000000UL));
    assert(!gyrostab::isSampleFresh(sample, 100U, UINT32_MAX));
}

} // namespace

int main()
{
    testChecksFreshnessBoundary();
    testAllowsZeroAgeLimitOnlyAtSameTick();
    testHandlesMicrosRollover();
    testRejectsFutureTimestamp();
    testRejectsUnpublishedSample();
    testRejectsAmbiguousMaximumAge();

    std::puts("isSampleFresh tests passed");
    return 0;
}
