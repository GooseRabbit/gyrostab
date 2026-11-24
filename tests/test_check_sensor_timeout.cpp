#include "../src/safety/SafetyManager.h"
#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace gyrostab
{

// These stubs satisfy the SensorManager driver boundary; freshness logic is real.
bool readImuSample(ImuSample&)
{
    return false;
}

ICM_20948_Status_e checkImuStatus()
{
    return ICM_20948_Stat_Unknown;
}

} // namespace gyrostab

namespace
{

gyrostab::ImuSample makeSample(uint32_t timestampUs)
{
    gyrostab::ImuSample sample = {};
    sample.timestampUs = timestampUs;
    sample.sequence = 17U;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = false;
    return sample;
}

void testAcceptsFreshSampleAtAgeBoundary()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(gyrostab::checkSensorTimeout(sample, 150U, 50U));
}

void testRejectsExpiredSample()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(!gyrostab::checkSensorTimeout(sample, 151U, 50U));
}

void testRejectsMissingRequiredChannels()
{
    gyrostab::ImuSample sample = makeSample(100U);
    sample.accelerometerValid = false;
    assert(!gyrostab::checkSensorTimeout(sample, 100U, 50U));

    sample = makeSample(100U);
    sample.gyroscopeValid = false;
    assert(!gyrostab::checkSensorTimeout(sample, 100U, 50U));
}

void testDoesNotRequireMagnetometer()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    assert(!sample.magnetometerValid);
    assert(gyrostab::checkSensorTimeout(sample, 100U, 0U));
}

void testHandlesMicrosRollover()
{
    const gyrostab::ImuSample sample = makeSample(UINT32_MAX - 4U);
    assert(gyrostab::checkSensorTimeout(sample, 5U, 10U));
    assert(!gyrostab::checkSensorTimeout(sample, 5U, 9U));
}

void testRejectsFutureTimestampAndAmbiguousLimit()
{
    const gyrostab::ImuSample futureSample = makeSample(101U);
    assert(!gyrostab::checkSensorTimeout(futureSample, 100U, 1000U));

    const gyrostab::ImuSample sample = makeSample(100U);
    assert(!gyrostab::checkSensorTimeout(sample, 100U, 0x80000000UL));
    assert(!gyrostab::checkSensorTimeout(sample, 100U, UINT32_MAX));
}

void testDoesNotModifySample()
{
    const gyrostab::ImuSample sample = makeSample(100U);
    const uint32_t timestampUs = sample.timestampUs;
    const uint32_t sequence = sample.sequence;
    const bool accelerometerValid = sample.accelerometerValid;
    const bool gyroscopeValid = sample.gyroscopeValid;

    assert(gyrostab::checkSensorTimeout(sample, 110U, 10U));
    assert(sample.timestampUs == timestampUs);
    assert(sample.sequence == sequence);
    assert(sample.accelerometerValid == accelerometerValid);
    assert(sample.gyroscopeValid == gyroscopeValid);
}

} // namespace

int main()
{
    testAcceptsFreshSampleAtAgeBoundary();
    testRejectsExpiredSample();
    testRejectsMissingRequiredChannels();
    testDoesNotRequireMagnetometer();
    testHandlesMicrosRollover();
    testRejectsFutureTimestampAndAmbiguousLimit();
    testDoesNotModifySample();

    std::puts("checkSensorTimeout tests passed");
    return 0;
}
