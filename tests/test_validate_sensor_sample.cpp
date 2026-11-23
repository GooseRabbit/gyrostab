#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::ImuSample makeValidSample()
{
    gyrostab::ImuSample sample = {};
    sample.accelerationMps2 = {0.0f, 0.0f, 9.80665f};
    sample.angularVelocityRadPerSec = {0.0f, 0.0f, 0.0f};
    sample.magneticFieldMicroTesla = {25.0f, -10.0f, 40.0f};
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = true;
    return sample;
}

gyrostab::ImuConfig makeDefaultScale()
{
    gyrostab::ImuConfig config = {};
    config.fullScale.a = 0;
    config.fullScale.g = 0;
    return config;
}

void testAcceptsValidSample()
{
    const gyrostab::ImuSample sample = makeValidSample();
    const gyrostab::ImuConfig config = makeDefaultScale();
    assert(gyrostab::validateSensorSample(sample, config));
}

void testRequiresAccelerometerAndGyroscope()
{
    gyrostab::ImuSample sample = makeValidSample();
    const gyrostab::ImuConfig config = makeDefaultScale();

    sample.accelerometerValid = false;
    assert(!gyrostab::validateSensorSample(sample, config));

    sample = makeValidSample();
    sample.gyroscopeValid = false;
    assert(!gyrostab::validateSensorSample(sample, config));
}

void testRejectsNonFiniteRequiredChannels()
{
    gyrostab::ImuSample sample = makeValidSample();
    const gyrostab::ImuConfig config = makeDefaultScale();
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    sample.accelerationMps2.x = nanValue;
    assert(!gyrostab::validateSensorSample(sample, config));

    sample = makeValidSample();
    sample.angularVelocityRadPerSec.y = infinity;
    assert(!gyrostab::validateSensorSample(sample, config));
}

void testUsesConfiguredAccelerometerAndGyroscopeRanges()
{
    gyrostab::ImuSample sample = makeValidSample();
    gyrostab::ImuConfig config = makeDefaultScale();
    sample.accelerationMps2.x = 30.0f;
    sample.angularVelocityRadPerSec.x = 6.0f;

    // При ±2g и ±250°/с оба значения выходят за выбранные диапазоны.
    assert(!gyrostab::validateSensorSample(sample, config));

    // Переключение full-scale на ±4g и ±500°/с делает эти значения допустимыми.
    config.fullScale.a = 1;
    config.fullScale.g = 1;
    assert(gyrostab::validateSensorSample(sample, config));

    sample.accelerationMps2.x = 50.0f;
    assert(!gyrostab::validateSensorSample(sample, config));

    sample = makeValidSample();
    sample.angularVelocityRadPerSec.x = 10.0f;
    assert(!gyrostab::validateSensorSample(sample, config));
}

void testRejectsUnsupportedFullScale()
{
    gyrostab::ImuSample sample = makeValidSample();
    gyrostab::ImuConfig config = makeDefaultScale();

    config.fullScale.a = 4;
    assert(!gyrostab::validateSensorSample(sample, config));

    config = makeDefaultScale();
    config.fullScale.g = 4;
    assert(!gyrostab::validateSensorSample(sample, config));
}

void testAcceptsDigitalFullScaleBoundary()
{
    gyrostab::ImuSample sample = makeValidSample();
    gyrostab::ImuConfig config = makeDefaultScale();
    constexpr float kRawValueMagnitude = 32768.0f;
    constexpr float kStandardGravity = 9.80665f;
    constexpr float kDegreesToRadians = 0.01745329252f;
    constexpr float kMagnetometerMicroTeslaPerCount = 0.15f;
    constexpr float kAccelerometerCountsPerG = 16384.0f;
    constexpr float kGyroscopeCountsPerDegreePerSecond = 131.0f;

    sample.accelerationMps2.x =
        (kRawValueMagnitude / kAccelerometerCountsPerG) * kStandardGravity;
    sample.angularVelocityRadPerSec.x =
        (kRawValueMagnitude / kGyroscopeCountsPerDegreePerSecond) * kDegreesToRadians;
    sample.magneticFieldMicroTesla.x = kRawValueMagnitude * kMagnetometerMicroTeslaPerCount;

    assert(gyrostab::validateSensorSample(sample, config));
}

void testMagnetometerIsOptionalButCheckedWhenValid()
{
    gyrostab::ImuSample sample = makeValidSample();
    const gyrostab::ImuConfig config = makeDefaultScale();
    const float nanValue = std::numeric_limits<float>::quiet_NaN();

    sample.magnetometerValid = false;
    sample.magneticFieldMicroTesla.x = nanValue;
    assert(gyrostab::validateSensorSample(sample, config));

    sample.magnetometerValid = true;
    assert(!gyrostab::validateSensorSample(sample, config));

    sample = makeValidSample();
    sample.magneticFieldMicroTesla.z = 5000.0f;
    assert(!gyrostab::validateSensorSample(sample, config));
}

} // namespace

int main()
{
    testAcceptsValidSample();
    testRequiresAccelerometerAndGyroscope();
    testRejectsNonFiniteRequiredChannels();
    testUsesConfiguredAccelerometerAndGyroscopeRanges();
    testRejectsUnsupportedFullScale();
    testAcceptsDigitalFullScaleBoundary();
    testMagnetometerIsOptionalButCheckedWhenValid();

    std::puts("validateSensorSample tests passed");
    return 0;
}
