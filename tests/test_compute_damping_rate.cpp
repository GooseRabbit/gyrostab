#include "../src/control/DampingRate.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-5f)
{
    return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
    using namespace gyrostab;

    const AxisConfig axisX = {{1.0f, 0.0f, 0.0f}};
    const AxisConfig axisY = {{0.0f, 1.0f, 0.0f}};
    const Vector3f bias = {0.5f, -0.5f, 1.0f};
    ImuSample sample = {
        {0.0f, 0.0f, 9.81f},
        {2.0f, -3.0f, 4.0f},
        {0.0f, 0.0f, 0.0f},
        1000U,
        7U,
        true,
        true,
        false
    };

    float rateRadPerSec = 77.0f;
    assert(computeDampingRate(sample, bias, axisX, 0.00001f, rateRadPerSec));
    assert(nearlyEqual(rateRadPerSec, 1.5f));

    // The same corrected IMU rate is projected onto a different control axis.
    assert(computeDampingRate(sample, bias, axisY, 0.00001f, rateRadPerSec));
    assert(nearlyEqual(rateRadPerSec, -2.5f));

    // Arbitrary normalized axis is supported; no X/Y/Z assumption is made.
    const float inverseRootThree = static_cast<float>(1.0 / std::sqrt(3.0));
    const AxisConfig diagonal = {{inverseRootThree, inverseRootThree, inverseRootThree}};
    assert(computeDampingRate(sample, bias, diagonal, 0.00001f, rateRadPerSec));
    assert(nearlyEqual(rateRadPerSec, 2.0f * inverseRootThree));

    // Reversing the axis reverses the signed angular rate.
    const AxisConfig reverseX = {{-1.0f, 0.0f, 0.0f}};
    assert(computeDampingRate(sample, bias, reverseX, 0.00001f, rateRadPerSec));
    assert(nearlyEqual(rateRadPerSec, -1.5f));

    // Missing gyro data is rejected without changing the output.
    const float savedRate = rateRadPerSec;
    sample.gyroscopeValid = false;
    assert(!computeDampingRate(sample, bias, axisX, 0.00001f, rateRadPerSec));
    assert(rateRadPerSec == savedRate);
    sample.gyroscopeValid = true;

    // Invalid measurements, bias, axis, and tolerance also preserve the output.
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    Vector3f savedAngularVelocity = sample.angularVelocityRadPerSec;
    sample.angularVelocityRadPerSec.x = nanValue;
    assert(!computeDampingRate(sample, bias, axisX, 0.00001f, rateRadPerSec));
    assert(rateRadPerSec == savedRate);
    sample.angularVelocityRadPerSec = savedAngularVelocity;

    const Vector3f invalidBias = {infinity, 0.0f, 0.0f};
    assert(!computeDampingRate(sample, invalidBias, axisX, 0.00001f, rateRadPerSec));
    assert(rateRadPerSec == savedRate);
    assert(!computeDampingRate(sample, bias, {{0.0f, 0.0f, 0.0f}}, 0.00001f, rateRadPerSec));
    assert(rateRadPerSec == savedRate);
    assert(!computeDampingRate(sample, bias, axisX, nanValue, rateRadPerSec));
    assert(rateRadPerSec == savedRate);
    assert(!computeDampingRate(sample, bias, axisX, -0.01f, rateRadPerSec));
    assert(rateRadPerSec == savedRate);

    // No input sample or bias is modified.
    assert(sample.angularVelocityRadPerSec.x == savedAngularVelocity.x);
    assert(sample.angularVelocityRadPerSec.y == savedAngularVelocity.y);
    assert(sample.angularVelocityRadPerSec.z == savedAngularVelocity.z);
    assert(sample.timestampUs == 1000U && sample.sequence == 7U);
    assert(bias.x == 0.5f && bias.y == -0.5f && bias.z == 1.0f);

    std::puts("computeDampingRate tests passed");
    return 0;
}
