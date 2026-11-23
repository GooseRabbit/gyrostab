#include "../src/geometry/AxisConfig.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using gyrostab::AxisConfig;
using gyrostab::normalizeAxisConfig;

namespace
{

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}

void testNormalizesAxisInSensorFrame()
{
    const AxisConfig input = {{0.0f, 3.0f, 4.0f}};
    AxisConfig output = {{0.0f, 0.0f, 0.0f}};

    assert(normalizeAxisConfig(input, output));
    assert(nearlyEqual(output.axisInSensorFrame.x, 0.0f));
    assert(nearlyEqual(output.axisInSensorFrame.y, 0.6f));
    assert(nearlyEqual(output.axisInSensorFrame.z, 0.8f));
}

void testPreservesPositiveDirection()
{
    const AxisConfig input = {{0.0f, -5.0f, 0.0f}};
    AxisConfig output = {{0.0f, 0.0f, 0.0f}};

    assert(normalizeAxisConfig(input, output));
    assert(nearlyEqual(output.axisInSensorFrame.x, 0.0f));
    assert(nearlyEqual(output.axisInSensorFrame.y, -1.0f));
    assert(nearlyEqual(output.axisInSensorFrame.z, 0.0f));
}

void testRejectsZeroAxisAndPreservesOutput()
{
    const AxisConfig input = {{0.0f, 0.0f, 0.0f}};
    AxisConfig output = {{7.0f, 8.0f, 9.0f}};

    assert(!normalizeAxisConfig(input, output));
    assert(output.axisInSensorFrame.x == 7.0f);
    assert(output.axisInSensorFrame.y == 8.0f);
    assert(output.axisInSensorFrame.z == 9.0f);
}

void testRejectsNonFiniteAxisAndPreservesOutput()
{
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const AxisConfig invalid[] = {
        {{nanValue, 1.0f, 0.0f}},
        {{0.0f, infinity, 1.0f}},
        {{0.0f, 1.0f, -infinity}}
    };

    for (const AxisConfig& input : invalid)
    {
        AxisConfig output = {{7.0f, 8.0f, 9.0f}};
        assert(!normalizeAxisConfig(input, output));
        assert(output.axisInSensorFrame.x == 7.0f);
        assert(output.axisInSensorFrame.y == 8.0f);
        assert(output.axisInSensorFrame.z == 9.0f);
    }
}

void testSupportsInPlaceNormalization()
{
    AxisConfig value = {{2.0f, 0.0f, 0.0f}};

    assert(normalizeAxisConfig(value, value));
    assert(nearlyEqual(value.axisInSensorFrame.x, 1.0f));
    assert(nearlyEqual(value.axisInSensorFrame.y, 0.0f));
    assert(nearlyEqual(value.axisInSensorFrame.z, 0.0f));
}

} // namespace

int main()
{
    testNormalizesAxisInSensorFrame();
    testPreservesPositiveDirection();
    testRejectsZeroAxisAndPreservesOutput();
    testRejectsNonFiniteAxisAndPreservesOutput();
    testSupportsInPlaceNormalization();

    std::cout << "All AxisConfig tests passed.\n";
    return 0;
}
