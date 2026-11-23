#include "../src/geometry/Vector3.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using gyrostab::Vector3f;
using gyrostab::normalizeVector3;

namespace
{

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}

void testNormalizesAxisAlignedVector()
{
    Vector3f output = {0.0f, 0.0f, 0.0f};
    const Vector3f input = {0.0f, -4.0f, 0.0f};

    assert(normalizeVector3(input, output));
    assert(nearlyEqual(output.x, 0.0f));
    assert(nearlyEqual(output.y, -1.0f));
    assert(nearlyEqual(output.z, 0.0f));
}

void testNormalizesArbitraryVector()
{
    Vector3f output = {0.0f, 0.0f, 0.0f};
    const Vector3f input = {2.0f, 3.0f, 6.0f};

    assert(normalizeVector3(input, output));
    assert(nearlyEqual(output.x, 2.0f / 7.0f));
    assert(nearlyEqual(output.y, 3.0f / 7.0f));
    assert(nearlyEqual(output.z, 6.0f / 7.0f));
    assert(nearlyEqual(std::sqrt(output.x * output.x + output.y * output.y + output.z * output.z), 1.0f));
}

void testRejectsZeroVectorAndPreservesOutput()
{
    Vector3f output = {7.0f, 8.0f, 9.0f};
    const Vector3f input = {0.0f, 0.0f, 0.0f};

    assert(!normalizeVector3(input, output));
    assert(output.x == 7.0f && output.y == 8.0f && output.z == 9.0f);
}

void testRejectsNonFiniteComponentsAndPreservesOutput()
{
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const Vector3f invalid[] = {
        {nanValue, 1.0f, 2.0f},
        {1.0f, infinity, 2.0f},
        {1.0f, 2.0f, -infinity}
    };

    for (const Vector3f& input : invalid)
    {
        Vector3f output = {7.0f, 8.0f, 9.0f};
        assert(!normalizeVector3(input, output));
        assert(output.x == 7.0f && output.y == 8.0f && output.z == 9.0f);
    }
}

void testHandlesVeryLargeAndVerySmallValues()
{
    Vector3f output = {0.0f, 0.0f, 0.0f};
    const float large = std::numeric_limits<float>::max();
    const Vector3f largeInput = {large, large, 0.0f};
    assert(normalizeVector3(largeInput, output));
    assert(nearlyEqual(output.x, 0.70710678f));
    assert(nearlyEqual(output.y, 0.70710678f));
    assert(nearlyEqual(output.z, 0.0f));

    const float small = std::numeric_limits<float>::min();
    const Vector3f smallInput = {small, 0.0f, small};
    assert(normalizeVector3(smallInput, output));
    assert(nearlyEqual(output.x, 0.70710678f));
    assert(nearlyEqual(output.y, 0.0f));
    assert(nearlyEqual(output.z, 0.70710678f));
}

void testSupportsInPlaceNormalization()
{
    Vector3f value = {0.0f, 3.0f, 4.0f};
    assert(normalizeVector3(value, value));
    assert(nearlyEqual(value.x, 0.0f));
    assert(nearlyEqual(value.y, 0.6f));
    assert(nearlyEqual(value.z, 0.8f));
}

} // namespace

int main()
{
    testNormalizesAxisAlignedVector();
    testNormalizesArbitraryVector();
    testRejectsZeroVectorAndPreservesOutput();
    testRejectsNonFiniteComponentsAndPreservesOutput();
    testHandlesVeryLargeAndVerySmallValues();
    testSupportsInPlaceNormalization();

    std::cout << "All Vector3 tests passed.\n";
    return 0;
}
