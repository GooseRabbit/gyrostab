#include "../src/geometry/ControlAxisProjection.h"

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

    const double pi = 3.14159265358979323846;
    const float half = static_cast<float>(std::sqrt(0.5));
    const AxisConfig axisX = {{1.0f, 0.0f, 0.0f}};
    const AxisConfig axisY = {{0.0f, 1.0f, 0.0f}};
    const AxisConfig axisZ = {{0.0f, 0.0f, 1.0f}};
    const float normTolerance = 0.00001f;

    float errorRad = 99.0f;
    assert(projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, 0.0f));

    // Positive and negative rotations around each supported axis.
    assert(projectErrorOnControlAxis({half, half, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 2.0)));
    assert(projectErrorOnControlAxis({half, -half, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, -static_cast<float>(pi / 2.0)));

    assert(projectErrorOnControlAxis({half, 0.0f, half, 0.0f},
                                     axisY, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 2.0)));
    assert(projectErrorOnControlAxis({half, 0.0f, -half, 0.0f},
                                     axisY, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, -static_cast<float>(pi / 2.0)));

    assert(projectErrorOnControlAxis({half, 0.0f, 0.0f, half},
                                     axisZ, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 2.0)));
    assert(projectErrorOnControlAxis({half, 0.0f, 0.0f, -half},
                                     axisZ, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, -static_cast<float>(pi / 2.0)));

    // A swing perpendicular to X does not create an X-axis twist error.
    assert(projectErrorOnControlAxis({half, 0.0f, half, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, 0.0f));

    // Combining a Y swing and X twist must preserve the X twist component.
    const float cosQuarter = static_cast<float>(std::cos(pi / 8.0));
    const float sinQuarter = static_cast<float>(std::sin(pi / 8.0));
    const Quaternionf swingThenTwist = {
        cosQuarter * cosQuarter,
        cosQuarter * sinQuarter,
        sinQuarter * cosQuarter,
        -sinQuarter * sinQuarter
    };
    assert(projectErrorOnControlAxis(swingThenTwist, axisX,
                                     normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 4.0)));

    // q and -q are equivalent, including the exactly-180-degree boundary.
    assert(projectErrorOnControlAxis({-half, -half, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 2.0)));
    assert(projectErrorOnControlAxis({0.0f, 1.0f, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi)));
    assert(projectErrorOnControlAxis({0.0f, -1.0f, 0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi)));

    // A 270-degree rotation is represented by its shortest signed angle (-90 degrees).
    const float cosThreeQuarterPi = static_cast<float>(std::cos(3.0 * pi / 4.0));
    const float sinThreeQuarterPi = static_cast<float>(std::sin(3.0 * pi / 4.0));
    assert(projectErrorOnControlAxis({cosThreeQuarterPi, sinThreeQuarterPi,
                                      0.0f, 0.0f},
                                     axisX, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, -static_cast<float>(pi / 2.0)));

    // An arbitrary normalized axis is supported; no X/Y/Z assumption is made.
    const float inverseRootThree = static_cast<float>(1.0 / std::sqrt(3.0));
    const AxisConfig diagonalAxis = {{inverseRootThree, inverseRootThree,
                                      inverseRootThree}};
    assert(projectErrorOnControlAxis({half, half * inverseRootThree,
                                      half * inverseRootThree,
                                      half * inverseRootThree},
                                     diagonalAxis, normTolerance, errorRad));
    assert(nearlyEqual(errorRad, static_cast<float>(pi / 2.0)));

    // A 180-degree swing with no component around X has no defined X twist.
    const float savedError = errorRad;
    assert(!projectErrorOnControlAxis({0.0f, 0.0f, 1.0f, 0.0f},
                                      axisX, normTolerance, errorRad));
    assert(errorRad == savedError);

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!projectErrorOnControlAxis({nanValue, 0.0f, 0.0f, 0.0f},
                                      axisX, 0.01f, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({infinity, 0.0f, 0.0f, 0.0f},
                                      axisX, 0.01f, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.1f, 0.0f, 0.0f, 0.0f},
                                      axisX, 0.01f, errorRad));
    assert(errorRad == savedError);

    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      {{0.0f, 0.0f, 0.0f}}, 0.01f, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      {{1.1f, 0.0f, 0.0f}}, 0.01f, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      {{1.0f, infinity, 0.0f}}, 0.01f, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      {{1.0f, nanValue, 0.0f}}, 0.01f, errorRad));
    assert(errorRad == savedError);

    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      axisX, nanValue, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      axisX, infinity, errorRad));
    assert(errorRad == savedError);
    assert(!projectErrorOnControlAxis({1.0f, 0.0f, 0.0f, 0.0f},
                                      axisX, -0.01f, errorRad));
    assert(errorRad == savedError);

    std::puts("projectErrorOnControlAxis tests passed");
    return 0;
}
