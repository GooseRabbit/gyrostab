#include "../src/geometry/GyroRateProjection.h"

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

    const float tolerance = 0.00001f;
    const AxisConfig axisX = {{1.0f, 0.0f, 0.0f}};
    const AxisConfig axisY = {{0.0f, 1.0f, 0.0f}};
    const AxisConfig axisZ = {{0.0f, 0.0f, 1.0f}};
    const Vector3f zeroBias = {0.0f, 0.0f, 0.0f};
    float projectedRate = 99.0f;

    // Нулевая скорость и положительное/отрицательное вращение по X, Y и Z.
    assert(projectGyroRateOnControlAxis({0.0f, 0.0f, 0.0f}, zeroBias,
                                        axisX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 0.0f));
    assert(projectGyroRateOnControlAxis({1.25f, 0.0f, 0.0f}, zeroBias,
                                        axisX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 1.25f));
    assert(projectGyroRateOnControlAxis({-1.25f, 0.0f, 0.0f}, zeroBias,
                                        axisX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, -1.25f));

    assert(projectGyroRateOnControlAxis({0.0f, 2.5f, 0.0f}, zeroBias,
                                        axisY, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 2.5f));
    assert(projectGyroRateOnControlAxis({0.0f, -2.5f, 0.0f}, zeroBias,
                                        axisY, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, -2.5f));

    assert(projectGyroRateOnControlAxis({0.0f, 0.0f, 3.75f}, zeroBias,
                                        axisZ, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 3.75f));
    assert(projectGyroRateOnControlAxis({0.0f, 0.0f, -3.75f}, zeroBias,
                                        axisZ, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, -3.75f));

    // Компоненты поперёк выбранной оси не влияют на результат.
    assert(projectGyroRateOnControlAxis({0.0f, 4.0f, -2.0f}, zeroBias,
                                        axisX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 0.0f));

    // Bias вычитается отдельно из каждой компоненты до проекции.
    const Vector3f measuredRate = {1.5f, -2.0f, 0.5f};
    const Vector3f gyroBias = {0.5f, -1.0f, 0.5f};
    assert(projectGyroRateOnControlAxis(measuredRate, gyroBias,
                                        axisX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, 1.0f));
    assert(projectGyroRateOnControlAxis(measuredRate, gyroBias,
                                        axisY, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, -1.0f));

    // Произвольная нормированная ось поддерживается без предположения X/Y/Z.
    const float inverseRootThree = static_cast<float>(1.0 / std::sqrt(3.0));
    const AxisConfig diagonalAxis = {{inverseRootThree, inverseRootThree,
                                      inverseRootThree}};
    assert(projectGyroRateOnControlAxis({1.0f, 2.0f, 3.0f}, zeroBias,
                                        diagonalAxis, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, static_cast<float>(6.0 / std::sqrt(3.0))));

    // Разворот направления оси разворачивает знак проекции.
    const AxisConfig negativeX = {{-1.0f, 0.0f, 0.0f}};
    assert(projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                        negativeX, tolerance, projectedRate));
    assert(nearlyEqual(projectedRate, -1.0f));

    const float savedRate = projectedRate;
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!projectGyroRateOnControlAxis({nanValue, 0.0f, 0.0f}, zeroBias,
                                         axisX, tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({infinity, 0.0f, 0.0f}, zeroBias,
                                         axisX, tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f},
                                         {0.0f, nanValue, 0.0f},
                                         axisX, tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f},
                                         {0.0f, 0.0f, infinity},
                                         axisX, tolerance, projectedRate));
    assert(projectedRate == savedRate);

    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         {{0.0f, 0.0f, 0.0f}},
                                         tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         {{1.1f, 0.0f, 0.0f}},
                                         tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         {{1.0f, infinity, 0.0f}},
                                         tolerance, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         {{1.0f, nanValue, 0.0f}},
                                         tolerance, projectedRate));
    assert(projectedRate == savedRate);

    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         axisX, nanValue, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         axisX, infinity, projectedRate));
    assert(projectedRate == savedRate);
    assert(!projectGyroRateOnControlAxis({1.0f, 0.0f, 0.0f}, zeroBias,
                                         axisX, -0.01f, projectedRate));
    assert(projectedRate == savedRate);

    // Отклоняем проекцию, которая не представима типом float.
    const float maximumFloat = std::numeric_limits<float>::max();
    assert(!projectGyroRateOnControlAxis({maximumFloat, maximumFloat, maximumFloat},
                                         zeroBias, diagonalAxis, tolerance,
                                         projectedRate));
    assert(projectedRate == savedRate);

    std::puts("projectGyroRateOnControlAxis tests passed");
    return 0;
}
