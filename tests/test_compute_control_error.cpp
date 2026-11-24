#include "../src/control/ControlError.h"

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
    const float half = static_cast<float>(std::sqrt(0.5));
    const float pi = 3.14159265358979323846f;
    const float tolerance = 0.00001f;
    const AxisConfig axisX = {{1.0f, 0.0f, 0.0f}};
    const ReferenceOrientation identityReference = {{1.0f, 0.0f, 0.0f, 0.0f}, true};

    float errorRad = 77.0f;
    OrientationEstimate current = {{1.0f, 0.0f, 0.0f, 0.0f}, 10U, 1U, true};
    assert(computeControlError(current, identityReference, axisX, tolerance, errorRad));
    assert(nearlyEqual(errorRad, 0.0f));

    // Относительный поворот +90 градусов вокруг X возвращает +pi/2.
    current.quaternion = {half, half, 0.0f, 0.0f};
    assert(computeControlError(current, identityReference, axisX, tolerance, errorRad));
    assert(nearlyEqual(errorRad, pi / 2.0f));

    // Неперестановочные повороты фиксируют порядок inverse(reference) * current.
    const ReferenceOrientation referenceX = {{half, half, 0.0f, 0.0f}, true};
    current.quaternion = {half, 0.0f, half, 0.0f};
    assert(computeControlError(current, referenceX, axisX, tolerance, errorRad));
    assert(nearlyEqual(errorRad, -pi / 2.0f));

    // При неверном current выходное значение остаётся прежним.
    const float savedError = errorRad;
    current.valid = false;
    assert(!computeControlError(current, referenceX, axisX, tolerance, errorRad));
    assert(errorRad == savedError);
    current.valid = true;
    current.quaternion = {1.1f, 0.0f, 0.0f, 0.0f};
    assert(!computeControlError(current, identityReference, axisX, tolerance, errorRad));
    assert(errorRad == savedError);

    // Невалидная опора, ось или допуск не меняют выход.
    current.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    const ReferenceOrientation invalidReference = {{1.0f, 0.0f, 0.0f, 0.0f}, false};
    assert(!computeControlError(current, invalidReference, axisX, tolerance, errorRad));
    assert(errorRad == savedError);
    assert(!computeControlError(current, identityReference,
                                {{0.0f, 0.0f, 0.0f}}, tolerance, errorRad));
    assert(errorRad == savedError);
    assert(!computeControlError(current, identityReference,
                                {{1.1f, 0.0f, 0.0f}}, tolerance, errorRad));
    assert(errorRad == savedError);
    assert(!computeControlError(current, identityReference, axisX,
                                std::numeric_limits<float>::quiet_NaN(), errorRad));
    assert(errorRad == savedError);
    assert(!computeControlError(current, identityReference, axisX,
                                -0.01f, errorRad));
    assert(errorRad == savedError);

    // Неоднозначный 180-градусный swing также сохраняет старую ошибку.
    const ReferenceOrientation identity = {{1.0f, 0.0f, 0.0f, 0.0f}, true};
    current.quaternion = {0.0f, 0.0f, 1.0f, 0.0f};
    assert(!computeControlError(current, identity, axisX, tolerance, errorRad));
    assert(errorRad == savedError);

    // Метаданные и входные кватернионы не изменяются функцией.
    assert(current.quaternion.w == 0.0f && current.quaternion.y == 1.0f);
    assert(current.timestampUs == 10U && current.sequence == 1U);
    assert(referenceX.quaternion.w == half && referenceX.quaternion.x == half);

    std::puts("computeControlError tests passed");
    return 0;
}
