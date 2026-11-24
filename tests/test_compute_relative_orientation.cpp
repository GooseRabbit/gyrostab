#include "../src/geometry/RelativeOrientation.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}

bool sameQuaternion(const gyrostab::Quaternionf& left, const gyrostab::Quaternionf& right)
{
    return left.w == right.w && left.x == right.x &&
           left.y == right.y && left.z == right.z;
}

} // namespace

int main()
{
    using namespace gyrostab;
    const float half = static_cast<float>(std::sqrt(0.5));

    Quaternionf relative = {0.0f, 0.0f, 0.0f, 0.0f};
    const ReferenceOrientation identityReference = {{1.0f, 0.0f, 0.0f, 0.0f}, true};
    OrientationEstimate current = {{1.0f, 0.0f, 0.0f, 0.0f}, 1U, 1U, true};
    assert(computeRelativeOrientation(current, identityReference, 0.0f, relative));
    assert(nearlyEqual(relative.w, 1.0f) && nearlyEqual(relative.x, 0.0f) &&
           nearlyEqual(relative.y, 0.0f) && nearlyEqual(relative.z, 0.0f));

    // Неперпендикулярные повороты фиксируют порядок inverse(reference) * current.
    const ReferenceOrientation referenceX = {{half, half, 0.0f, 0.0f}, true};
    current.quaternion = {half, 0.0f, half, 0.0f};
    assert(computeRelativeOrientation(current, referenceX, 0.00001f, relative));
    assert(nearlyEqual(relative.w, 0.5f));
    assert(nearlyEqual(relative.x, -0.5f));
    assert(nearlyEqual(relative.y, 0.5f));
    assert(nearlyEqual(relative.z, -0.5f));

    const float norm = std::sqrt(relative.w * relative.w + relative.x * relative.x +
                                 relative.y * relative.y + relative.z * relative.z);
    assert(nearlyEqual(norm, 1.0f));

    // Одинаковые current и reference дают нейтральное относительное положение.
    const ReferenceOrientation sameReference = {current.quaternion, true};
    assert(computeRelativeOrientation(current, sameReference, 0.00001f, relative));
    assert(nearlyEqual(relative.w, 1.0f));
    assert(nearlyEqual(relative.x, 0.0f));
    assert(nearlyEqual(relative.y, 0.0f));
    assert(nearlyEqual(relative.z, 0.0f));

    const Quaternionf saved = {7.0f, 8.0f, 9.0f, 10.0f};
    relative = saved;
    const Quaternionf validCurrent = current.quaternion;
    const ReferenceOrientation invalidReference = {{1.0f, 0.0f, 0.0f, 0.0f}, false};
    assert(!computeRelativeOrientation(current, invalidReference, 0.01f, relative));
    assert(sameQuaternion(relative, saved));

    current.valid = false;
    assert(!computeRelativeOrientation(current, referenceX, 0.01f, relative));
    assert(sameQuaternion(relative, saved));
    current.valid = true;
    current.quaternion = {1.125f, 0.0f, 0.0f, 0.0f};
    assert(!computeRelativeOrientation(current, identityReference, 0.1f, relative));
    assert(sameQuaternion(relative, saved));
    current.quaternion = validCurrent;

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!computeRelativeOrientation(current, referenceX, nanValue, relative));
    assert(!computeRelativeOrientation(current, referenceX, infinity, relative));
    const ReferenceOrientation invalidQuaternionReference = {{nanValue, 0.0f, 0.0f, 0.0f}, true};
    assert(!computeRelativeOrientation(current, invalidQuaternionReference, 0.01f, relative));
    assert(sameQuaternion(relative, saved));
    assert(!computeRelativeOrientation(current, referenceX, -0.01f, relative));
    assert(sameQuaternion(relative, saved));

    // Небольшая допустимая погрешность нормы reference учитывается полной обратной величиной.
    const ReferenceOrientation nonUnitReference = {{0.7072f, 0.7072f, 0.0f, 0.0f}, true};
    assert(computeRelativeOrientation(current, nonUnitReference, 0.001f, relative));
    assert(nearlyEqual(relative.w * relative.w + relative.x * relative.x +
                       relative.y * relative.y + relative.z * relative.z,
                       1.0f));

    // Входные значения не модифицируются.
    assert(current.quaternion.w == validCurrent.w && current.quaternion.x == validCurrent.x &&
           current.quaternion.y == validCurrent.y && current.quaternion.z == validCurrent.z);
    assert(referenceX.quaternion.w == half && referenceX.quaternion.x == half);

    std::puts("computeRelativeOrientation tests passed");
    return 0;
}
