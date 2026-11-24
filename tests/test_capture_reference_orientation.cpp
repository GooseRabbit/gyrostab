#include "../src/geometry/ReferenceOrientation.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace
{

bool sameReference(const gyrostab::ReferenceOrientation& left,
                   const gyrostab::ReferenceOrientation& right)
{
    return left.quaternion.w == right.quaternion.w &&
           left.quaternion.x == right.quaternion.x &&
           left.quaternion.y == right.quaternion.y &&
           left.quaternion.z == right.quaternion.z &&
           left.valid == right.valid;
}

bool sameEstimate(const gyrostab::OrientationEstimate& left,
                  const gyrostab::OrientationEstimate& right)
{
    return left.quaternion.w == right.quaternion.w &&
           left.quaternion.x == right.quaternion.x &&
           left.quaternion.y == right.quaternion.y &&
           left.quaternion.z == right.quaternion.z &&
           left.timestampUs == right.timestampUs &&
           left.sequence == right.sequence &&
           left.valid == right.valid;
}

} // namespace

int main()
{
    using namespace gyrostab;

    OrientationEstimate current = {
        {0.5f, -0.5f, 0.5f, -0.5f},
        123456U,
        42U,
        true
    };
    ReferenceOrientation reference = {{1.0f, 0.0f, 0.0f, 0.0f}, false};
    const OrientationEstimate original = current;
    assert(captureReferenceOrientation(current, 0.0f, reference));
    assert(reference.valid);
    assert(reference.quaternion.w == current.quaternion.w);
    assert(reference.quaternion.x == current.quaternion.x);
    assert(reference.quaternion.y == current.quaternion.y);
    assert(reference.quaternion.z == current.quaternion.z);
    assert(sameEstimate(current, original));

    // Повторный захват заменяет прежнее опорное положение.
    current.quaternion = {0.0f, 0.0f, 0.6f, 0.8f};
    current.timestampUs = 200000U;
    assert(captureReferenceOrientation(current, 0.00001f, reference));
    assert(reference.quaternion.w == 0.0f && reference.quaternion.x == 0.0f &&
           reference.quaternion.y == 0.6f && reference.quaternion.z == 0.8f);

    // Не установлен valid-флаг — существующая опора остаётся без изменений.
    const ReferenceOrientation saved = reference;
    current.valid = false;
    assert(!captureReferenceOrientation(current, 0.01f, reference));
    assert(sameReference(reference, saved));
    current.valid = true;

    // Некорректная норма за пределами допуска не захватывается.
    current.quaternion = {1.125f, 0.0f, 0.0f, 0.0f};
    assert(!captureReferenceOrientation(current, 0.1f, reference));
    assert(sameReference(reference, saved));

    // NaN/Inf в оценке и некорректные допуски также отклоняются.
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    current.quaternion = {nanValue, 0.0f, 0.0f, 0.0f};
    assert(!captureReferenceOrientation(current, 0.01f, reference));
    assert(sameReference(reference, saved));
    current.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    assert(!captureReferenceOrientation(current, nanValue, reference));
    assert(!captureReferenceOrientation(current, infinity, reference));
    assert(!captureReferenceOrientation(current, -0.01f, reference));
    assert(sameReference(reference, saved));

    std::puts("captureReferenceOrientation tests passed");
    return 0;
}
