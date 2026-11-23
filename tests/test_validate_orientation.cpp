#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

bool sameState(const gyrostab::OrientationEstimate& left,
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

    OrientationEstimate estimate = {
        {1.0f, 0.0f, 0.0f, 0.0f}, 0U, 0U, true
    };
    assert(validateOrientation(estimate, 0.0f));

    estimate.quaternion = {0.0f, 0.6f, 0.8f, 0.0f};
    assert(validateOrientation(estimate, 0.00001f));

    // Допуск включительный: значение точно на границе принимается.
    estimate.quaternion = {1.125f, 0.0f, 0.0f, 0.0f};
    assert(validateOrientation(estimate, 0.125f));
    estimate.quaternion = {1.25f, 0.0f, 0.0f, 0.0f};
    assert(!validateOrientation(estimate, 0.125f));

    // Без валидного результата и с нулевым quaternion оценка отклоняется.
    estimate.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    estimate.valid = false;
    assert(!validateOrientation(estimate, 0.01f));
    estimate.valid = true;
    estimate.quaternion = {0.0f, 0.0f, 0.0f, 0.0f};
    assert(!validateOrientation(estimate, 0.01f));

    // Каждая не конечная компонента должна быть отклонена.
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const Quaternionf invalidQuaternions[] = {
        {nanValue, 0.0f, 0.0f, 0.0f},
        {1.0f, nanValue, 0.0f, 0.0f},
        {1.0f, 0.0f, infinity, 0.0f},
        {1.0f, 0.0f, 0.0f, -infinity}
    };
    for (const Quaternionf& quaternion : invalidQuaternions)
    {
        estimate.quaternion = quaternion;
        assert(!validateOrientation(estimate, 0.01f));
    }

    estimate.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    assert(!validateOrientation(estimate, -0.01f));
    assert(!validateOrientation(estimate, nanValue));
    assert(!validateOrientation(estimate, infinity));

    // Значения timestamp/sequence не используются как признак валидности.
    estimate.timestampUs = 0U;
    estimate.sequence = 0U;
    assert(validateOrientation(estimate, 0.0f));

    // Проверка не изменяет исходную оценку.
    const OrientationEstimate saved = estimate;
    (void)validateOrientation(estimate, 0.0f);
    assert(sameState(estimate, saved));

    std::puts("validateOrientation tests passed");
    return 0;
}
