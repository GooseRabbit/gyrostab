#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace
{
bool isNeutral(const gyrostab::OrientationEstimate& estimate)
{
    return estimate.quaternion.w == 1.0f && estimate.quaternion.x == 0.0f &&
           estimate.quaternion.y == 0.0f && estimate.quaternion.z == 0.0f &&
           estimate.timestampUs == 0U && estimate.sequence == 0U && !estimate.valid;
}
}

int main()
{
    using namespace gyrostab;

    // Полная перезапись прежней, в том числе повреждённой, оценки.
    OrientationEstimate estimate = {
        {std::numeric_limits<float>::quiet_NaN(), -3.0f, 4.0f, 5.0f},
        123456U,
        789U,
        true
    };
    resetOrientation(estimate);
    assert(isNeutral(estimate));

    // Повторный вызов сохраняет то же детерминированное состояние.
    resetOrientation(estimate);
    assert(isNeutral(estimate));

    // Сброс очищает оценку, которая до этого стала валидной после IMU sample.
    ImuSample sample = {};
    sample.angularVelocityRadPerSec = {0.0f, 0.0f, 0.0f};
    sample.timestampUs = 1000U;
    sample.sequence = 1U;
    sample.gyroscopeValid = true;
    const Vector3f zeroBias = {0.0f, 0.0f, 0.0f};
    assert(updateOrientation(estimate, sample, zeroBias, 10000U));
    assert(estimate.valid);
    assert(estimate.timestampUs == 1000U && estimate.sequence == 1U);
    resetOrientation(estimate);
    assert(isNeutral(estimate));

    std::puts("resetOrientation tests passed");
    return 0;
}
