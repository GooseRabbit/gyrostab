#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

int main()
{
    using namespace gyrostab;

    // Initialization must overwrite every field, including invalid old state.
    OrientationEstimate estimate = {
        {std::numeric_limits<float>::quiet_NaN(), 7.0f, -8.0f, 9.0f},
        123456U,
        987U,
        true
    };

    initializeEstimator(estimate);

    assert(estimate.quaternion.w == 1.0f);
    assert(estimate.quaternion.x == 0.0f);
    assert(estimate.quaternion.y == 0.0f);
    assert(estimate.quaternion.z == 0.0f);
    assert(estimate.timestampUs == 0U);
    assert(estimate.sequence == 0U);
    assert(!estimate.valid);

    const float norm = std::sqrt(
        estimate.quaternion.w * estimate.quaternion.w +
        estimate.quaternion.x * estimate.quaternion.x +
        estimate.quaternion.y * estimate.quaternion.y +
        estimate.quaternion.z * estimate.quaternion.z);
    assert(norm == 1.0f);

    // Reinitialization is deterministic and idempotent.
    initializeEstimator(estimate);
    assert(estimate.quaternion.w == 1.0f);
    assert(estimate.quaternion.x == 0.0f);
    assert(estimate.quaternion.y == 0.0f);
    assert(estimate.quaternion.z == 0.0f);
    assert(estimate.timestampUs == 0U);
    assert(estimate.sequence == 0U);
    assert(!estimate.valid);

    std::puts("initializeEstimator tests passed");
    return 0;
}
