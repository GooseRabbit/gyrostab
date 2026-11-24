#include "../src/safety/SafetyManager.h"
#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace gyrostab
{

// SensorManager's driver boundary is stubbed; its freshness helper is linked separately.
bool readImuSample(ImuSample&)
{
    return false;
}

ICM_20948_Status_e checkImuStatus()
{
    return ICM_20948_Stat_Unknown;
}

} // namespace gyrostab

namespace
{

gyrostab::OrientationEstimate makeEstimate(uint32_t timestampUs)
{
    const gyrostab::OrientationEstimate estimate = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        timestampUs,
        12U,
        true
    };
    return estimate;
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

void testAcceptsFreshEstimateAtAgeBoundary()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    assert(gyrostab::checkEstimatorHealth(estimate, 150U, 50U, 0.001f));
}

void testRejectsExpiredEstimate()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    assert(!gyrostab::checkEstimatorHealth(estimate, 151U, 50U, 0.001f));
}

void testZeroAgeLimitAndSameTick()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    assert(gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.0f));
    assert(!gyrostab::checkEstimatorHealth(estimate, 101U, 0U, 0.0f));
}

void testHandlesMicrosRollover()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(UINT32_MAX - 4U);
    assert(gyrostab::checkEstimatorHealth(estimate, 5U, 10U, 0.0f));
    assert(!gyrostab::checkEstimatorHealth(estimate, 5U, 9U, 0.0f));
}

void testRejectsFutureTimestampAndAmbiguousAgeLimit()
{
    const gyrostab::OrientationEstimate futureEstimate = makeEstimate(101U);
    assert(!gyrostab::checkEstimatorHealth(futureEstimate, 100U, 1000U, 0.0f));

    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0x80000000UL, 0.0f));
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, UINT32_MAX, 0.0f));
}

void testRejectsInvalidEstimateAndQuaternion()
{
    gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    estimate.valid = false;
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.01f));

    estimate = makeEstimate(100U);
    estimate.quaternion = {0.0f, 0.0f, 0.0f, 0.0f};
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.01f));

    estimate = makeEstimate(100U);
    estimate.quaternion = {1.25f, 0.0f, 0.0f, 0.0f};
    assert(gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.25f));
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.1f));
}

void testRejectsInvalidNormTolerance()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, -0.01f));
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, nanValue));
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, infinity));
}

void testRejectsNonFiniteQuaternion()
{
    gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    estimate.quaternion.x = std::numeric_limits<float>::quiet_NaN();
    assert(!gyrostab::checkEstimatorHealth(estimate, 100U, 0U, 0.01f));
}

void testDoesNotModifyEstimate()
{
    const gyrostab::OrientationEstimate estimate = makeEstimate(100U);
    const gyrostab::OrientationEstimate saved = estimate;
    assert(gyrostab::checkEstimatorHealth(estimate, 110U, 10U, 0.0f));
    assert(sameEstimate(estimate, saved));
}

} // namespace

int main()
{
    testAcceptsFreshEstimateAtAgeBoundary();
    testRejectsExpiredEstimate();
    testZeroAgeLimitAndSameTick();
    testHandlesMicrosRollover();
    testRejectsFutureTimestampAndAmbiguousAgeLimit();
    testRejectsInvalidEstimateAndQuaternion();
    testRejectsInvalidNormTolerance();
    testRejectsNonFiniteQuaternion();
    testDoesNotModifyEstimate();

    std::puts("checkEstimatorHealth tests passed");
    return 0;
}
