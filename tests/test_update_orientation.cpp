#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::ImuSample makeSample(uint32_t timestampUs, uint32_t sequence,
                               float gx, float gy, float gz, bool gyroValid = true)
{
    gyrostab::ImuSample sample = {};
    sample.angularVelocityRadPerSec = {gx, gy, gz};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.gyroscopeValid = gyroValid;
    return sample;
}

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-5f)
{
    return std::fabs(actual - expected) <= tolerance;
}

void assertSameState(const gyrostab::OrientationEstimate& actual,
                     const gyrostab::OrientationEstimate& expected)
{
    assert(actual.quaternion.w == expected.quaternion.w);
    assert(actual.quaternion.x == expected.quaternion.x);
    assert(actual.quaternion.y == expected.quaternion.y);
    assert(actual.quaternion.z == expected.quaternion.z);
    assert(actual.timestampUs == expected.timestampUs);
    assert(actual.sequence == expected.sequence);
    assert(actual.valid == expected.valid);
}

} // namespace

int main()
{
    using namespace gyrostab;
    const Vector3f zeroBias = {0.0f, 0.0f, 0.0f};

    // Первый образец фиксирует начальную привязку и не создаёт поворот.
    OrientationEstimate estimate = {};
    initializeEstimator(estimate);
    assert(updateOrientation(estimate, makeSample(1000U, 10U, 0.0f, 0.0f, 1.0f),
                             zeroBias, 10000U));
    assert(estimate.valid);
    assert(estimate.timestampUs == 1000U && estimate.sequence == 10U);
    assert(estimate.quaternion.w == 1.0f && estimate.quaternion.x == 0.0f &&
           estimate.quaternion.y == 0.0f && estimate.quaternion.z == 0.0f);

    // 1 рад/с вокруг Z в течение 1 секунды даёт ожидаемый half-angle quaternion.
    assert(updateOrientation(estimate, makeSample(1001000U, 11U, 0.0f, 0.0f, 1.0f),
                             zeroBias, 1000000U));
    assert(nearlyEqual(estimate.quaternion.w, static_cast<float>(std::cos(0.5))));
    assert(nearlyEqual(estimate.quaternion.x, 0.0f));
    assert(nearlyEqual(estimate.quaternion.y, 0.0f));
    assert(nearlyEqual(estimate.quaternion.z, static_cast<float>(std::sin(0.5))));

    const float norm = std::sqrt(
        estimate.quaternion.w * estimate.quaternion.w +
        estimate.quaternion.x * estimate.quaternion.x +
        estimate.quaternion.y * estimate.quaternion.y +
        estimate.quaternion.z * estimate.quaternion.z);
    assert(nearlyEqual(norm, 1.0f));

    // Вычитание bias убирает дополнительное вращение.
    const OrientationEstimate beforeBias = estimate;
    const Vector3f bias = {0.0f, 0.0f, 2.0f};
    assert(updateOrientation(estimate, makeSample(1002000U, 12U, 0.0f, 0.0f, 2.0f),
                             bias, 1000U));
    assert(nearlyEqual(estimate.quaternion.w, beforeBias.quaternion.w));
    assert(nearlyEqual(estimate.quaternion.z, beforeBias.quaternion.z));

    // Пропуск sequence допустим, если временной интервал укладывается в предел.
    assert(updateOrientation(estimate, makeSample(1003000U, 14U, 0.0f, 0.0f, 0.0f),
                             zeroBias, 1000U));

    // Беззнаковые разности корректно обрабатывают rollover timestamp и sequence.
    OrientationEstimate rollover = {};
    initializeEstimator(rollover);
    assert(updateOrientation(rollover, makeSample(0xFFFFFFF0UL, 0xFFFFFFFFUL, 0, 0, 0),
                             zeroBias, 100U));
    assert(updateOrientation(rollover, makeSample(0x00000010UL, 0U, 0, 0, 1),
                             zeroBias, 100U));
    assert(rollover.timestampUs == 0x10U && rollover.sequence == 0U);
    assert(nearlyEqual(rollover.quaternion.z, static_cast<float>(std::sin(0.000016))));

    // Невалидные входы и временные пределы не меняют оценку.
    const OrientationEstimate saved = estimate;
    assert(!updateOrientation(estimate, makeSample(1004000U, 15U, 0, 0, 0, false),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    assert(!updateOrientation(estimate, makeSample(1004000U, 15U, nanValue, 0, 0),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);
    const Vector3f badBias = {0.0f, std::numeric_limits<float>::infinity(), 0.0f};
    assert(!updateOrientation(estimate, makeSample(1004000U, 15U, 0, 0, 0),
                              badBias, 1000U));
    assertSameState(estimate, saved);
    assert(!updateOrientation(estimate, makeSample(1004000U, 15U, 0, 0, 0),
                              zeroBias, 0U));
    assertSameState(estimate, saved);
    assert(!updateOrientation(estimate, makeSample(1004000U, 15U, 0, 0, 0),
                              zeroBias, 0x80000000UL));
    assertSameState(estimate, saved);

    // Повторные/обратные метки и слишком большой dt отклоняются.
    assert(!updateOrientation(estimate, makeSample(1004000U, saved.sequence, 0, 0, 0),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);
    assert(!updateOrientation(estimate, makeSample(1004000U, saved.sequence - 1U, 0, 0, 0),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);
    assert(!updateOrientation(estimate, makeSample(saved.timestampUs, saved.sequence + 1U, 0, 0, 0),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);
    assert(!updateOrientation(estimate, makeSample(saved.timestampUs + 1001U, saved.sequence + 1U, 0, 0, 0),
                              zeroBias, 1000U));
    assertSameState(estimate, saved);

    // Нулевой quaternion при valid == true считается повреждённым состоянием.
    OrientationEstimate invalidState = saved;
    invalidState.quaternion = {0.0f, 0.0f, 0.0f, 0.0f};
    const OrientationEstimate invalidSaved = invalidState;
    assert(!updateOrientation(invalidState, makeSample(1004000U, 15U, 0, 0, 0),
                              zeroBias, 1000U));
    assertSameState(invalidState, invalidSaved);

    std::puts("updateOrientation tests passed");
    return 0;
}
