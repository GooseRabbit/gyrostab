#include "../src/calibration/CalibrationManager.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::ImuSample makeSample(float x, float y, float z)
{
    gyrostab::ImuSample sample = {};
    sample.magneticFieldMicroTesla = {x, y, z};
    sample.magnetometerValid = true;
    return sample;
}

void assertOffset(const gyrostab::Vector3f& offset,
                  float x, float y, float z)
{
    assert(std::fabs(offset.x - x) < 0.000001f);
    assert(std::fabs(offset.y - y) < 0.000001f);
    assert(std::fabs(offset.z - z) < 0.000001f);
}

void assertOffsetUnchanged(const gyrostab::Vector3f& offset)
{
    assert(offset.x == 7.0f);
    assert(offset.y == 8.0f);
    assert(offset.z == 9.0f);
}

} // namespace

namespace gyrostab
{

// Не используется calibrateMagnetometer(); требуется только для линковки других функций файла.
bool isSampleFresh(const ImuSample&, uint32_t, uint32_t)
{
    return false;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;

    ImuSample samples[] = {
        makeSample(10.0f, -30.0f, 4.0f),
        makeSample(30.0f, 10.0f, 24.0f),
        makeSample(20.0f, 0.0f, -16.0f),
        makeSample(50.0f, -10.0f, 44.0f)
    };
    const Vector3f minimumSpan = {40.0f, 40.0f, 60.0f};

    Vector3f offset = {7.0f, 8.0f, 9.0f};
    assert(calibrateMagnetometer(samples, 4U, minimumSpan, offset));
    assertOffset(offset, 30.0f, -10.0f, 14.0f);

    // Минимальный разброс включителен; превышение заданного порога отклоняется.
    offset = {7.0f, 8.0f, 9.0f};
    assert(calibrateMagnetometer(samples, 4U, minimumSpan, offset));
    assertOffset(offset, 30.0f, -10.0f, 14.0f);
    offset = {7.0f, 8.0f, 9.0f};
    const Vector3f excessiveSpan = {40.01f, 40.0f, 60.0f};
    assert(!calibrateMagnetometer(samples, 4U, excessiveSpan, offset));
    assertOffsetUnchanged(offset);

    // Недостаточное окно, null и отсутствие данных отклоняются.
    assert(!calibrateMagnetometer(nullptr, 4U, minimumSpan, offset));
    assert(!calibrateMagnetometer(samples, 0U, minimumSpan, offset));
    assert(!calibrateMagnetometer(samples, 1U, minimumSpan, offset));
    assertOffsetUnchanged(offset);

    // Все три оси должны иметь заданный ненулевой разброс.
    ImuSample constantAxis[] = {
        makeSample(1.0f, 2.0f, 3.0f),
        makeSample(2.0f, 2.0f, 4.0f),
        makeSample(3.0f, 2.0f, 5.0f)
    };
    const Vector3f positiveSpan = {1.0f, 1.0f, 1.0f};
    assert(!calibrateMagnetometer(constantAxis, 3U, positiveSpan, offset));
    assertOffsetUnchanged(offset);

    // Невалидный или неконечный магнитометрический канал отклоняет всё окно.
    samples[2].magnetometerValid = false;
    assert(!calibrateMagnetometer(samples, 4U, minimumSpan, offset));
    assertOffsetUnchanged(offset);
    samples[2].magnetometerValid = true;
    samples[2].magneticFieldMicroTesla.x = std::numeric_limits<float>::quiet_NaN();
    assert(!calibrateMagnetometer(samples, 4U, minimumSpan, offset));
    assertOffsetUnchanged(offset);
    samples[2].magneticFieldMicroTesla.x = 20.0f;
    samples[1].magneticFieldMicroTesla.y = std::numeric_limits<float>::infinity();
    assert(!calibrateMagnetometer(samples, 4U, minimumSpan, offset));
    assertOffsetUnchanged(offset);
    samples[1].magneticFieldMicroTesla.y = 10.0f;

    // Некорректные критерии отклоняются до публикации результата.
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const Vector3f negativeSpan = {-1.0f, 1.0f, 1.0f};
    const Vector3f zeroSpan = {1.0f, 0.0f, 1.0f};
    const Vector3f nanSpan = {1.0f, nanValue, 1.0f};
    const Vector3f infiniteSpan = {1.0f, 1.0f, infinity};
    assert(!calibrateMagnetometer(samples, 4U, negativeSpan, offset));
    assert(!calibrateMagnetometer(samples, 4U, zeroSpan, offset));
    assert(!calibrateMagnetometer(samples, 4U, nanSpan, offset));
    assert(!calibrateMagnetometer(samples, 4U, infiniteSpan, offset));
    assertOffsetUnchanged(offset);

    // Функция не меняет входные измерения.
    assert(samples[0].magneticFieldMicroTesla.x == 10.0f);
    assert(samples[0].magneticFieldMicroTesla.y == -30.0f);
    assert(samples[0].magneticFieldMicroTesla.z == 4.0f);

    std::puts("calibrateMagnetometer tests passed");
    return 0;
}
