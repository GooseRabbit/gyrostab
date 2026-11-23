#include "../src/geometry/FrameTransform.h"

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

bool sameVector(const gyrostab::Vector3f& left, const gyrostab::Vector3f& right)
{
    return left.x == right.x && left.y == right.y && left.z == right.z;
}

} // namespace

int main()
{
    using namespace gyrostab;

    const MountingConfig identity = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    Vector3f platform = {0.0f, 0.0f, 0.0f};
    const Vector3f sensor = {1.25f, -2.5f, 9.0f};
    assert(transformImuToPlatform(sensor, identity, platform));
    assert(nearlyEqual(platform.x, 1.25f));
    assert(nearlyEqual(platform.y, -2.5f));
    assert(nearlyEqual(platform.z, 9.0f));

    // Поворот платформы на +90 градусов вокруг Z относительно IMU.
    const MountingConfig rotated = {
        {0.0f, 1.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(transformImuToPlatform({1.0f, 0.0f, 0.0f}, rotated, platform));
    assert(nearlyEqual(platform.x, 0.0f));
    assert(nearlyEqual(platform.y, -1.0f));
    assert(nearlyEqual(platform.z, 0.0f));

    // Для общего ортонормированного повёрнутого базиса проверяются проекции.
    const float half = static_cast<float>(std::sqrt(0.5));
    const MountingConfig diagonal = {
        {half, half, 0.0f},
        {-half, half, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(transformImuToPlatform({1.0f, 0.0f, 2.0f}, diagonal, platform));
    assert(nearlyEqual(platform.x, half));
    assert(nearlyEqual(platform.y, -half));
    assert(nearlyEqual(platform.z, 2.0f));

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const Vector3f saved = {7.0f, 8.0f, 9.0f};
    platform = saved;
    assert(!transformImuToPlatform({nanValue, 0.0f, 0.0f}, identity, platform));
    assert(sameVector(platform, saved));
    assert(!transformImuToPlatform({0.0f, infinity, 0.0f}, identity, platform));
    assert(sameVector(platform, saved));

    // Некорректная конечная конфигурация, не прошедшая validateMountingConfig(),
    // может переполнить float на выходе; функция сохраняет выход при отказе.
    const MountingConfig oversized = {
        {std::numeric_limits<float>::max(), 0.0f, 0.0f},
        {0.0f, std::numeric_limits<float>::max(), 0.0f},
        {0.0f, 0.0f, std::numeric_limits<float>::max()}
    };
    assert(!transformImuToPlatform({2.0f, 2.0f, 2.0f}, oversized, platform));
    assert(sameVector(platform, saved));

    // Входной вектор и базис не изменяются.
    assert(sensor.x == 1.25f && sensor.y == -2.5f && sensor.z == 9.0f);
    assert(rotated.platformXInSensorFrame.x == 0.0f &&
           rotated.platformXInSensorFrame.y == 1.0f &&
           rotated.platformXInSensorFrame.z == 0.0f);
    assert(rotated.platformYInSensorFrame.x == -1.0f &&
           rotated.platformYInSensorFrame.y == 0.0f &&
           rotated.platformYInSensorFrame.z == 0.0f);
    assert(rotated.platformZInSensorFrame.x == 0.0f &&
           rotated.platformZInSensorFrame.y == 0.0f &&
           rotated.platformZInSensorFrame.z == 1.0f);

    std::puts("transformImuToPlatform tests passed");
    return 0;
}
