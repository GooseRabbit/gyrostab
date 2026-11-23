#include "../src/geometry/MountingConfig.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

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
    assert(validateMountingConfig(identity, 0.0f));

    // Поворот на 90 градусов вокруг Z: оси платформы заданы в координатах IMU.
    const MountingConfig rotated = {
        {0.0f, 1.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(validateMountingConfig(rotated, 0.0f));

    // Включительная граница допуска по длине оси.
    const MountingConfig boundary = {
        {1.0625f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(validateMountingConfig(boundary, 0.0625f));
    assert(!validateMountingConfig(boundary, 0.0624f));

    // Левая система координат и неперпендикулярные оси отклоняются.
    const MountingConfig mirrored = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(!validateMountingConfig(mirrored, 0.01f));

    const MountingConfig nonOrthogonal = {
        {1.0f, 0.0f, 0.0f},
        {0.125f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(!validateMountingConfig(nonOrthogonal, 0.01f));

    const MountingConfig nonUnit = {
        {1.25f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(!validateMountingConfig(nonUnit, 0.01f));

    const MountingConfig zeroAxis = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    assert(!validateMountingConfig(zeroAxis, 0.01f));

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    MountingConfig invalid = identity;
    invalid.platformYInSensorFrame.y = nanValue;
    assert(!validateMountingConfig(invalid, 0.01f));
    invalid = identity;
    invalid.platformZInSensorFrame.x = infinity;
    assert(!validateMountingConfig(invalid, 0.01f));

    assert(!validateMountingConfig(identity, -0.01f));
    assert(!validateMountingConfig(identity, nanValue));
    assert(!validateMountingConfig(identity, infinity));
    assert(!validateMountingConfig(identity, 1.0f));

    // Проверка не меняет исходную конфигурацию.
    const MountingConfig saved = rotated;
    (void)validateMountingConfig(rotated, 0.0f);
    assert(sameVector(rotated.platformXInSensorFrame, saved.platformXInSensorFrame));
    assert(sameVector(rotated.platformYInSensorFrame, saved.platformYInSensorFrame));
    assert(sameVector(rotated.platformZInSensorFrame, saved.platformZInSensorFrame));

    std::puts("validateMountingConfig tests passed");
    return 0;
}
