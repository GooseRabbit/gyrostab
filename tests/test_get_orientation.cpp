#include "../src/orientation/OrientationEstimator.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace
{
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
}

int main()
{
    using namespace gyrostab;
    const OrientationEstimate latest = {{0.5f,0.5f,0.5f,0.5f},0U,0xFFFFFFFFUL,true};
    OrientationEstimate output = {{1.0f,0.0f,0.0f,0.0f},123U,456U,false};

    // Валидная оценка копируется целиком; нулевой timestamp допустим.
    assert(getOrientation(latest, 0.0f, output));
    assert(sameEstimate(output, latest));

    // Нулевые и максимальные метаданные не используются как критерий валидности.
    OrientationEstimate maximumMetadata = latest;
    maximumMetadata.timestampUs = 0xFFFFFFFFUL;
    maximumMetadata.sequence = 0U;
    assert(getOrientation(maximumMetadata, 0.0f, output));
    assert(sameEstimate(output, maximumMetadata));

    const OrientationEstimate savedOutput = output;
    OrientationEstimate invalid = latest;
    invalid.valid = false;
    assert(!getOrientation(invalid, 0.01f, output));
    assert(sameEstimate(output, savedOutput));

    invalid = latest;
    invalid.quaternion = {0.0f,0.0f,0.0f,0.0f};
    assert(!getOrientation(invalid, 0.01f, output));
    assert(sameEstimate(output, savedOutput));

    invalid = latest;
    invalid.quaternion.w = 1.1f;
    assert(!getOrientation(invalid, 0.01f, output));
    assert(sameEstimate(output, savedOutput));

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    invalid = latest;
    invalid.quaternion.y = nanValue;
    assert(!getOrientation(invalid, 0.01f, output));
    assert(sameEstimate(output, savedOutput));

    invalid = latest;
    invalid.quaternion.z = infinity;
    assert(!getOrientation(invalid, 0.01f, output));
    assert(sameEstimate(output, savedOutput));

    assert(!getOrientation(latest, -0.01f, output));
    assert(sameEstimate(output, savedOutput));
    assert(!getOrientation(latest, nanValue, output));
    assert(sameEstimate(output, savedOutput));
    assert(!getOrientation(latest, infinity, output));
    assert(sameEstimate(output, savedOutput));

    // Getter не меняет источник и допускает безопасный alias output/source.
    const OrientationEstimate original = latest;
    assert(getOrientation(latest, 0.0f, output));
    assert(sameEstimate(latest, original));
    assert(getOrientation(output, 0.0f, output));
    assert(sameEstimate(output, original));

    std::puts("getOrientation tests passed");
    return 0;
}
