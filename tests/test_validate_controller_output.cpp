#include "../src/control/ControllerOutputValidation.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

int main()
{
    using namespace gyrostab;

    // Ноль и конечные поправки обоих знаков допустимы.
    assert(validateControllerOutput(0.0f));
    assert(validateControllerOutput(1.0f));
    assert(validateControllerOutput(-1.0f));
    assert(validateControllerOutput(1234.5f));
    assert(validateControllerOutput(-1234.5f));

    // Небольшие значения около нуля также допустимы.
    assert(validateControllerOutput(std::numeric_limits<float>::min()));
    assert(validateControllerOutput(-std::numeric_limits<float>::min()));

    // Проверяется именно диапазон int; физические min/max PWM здесь не заданы.
    const float largestRepresentableBelowIntMax = std::nextafter(
        static_cast<float>(std::numeric_limits<int>::max()), 0.0f);
    assert(validateControllerOutput(largestRepresentableBelowIntMax));
    assert(validateControllerOutput(
        static_cast<float>(std::numeric_limits<int>::lowest())));

    // Следующее float за пределами int недопустимо.
    const float firstFloatAboveIntMax = std::nextafter(
        static_cast<float>(std::numeric_limits<int>::max()),
        std::numeric_limits<float>::infinity());
    assert(!validateControllerOutput(firstFloatAboveIntMax));

    const float firstFloatBelowIntMin = std::nextafter(
        static_cast<float>(std::numeric_limits<int>::lowest()),
        -std::numeric_limits<float>::infinity());
    assert(!validateControllerOutput(firstFloatBelowIntMin));

    // NaN и обе бесконечности недопустимы.
    assert(!validateControllerOutput(std::numeric_limits<float>::quiet_NaN()));
    assert(!validateControllerOutput(std::numeric_limits<float>::infinity()));
    assert(!validateControllerOutput(-std::numeric_limits<float>::infinity()));

    std::puts("validateControllerOutput tests passed");
    return 0;
}
