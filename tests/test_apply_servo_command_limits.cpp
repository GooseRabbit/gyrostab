#include "../src/control/ServoCommandLimits.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdint.h>

namespace
{

bool apply(float correctionUs, int centerPulseUs, int direction,
           int currentPulseUs, float maximumChangeUsPerSecond,
           uint32_t elapsedTimeUs, int& limitedPulseUs)
{
    return gyrostab::applyServoCommandLimits(
        correctionUs, centerPulseUs, direction, currentPulseUs,
        1000, 2000, maximumChangeUsPerSecond, elapsedTimeUs, limitedPulseUs);
}

} // namespace

int main()
{
    using gyrostab::applyServoCommandLimits;

    int limitedPulseUs = -1;

    // Положительная поправка добавляется к центру при направлении +1.
    assert(apply(150.0f, 1500, 1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1650);

    // Подтверждённая противоположная полярность меняет направление смещения.
    assert(apply(150.0f, 1500, -1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1350);

    // Отрицательная поправка сохраняет собственный знак.
    assert(apply(-150.0f, 1500, 1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1350);

    // Поправка округляется до ближайшей целой микросекунды.
    assert(apply(10.49f, 1500, 1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1510);
    assert(apply(10.5f, 1500, 1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1511);
    assert(apply(-10.5f, 1500, 1, 1500, 1000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1489);

    // Положение ограничивается заданными безопасными min/max.
    assert(apply(1000.0f, 1500, 1, 1500, 10000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 2000);
    assert(apply(-1000.0f, 1500, 1, 1500, 10000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1000);

    // После ограничения позиции применяется скорость изменения команды.
    assert(apply(1000.0f, 1500, 1, 1500, 100.0f, 250000U, limitedPulseUs));
    assert(limitedPulseUs == 1525);
    assert(apply(-1000.0f, 1500, 1, 1500, 100.0f, 250000U, limitedPulseUs));
    assert(limitedPulseUs == 1475);

    // Нулевая скорость или прошедшее время сохраняют текущую команду.
    assert(apply(300.0f, 1500, 1, 1500, 0.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1500);
    assert(apply(300.0f, 1500, 1, 1500, 100.0f, 0U, limitedPulseUs));
    assert(limitedPulseUs == 1500);

    // Текущая позиция, отличная от центра, также ограничивается корректно.
    assert(apply(100.0f, 1500, 1, 1550, 100.0f, 500000U, limitedPulseUs));
    assert(limitedPulseUs == 1600);

    const int savedOutput = limitedPulseUs;
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    // Некорректные correction/direction не меняют выход.
    assert(!apply(nanValue, 1500, 1, 1500, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(infinity, 1500, 1, 1500, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(10.0f, 1500, 0, 1500, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(10.0f, 1500, 2, 1500, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);

    // Центр и текущая позиция должны лежать в механическом безопасном диапазоне.
    assert(!apply(10.0f, 900, 1, 1500, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(10.0f, 1500, 1, 900, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);

    // Некорректные диапазоны или скорость отклоняются без изменения результата.
    assert(!applyServoCommandLimits(10.0f, 1500, 1, 1500,
                                    999, 2000, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!applyServoCommandLimits(10.0f, 1500, 1, 1500,
                                    1000, 2001, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(10.0f, 1500, 1, 1500, -1.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);
    assert(!apply(10.0f, 1500, 1, 1500, nanValue, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == savedOutput);

    // Сложение центра и большой поправки не переполняет int до ограничения.
    const float largePositiveCorrection = std::nextafter(
        static_cast<float>(std::numeric_limits<int>::max()), 0.0f);
    assert(apply(largePositiveCorrection, 1500, 1, 1500,
                 1000000000.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 2000);

    // При отказе выходное значение остаётся без изменений.
    assert(!apply(10.0f, 1500, 1, 1500, 100.0f, 1000000U, limitedPulseUs)
           == false);

    std::puts("applyServoCommandLimits tests passed");
    return 0;
}
