#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdint.h>

int main()
{
    using gyrostab::limitServoSlewRate;

    int limitedPulseUs = -1;

    // Полный переход разрешён, если прошедшего времени достаточно.
    assert(limitServoSlewRate(1500, 1600, 100.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1600);

    // Ограничение вверх и вниз по 100 мкс/с за 250 мс даёт 25 мкс.
    assert(limitServoSlewRate(1500, 1700, 100.0f, 250000U, limitedPulseUs));
    assert(limitedPulseUs == 1525);
    assert(limitServoSlewRate(1500, 1300, 100.0f, 250000U, limitedPulseUs));
    assert(limitedPulseUs == 1475);

    // Нулевая скорость и нулевой интервал не разрешают изменение команды.
    assert(limitServoSlewRate(1500, 1800, 0.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1500);
    assert(limitServoSlewRate(1500, 1800, 100.0f, 0U, limitedPulseUs));
    assert(limitedPulseUs == 1500);

    // Дробная допустимая величина округляется вниз, не превышая скорость.
    assert(limitServoSlewRate(1000, 1100, 0.5f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1000);
    assert(limitServoSlewRate(1000, 1100, 0.5f, 2000000U, limitedPulseUs));
    assert(limitedPulseUs == 1001);

    // Одинаковая текущая и целевая команда возвращается без изменения.
    assert(limitServoSlewRate(1500, 1500, 100.0f, 100000U, limitedPulseUs));
    assert(limitedPulseUs == 1500);

    // Крайние значения int не вызывают переполнения разности или промежуточной суммы.
    assert(limitServoSlewRate(std::numeric_limits<int>::min(),
                              std::numeric_limits<int>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<uint32_t>::max(),
                              limitedPulseUs));
    assert(limitedPulseUs == std::numeric_limits<int>::max());
    assert(limitServoSlewRate(std::numeric_limits<int>::max(),
                              std::numeric_limits<int>::min(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<uint32_t>::max(),
                              limitedPulseUs));
    assert(limitedPulseUs == std::numeric_limits<int>::min());

    // Некорректная скорость не меняет выход.
    limitedPulseUs = 1234;
    assert(!limitServoSlewRate(1500, 1600, -1.0f, 1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoSlewRate(1500, 1600,
                               std::numeric_limits<float>::infinity(),
                               1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoSlewRate(1500, 1600,
                               std::numeric_limits<float>::quiet_NaN(),
                               1000000U, limitedPulseUs));
    assert(limitedPulseUs == 1234);

    // Функция ограничивает численную команду, но не отправляет PWM.
    Servo servo;
    assert(limitServoSlewRate(1500, 1600, 100.0f, 1000000U, limitedPulseUs));
    assert(servo.writeCalls() == 0);

    std::puts("limitServoSlewRate tests passed");
    return 0;
}
