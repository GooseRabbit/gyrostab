#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>
#include <limits>

int main()
{
    using gyrostab::limitServoPulse;

    int limitedPulseUs = -1;

    // Значение внутри диапазона сохраняется без изменений.
    assert(limitServoPulse(1500, 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 1500);

    // Границы диапазона включены.
    assert(limitServoPulse(1000, 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 1000);
    assert(limitServoPulse(2000, 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 2000);

    // Запрос за пределами диапазона прижимается к ближайшей границе.
    assert(limitServoPulse(900, 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 1000);
    assert(limitServoPulse(2100, 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 2000);

    // Экстремальные запросы int не вызывают арифметического переполнения.
    assert(limitServoPulse(std::numeric_limits<int>::min(), 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 1000);
    assert(limitServoPulse(std::numeric_limits<int>::max(), 1000, 2000, limitedPulseUs));
    assert(limitedPulseUs == 2000);

    // Допустимы пределы API ESP32Servo, но они не означают безопасный ход механики.
    assert(limitServoPulse(2500, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, limitedPulseUs));
    assert(limitedPulseUs == MAX_PULSE_WIDTH);

    // Некорректные пределы отклоняются; выход остаётся прежним.
    limitedPulseUs = 1234;
    assert(!limitServoPulse(1500, MIN_PULSE_WIDTH - 1, 2000, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoPulse(1500, 1000, MAX_PULSE_WIDTH + 1, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoPulse(1500, 1500, 1500, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoPulse(1500, 1800, 1700, limitedPulseUs));
    assert(limitedPulseUs == 1234);
    assert(!limitServoPulse(1500, 2000, 1000, limitedPulseUs));
    assert(limitedPulseUs == 1234);

    // Ограничитель не взаимодействует с PWM сервопривода.
    Servo servo;
    assert(limitServoPulse(1600, 1000, 2000, limitedPulseUs));
    assert(servo.writeCalls() == 0);

    std::puts("limitServoPulse tests passed");
    return 0;
}
