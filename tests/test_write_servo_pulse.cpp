#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>

int main()
{
    using gyrostab::writeServoPulse;

    // Подключённая серва принимает команду внутри безопасных min/max.
    Servo servo;
    servo.setAttached(true);
    assert(writeServoPulse(servo, 1500, 1000, 2000));
    assert(servo.writeCalls() == 1);
    assert(servo.lastWrittenPulseUs() == 1500);

    // Включённые границы передаются без изменения.
    assert(writeServoPulse(servo, 1000, 1000, 2000));
    assert(servo.lastWrittenPulseUs() == 1000);
    assert(writeServoPulse(servo, 2000, 1000, 2000));
    assert(servo.lastWrittenPulseUs() == 2000);
    assert(servo.writeCalls() == 3);

    // Отключённый объект не получает команду.
    Servo detached;
    assert(!writeServoPulse(detached, 1500, 1000, 2000));
    assert(detached.writeCalls() == 0);

    // Команда за механическими min/max отклоняется, а не ограничивается здесь.
    assert(!writeServoPulse(servo, 999, 1000, 2000));
    assert(!writeServoPulse(servo, 2001, 1000, 2000));
    assert(servo.writeCalls() == 3);

    // Некорректные границы отклоняются до обращения к PWM.
    assert(!writeServoPulse(servo, 1500, 1500, 1500));
    assert(!writeServoPulse(servo, 1500, 1800, 1700));
    assert(!writeServoPulse(servo, 1500, 499, 2000));
    assert(!writeServoPulse(servo, 1500, 1000, 2501));
    assert(servo.writeCalls() == 3);

    // Значение проверяется как целое без преобразований или округления.
    assert(!writeServoPulse(servo, 1500, 1600, 2000));
    assert(!writeServoPulse(servo, 1500, 1000, 1400));
    assert(servo.writeCalls() == 3);

    std::puts("writeServoPulse tests passed");
    return 0;
}
