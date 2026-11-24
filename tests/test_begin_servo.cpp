#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>

int main()
{
    using gyrostab::beginServo;

    // Нулевой номер PWM-канала допустим: успешность определяется attached().
    Servo servo;
    servo.setChannel(0);
    assert(beginServo(servo, 18, 50, 1000, 2000));
    assert(servo.periodCalls() == 1);
    assert(servo.attachCalls() == 1);
    assert(servo.writeCalls() == 0);
    assert(servo.lastPeriodHz() == 50);
    assert(servo.lastPin() == 18);
    assert(servo.lastMinimumPulseUs() == 1000);
    assert(servo.lastMaximumPulseUs() == 2000);

    // Параметры вызывающего кода передаются без замены на неутверждённые значения.
    Servo custom;
    assert(beginServo(custom, 7, 60, 900, 2100));
    assert(custom.lastPeriodHz() == 60);
    assert(custom.lastPin() == 7);
    assert(custom.lastMinimumPulseUs() == 900);
    assert(custom.lastMaximumPulseUs() == 2100);
    assert(custom.writeCalls() == 0);

    // Границы диапазона ESP32Servo допустимы, если min меньше max.
    Servo edge;
    assert(beginServo(edge, 4, 50, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));

    // Неверные аргументы отклоняются до вызова библиотеки.
    Servo invalid;
    assert(!beginServo(invalid, -1, 50, 1000, 2000));
    assert(invalid.periodCalls() == 0 && invalid.attachCalls() == 0);
    assert(!beginServo(invalid, 18, 0, 1000, 2000));
    assert(invalid.periodCalls() == 0 && invalid.attachCalls() == 0);
    assert(!beginServo(invalid, 18, -50, 1000, 2000));
    assert(invalid.periodCalls() == 0 && invalid.attachCalls() == 0);
    assert(!beginServo(invalid, 18, 50, 499, 2000));
    assert(!beginServo(invalid, 18, 50, 1000, 2501));
    assert(!beginServo(invalid, 18, 50, 1500, 1500));
    assert(!beginServo(invalid, 18, 50, 1800, 1700));
    assert(invalid.periodCalls() == 0 && invalid.attachCalls() == 0);

    // Уже подключённый объект не перенастраивается молча.
    Servo alreadyAttached;
    alreadyAttached.setAttached(true);
    assert(!beginServo(alreadyAttached, 18, 50, 1000, 2000));
    assert(alreadyAttached.periodCalls() == 0);
    assert(alreadyAttached.attachCalls() == 0);

    // Отказ подключения отражается false, если объект остался неподключённым.
    Servo rejected;
    rejected.setAttachSucceeds(false);
    assert(!beginServo(rejected, 18, 50, 1000, 2000));
    assert(rejected.periodCalls() == 1);
    assert(rejected.attachCalls() == 1);
    assert(rejected.writeCalls() == 0);

    std::puts("beginServo tests passed");
    return 0;
}
