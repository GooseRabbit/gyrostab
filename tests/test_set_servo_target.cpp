#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>
#include <limits>

int main()
{
    using gyrostab::setServoTarget;

    int pendingTargetPulseUs = 1500;
    setServoTarget(1750, pendingTargetPulseUs);
    assert(pendingTargetPulseUs == 1750);

    // Запрос сохраняется без преждевременного ограничения диапазона.
    setServoTarget(2800, pendingTargetPulseUs);
    assert(pendingTargetPulseUs == 2800);
    setServoTarget(-100, pendingTargetPulseUs);
    assert(pendingTargetPulseUs == -100);

    // Нулевое и граничные значения int копируются без арифметики.
    setServoTarget(0, pendingTargetPulseUs);
    assert(pendingTargetPulseUs == 0);
    setServoTarget(std::numeric_limits<int>::min(), pendingTargetPulseUs);
    assert(pendingTargetPulseUs == std::numeric_limits<int>::min());
    setServoTarget(std::numeric_limits<int>::max(), pendingTargetPulseUs);
    assert(pendingTargetPulseUs == std::numeric_limits<int>::max());

    // Состояния, принадлежащие разным вызывающим объектам, независимы.
    int firstPending = 1000;
    int secondPending = 2000;
    setServoTarget(1100, firstPending);
    setServoTarget(2100, secondPending);
    assert(firstPending == 1100);
    assert(secondPending == 2100);

    // Функция не получает Servo и не формирует PWM-команду.
    Servo servo;
    setServoTarget(1600, pendingTargetPulseUs);
    assert(pendingTargetPulseUs == 1600);
    assert(servo.writeCalls() == 0);

    std::puts("setServoTarget tests passed");
    return 0;
}
