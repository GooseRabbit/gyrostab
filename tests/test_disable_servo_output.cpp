#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>

int main()
{
    using gyrostab::disableServoOutput;

    // Подключённая серва отключается через detach().
    Servo servo;
    servo.setAttached(true);
    assert(disableServoOutput(servo));
    assert(!servo.attached());
    assert(servo.detachCalls() == 1);
    assert(servo.writeCalls() == 0);

    // Повторный вызов идемпотентен и не требует повторного detach().
    assert(disableServoOutput(servo));
    assert(servo.detachCalls() == 1);
    assert(servo.writeCalls() == 0);

    // Уже отключённый объект считается приведённым в требуемое состояние.
    Servo alreadyDetached;
    assert(disableServoOutput(alreadyDetached));
    assert(!alreadyDetached.attached());
    assert(alreadyDetached.detachCalls() == 0);
    assert(alreadyDetached.writeCalls() == 0);

    std::puts("disableServoOutput tests passed");
    return 0;
}
