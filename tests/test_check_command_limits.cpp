#include "../src/safety/SafetyManager.h"
#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace gyrostab
{

// Эти функции не используются в тесте checkCommandLimits(); заглушки нужны
// для линковки остальных функций общего модуля SafetyManager.cpp.
bool isSampleFresh(const ImuSample&, uint32_t, uint32_t)
{
    return false;
}

bool validateOrientation(const OrientationEstimate&, float)
{
    return false;
}

} // namespace gyrostab

int main()
{
    using gyrostab::checkCommandLimits;

    // Внутренняя часть диапазона и обе включённые границы принимаются.
    assert(checkCommandLimits(1500, 1000, 2000));
    assert(checkCommandLimits(1000, 1000, 2000));
    assert(checkCommandLimits(2000, 1000, 2000));

    // Цель за нижней и верхней границами отклоняется, а не прижимается.
    assert(!checkCommandLimits(999, 1000, 2000));
    assert(!checkCommandLimits(2001, 1000, 2000));
    assert(!checkCommandLimits(std::numeric_limits<int>::min(), 1000, 2000));
    assert(!checkCommandLimits(std::numeric_limits<int>::max(), 1000, 2000));

    // Допускаются крайние пределы API, если сами min/max заданы корректно.
    assert(checkCommandLimits(500, 500, 2500));
    assert(checkCommandLimits(2500, 500, 2500));

    // Недопустимые пределы API или пустой/обратный диапазон отклоняются.
    assert(!checkCommandLimits(1500, 499, 2000));
    assert(!checkCommandLimits(1500, 1000, 2501));
    assert(!checkCommandLimits(1500, 1000, 1000));
    assert(!checkCommandLimits(1500, 1800, 1700));
    assert(!checkCommandLimits(1500, 2000, 1000));

    // Проверка не взаимодействует с объектом серво и не выдаёт PWM.
    Servo servo;
    assert(checkCommandLimits(1600, 1000, 2000));
    assert(servo.writeCalls() == 0);

    std::puts("checkCommandLimits tests passed");
    return 0;
}
