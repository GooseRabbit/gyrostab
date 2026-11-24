#include "../src/control/PdCorrection.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

bool nearlyEqual(float actual, float expected, float tolerance = 1.0e-5f)
{
    return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
    using namespace gyrostab;

    float correctionUs = 77.0f;

    // Нулевая ошибка и нулевая скорость не создают поправку.
    assert(computePdCorrection(0.0f, 0.0f, 100.0f, 10.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 0.0f));

    // P-составляющая действует со знаком угловой ошибки.
    assert(computePdCorrection(0.25f, 0.0f, 400.0f, 10.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 100.0f));
    assert(computePdCorrection(-0.25f, 0.0f, 400.0f, 10.0f, correctionUs));
    assert(nearlyEqual(correctionUs, -100.0f));

    // D-составляющая вычитается: скорость в положительном направлении
    // уменьшает поправку, а отрицательная скорость увеличивает её.
    assert(computePdCorrection(0.0f, 2.0f, 400.0f, 10.0f, correctionUs));
    assert(nearlyEqual(correctionUs, -20.0f));
    assert(computePdCorrection(0.0f, -2.0f, 400.0f, 10.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 20.0f));

    // Обе составляющие складываются по заданной формуле.
    assert(computePdCorrection(0.2f, 0.5f, 100.0f, 8.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 16.0f));

    // Допускаются нулевые коэффициенты для независимой настройки P или D.
    assert(computePdCorrection(0.2f, 0.5f, 100.0f, 0.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 20.0f));
    assert(computePdCorrection(0.2f, 0.5f, 0.0f, 8.0f, correctionUs));
    assert(nearlyEqual(correctionUs, -4.0f));

    const float savedCorrection = correctionUs;
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    // Неконечные входы и отрицательные усиления отклоняются без изменения выхода.
    assert(!computePdCorrection(nanValue, 0.0f, 1.0f, 1.0f, correctionUs));
    assert(correctionUs == savedCorrection);
    assert(!computePdCorrection(0.0f, infinity, 1.0f, 1.0f, correctionUs));
    assert(correctionUs == savedCorrection);
    assert(!computePdCorrection(0.0f, 0.0f, nanValue, 1.0f, correctionUs));
    assert(correctionUs == savedCorrection);
    assert(!computePdCorrection(0.0f, 0.0f, 1.0f, infinity, correctionUs));
    assert(correctionUs == savedCorrection);
    assert(!computePdCorrection(0.0f, 0.0f, -0.1f, 1.0f, correctionUs));
    assert(correctionUs == savedCorrection);
    assert(!computePdCorrection(0.0f, 0.0f, 1.0f, -0.1f, correctionUs));
    assert(correctionUs == savedCorrection);

    // Конечные входы, переполняющие float-результат, также отклоняются.
    const float largest = std::numeric_limits<float>::max();
    assert(!computePdCorrection(largest, 0.0f, largest, 0.0f, correctionUs));
    assert(correctionUs == savedCorrection);

    // Функция не хранит состояние: одинаковые входы дают одинаковый результат.
    assert(computePdCorrection(0.1f, 0.0f, 100.0f, 0.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 10.0f));
    assert(computePdCorrection(0.1f, 0.0f, 100.0f, 0.0f, correctionUs));
    assert(nearlyEqual(correctionUs, 10.0f));

    std::puts("computePdCorrection tests passed");
    return 0;
}
