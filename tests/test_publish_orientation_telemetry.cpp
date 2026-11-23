#include "../src/diagnostics/OrientationTelemetry.h"
#include <Arduino.h>

#include <cassert>
#include <cstdio>
#include <iomanip>
#include <sstream>

MockSerialClass Serial;

void MockSerialClass::print(uint32_t value)
{
    output += std::to_string(value);
}

void MockSerialClass::print(int value)
{
    output += std::to_string(value);
}

void MockSerialClass::print(char value)
{
    output += value;
}

void MockSerialClass::print(float value, int digits)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(digits) << value;
    output += stream.str();
}

void MockSerialClass::println(float value, int digits)
{
    print(value, digits);
    output += '\n';
}

void MockSerialClass::clear()
{
    output.clear();
}

int main()
{
    using namespace gyrostab;

    OrientationEstimate estimate = {
        {0.5f, -0.25f, 0.125f, 0.75f},
        123456U,
        42U,
        true
    };

    publishOrientationTelemetry(estimate);
    assert(Serial.output ==
           "123456,42,1,0.500000,-0.250000,0.125000,0.750000\n");

    // Невалидная оценка тоже выводится, но явно помечается valid=0.
    Serial.clear();
    estimate = {{1.0f, 0.0f, 0.0f, 0.0f}, 0U, 0U, false};
    const OrientationEstimate saved = estimate;
    publishOrientationTelemetry(estimate);
    assert(Serial.output == "0,0,0,1.000000,0.000000,0.000000,0.000000\n");

    // Вывод не изменяет исходную оценку.
    assert(estimate.timestampUs == saved.timestampUs);
    assert(estimate.sequence == saved.sequence);
    assert(estimate.valid == saved.valid);
    assert(estimate.quaternion.w == saved.quaternion.w);
    assert(estimate.quaternion.x == saved.quaternion.x);
    assert(estimate.quaternion.y == saved.quaternion.y);
    assert(estimate.quaternion.z == saved.quaternion.z);

    // Проверяются отрицательные компоненты и максимальная метка uint32_t.
    Serial.clear();
    estimate = {{-1.0f, 0.0f, -0.125f, 0.0f}, 0xFFFFFFFFUL, 7U, true};
    publishOrientationTelemetry(estimate);
    assert(Serial.output ==
           "4294967295,7,1,-1.000000,0.000000,-0.125000,0.000000\n");

    std::puts("publishOrientationTelemetry tests passed");
    return 0;
}
