#include "../src/diagnostics/SystemTelemetry.h"
#include <Arduino.h>

#include <cassert>
#include <cstdint>
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

    DiagnosticCounters counters = {};

    // Одно новое принятое измерение, без ошибок и пропусков.
    updateCounters(counters, 1U, 0U, 0U, 0U);
    assert(counters.newPackets == 1U);
    assert(counters.spiErrors == 0U);
    assert(counters.missedSamples == 0U);
    assert(counters.timeouts == 0U);

    // Независимые приращения накапливаются в соответствующих полях.
    updateCounters(counters, 4U, 2U, 3U, 1U);
    assert(counters.newPackets == 5U);
    assert(counters.spiErrors == 2U);
    assert(counters.missedSamples == 3U);
    assert(counters.timeouts == 1U);

    // Нулевые приращения не меняют счётчики.
    updateCounters(counters, 0U, 0U, 0U, 0U);
    assert(counters.newPackets == 5U);
    assert(counters.spiErrors == 2U);
    assert(counters.missedSamples == 3U);
    assert(counters.timeouts == 1U);

    // Переполнение unsigned-счётчиков определено по модулю 2^32.
    counters.newPackets = UINT32_MAX;
    counters.spiErrors = UINT32_MAX - 1U;
    counters.missedSamples = UINT32_MAX - 2U;
    counters.timeouts = UINT32_MAX - 3U;
    updateCounters(counters, 1U, 2U, 3U, 4U);
    assert(counters.newPackets == 0U);
    assert(counters.spiErrors == 0U);
    assert(counters.missedSamples == 0U);
    assert(counters.timeouts == 0U);

    // Функция только обновляет счётчики и не выводит данные.
    assert(Serial.output.empty());

    std::puts("updateCounters tests passed");
    return 0;
}
