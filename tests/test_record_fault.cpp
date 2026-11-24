#include "../src/diagnostics/SystemTelemetry.h"
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

    TelemetrySnapshot snapshot = {
        123U, 1000U, 200.0f, true, true, -0.125f, 0.25f, true,
        12.5f, -3.25f, 9.25f, 1475, SafetyState::Active, 0U, nullptr
    };
    const char firstReason[] = "IMU timeout";
    const char secondReason[] = "orientation invalid";

    recordFault(snapshot, 17U, firstReason);
    assert(snapshot.faultCode == 17U);
    assert(snapshot.faultReason == firstReason);
    assert(snapshot.faultReason != nullptr);
    assert(snapshot.safetyState == SafetyState::Active);

    // Повторная запись заменяет только код и причину.
    recordFault(snapshot, 42U, secondReason);
    assert(snapshot.faultCode == 42U);
    assert(snapshot.faultReason == secondReason);
    assert(snapshot.safetyState == SafetyState::Active);

    // nullptr очищает указатель; код сохраняется ровно таким, каким его передал вызывающий код.
    recordFault(snapshot, 0U, nullptr);
    assert(snapshot.faultCode == 0U);
    assert(snapshot.faultReason == nullptr);

    // Остальные поля диагностического снимка не меняются.
    assert(snapshot.timestampUs == 123U);
    assert(snapshot.deltaTimeUs == 1000U);
    assert(snapshot.sampleRateHz == 200.0f);
    assert(snapshot.calibrationValid);
    assert(snapshot.orientationValid);
    assert(snapshot.controlErrorRad == -0.125f);
    assert(snapshot.angularRateRadPerSec == 0.25f);
    assert(snapshot.deadZoneActive);
    assert(snapshot.proportionalCorrectionUs == 12.5f);
    assert(snapshot.derivativeCorrectionUs == -3.25f);
    assert(snapshot.totalCorrectionUs == 9.25f);
    assert(snapshot.targetPulseUs == 1475);

    std::puts("recordFault tests passed");
    return 0;
}
