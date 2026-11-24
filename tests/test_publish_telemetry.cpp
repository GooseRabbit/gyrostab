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

void MockSerialClass::clear()
{
    output.clear();
}

int main()
{
    using namespace gyrostab;

    TelemetrySnapshot snapshot = {
        123456U, 1000U, 200.0f, true, true, -0.125f, 0.25f, true,
        12.5f, -3.25f, 9.25f, 1475, SafetyState::Active, 7U,
        "IMU timeout, gyro \"dropout\""
    };
    const TelemetrySnapshot saved = snapshot;

    publishTelemetry(snapshot);
    assert(Serial.output ==
           "123456,1000,200.000000,1,1,-0.125000,0.250000,1,12.500000,-3.250000,9.250000,1475,4,7,\"IMU timeout, gyro \"\"dropout\"\"\"\n");

    // Повторный снимок с пустой причиной корректно выводит пустое CSV-поле.
    Serial.clear();
    snapshot.faultReason = nullptr;
    snapshot.safetyState = SafetyState::Fault;
    snapshot.faultCode = 0U;
    publishTelemetry(snapshot);
    assert(Serial.output ==
           "123456,1000,200.000000,1,1,-0.125000,0.250000,1,12.500000,-3.250000,9.250000,1475,5,0,\"\"\n");

    // Неизвестное состояние сохраняется числовым кодом; переносы строк не ломают CSV-запись.
    Serial.clear();
    snapshot.safetyState = static_cast<SafetyState>(UINT8_MAX);
    snapshot.faultReason = "line one\nline two\r\nline three";
    publishTelemetry(snapshot);
    assert(Serial.output.find(",255,0,\"line one line two  line three\"\n") != std::string::npos);

    // Исходные диагностические значения не изменяются.
    assert(snapshot.timestampUs == saved.timestampUs);
    assert(snapshot.deltaTimeUs == saved.deltaTimeUs);
    assert(snapshot.sampleRateHz == saved.sampleRateHz);
    assert(snapshot.calibrationValid == saved.calibrationValid);
    assert(snapshot.orientationValid == saved.orientationValid);
    assert(snapshot.controlErrorRad == saved.controlErrorRad);
    assert(snapshot.angularRateRadPerSec == saved.angularRateRadPerSec);
    assert(snapshot.deadZoneActive == saved.deadZoneActive);
    assert(snapshot.proportionalCorrectionUs == saved.proportionalCorrectionUs);
    assert(snapshot.derivativeCorrectionUs == saved.derivativeCorrectionUs);
    assert(snapshot.totalCorrectionUs == saved.totalCorrectionUs);
    assert(snapshot.targetPulseUs == saved.targetPulseUs);
    assert(saved.safetyState == SafetyState::Active);
    assert(saved.faultCode == 7U);
    assert(saved.faultReason != nullptr);

    std::puts("publishTelemetry tests passed");
    return 0;
}
