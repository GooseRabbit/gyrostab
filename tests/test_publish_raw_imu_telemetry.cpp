#include "../src/diagnostics/RawImuTelemetry.h"
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

    ImuSample sample = {};
    sample.timestampUs = 123456U;
    sample.sequence = 42U;
    sample.accelerometerValid = true;
    sample.accelerationMps2 = {1.25f, -2.5f, 9.80665f};
    sample.gyroscopeValid = true;
    sample.angularVelocityRadPerSec = {0.125f, -0.25f, 1.5f};
    sample.magnetometerValid = true;
    sample.magneticFieldMicroTesla = {24.5f, -11.25f, 38.75f};

    publishRawImuTelemetry(sample);
    const std::string expected =
        "123456,42,1,1.250000,-2.500000,9.806650,1,0.125000,-0.250000,1.500000,1,24.500000,-11.250000,38.750000\n";
    assert(Serial.output == expected);

    // Невалидность канала явно передаётся флагом, сама функция образец не меняет.
    Serial.clear();
    sample.magnetometerValid = false;
    const uint32_t originalTimestamp = sample.timestampUs;
    const uint32_t originalSequence = sample.sequence;
    publishRawImuTelemetry(sample);
    assert(Serial.output.find(",0,24.500000,-11.250000,38.750000\n") != std::string::npos);
    assert(sample.timestampUs == originalTimestamp);
    assert(sample.sequence == originalSequence);
    assert(!sample.magnetometerValid);

    std::puts("publishRawImuTelemetry tests passed");
    return 0;
}
