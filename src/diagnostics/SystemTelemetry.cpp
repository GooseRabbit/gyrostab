#include "SystemTelemetry.h"

#include <Arduino.h>

namespace gyrostab
{
namespace
{

void printCsvText(const char* value)
{
    Serial.print('"');
    if (value != nullptr)
    {
        for (const char* character = value; *character != '\0'; ++character)
        {
            if (*character == '"')
            {
                Serial.print('"');
            }

            if (*character == '\r' || *character == '\n')
            {
                Serial.print(' ');
            }
            else
            {
                Serial.print(*character);
            }
        }
    }
    Serial.print('"');
}

} // namespace

void publishTelemetry(const TelemetrySnapshot& snapshot)
{
    // timestamp_us,dt_us,sample_rate_hz,calibration_valid,orientation_valid,
    // control_error_rad,angular_rate_rad_s,dead_zone_active,p_term_us,d_term_us,
    // total_correction_us,target_pulse_us,safety_state,fault_code,fault_reason.
    Serial.print(snapshot.timestampUs);
    Serial.print(',');
    Serial.print(snapshot.deltaTimeUs);
    Serial.print(',');
    Serial.print(snapshot.sampleRateHz, 6);
    Serial.print(',');
    Serial.print(snapshot.calibrationValid ? 1 : 0);
    Serial.print(',');
    Serial.print(snapshot.orientationValid ? 1 : 0);
    Serial.print(',');
    Serial.print(snapshot.controlErrorRad, 6);
    Serial.print(',');
    Serial.print(snapshot.angularRateRadPerSec, 6);
    Serial.print(',');
    Serial.print(snapshot.deadZoneActive ? 1 : 0);
    Serial.print(',');
    Serial.print(snapshot.proportionalCorrectionUs, 6);
    Serial.print(',');
    Serial.print(snapshot.derivativeCorrectionUs, 6);
    Serial.print(',');
    Serial.print(snapshot.totalCorrectionUs, 6);
    Serial.print(',');
    Serial.print(snapshot.targetPulseUs);
    Serial.print(',');
    Serial.print(static_cast<uint32_t>(snapshot.safetyState));
    Serial.print(',');
    Serial.print(static_cast<uint32_t>(snapshot.faultCode));
    Serial.print(',');
    printCsvText(snapshot.faultReason);
    Serial.print('\n');
}

void recordFault(TelemetrySnapshot& snapshot, uint16_t faultCode, const char* faultReason)
{
    snapshot.faultCode = faultCode;
    snapshot.faultReason = faultReason;
}

} // namespace gyrostab
