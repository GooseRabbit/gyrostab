#include "RawImuTelemetry.h"

#include <Arduino.h>

namespace gyrostab
{

void publishRawImuTelemetry(const ImuSample& sample)
{
    // timestamp_us, sequence, accel_valid, accel_xyz_mps2,
    // gyro_valid, gyro_xyz_rad_s, mag_valid, mag_xyz_uT.
    Serial.print(sample.timestampUs);
    Serial.print(',');
    Serial.print(sample.sequence);
    Serial.print(',');
    Serial.print(sample.accelerometerValid ? 1 : 0);
    Serial.print(',');
    Serial.print(sample.accelerationMps2.x, 6);
    Serial.print(',');
    Serial.print(sample.accelerationMps2.y, 6);
    Serial.print(',');
    Serial.print(sample.accelerationMps2.z, 6);
    Serial.print(',');
    Serial.print(sample.gyroscopeValid ? 1 : 0);
    Serial.print(',');
    Serial.print(sample.angularVelocityRadPerSec.x, 6);
    Serial.print(',');
    Serial.print(sample.angularVelocityRadPerSec.y, 6);
    Serial.print(',');
    Serial.print(sample.angularVelocityRadPerSec.z, 6);
    Serial.print(',');
    Serial.print(sample.magnetometerValid ? 1 : 0);
    Serial.print(',');
    Serial.print(sample.magneticFieldMicroTesla.x, 6);
    Serial.print(',');
    Serial.print(sample.magneticFieldMicroTesla.y, 6);
    Serial.print(',');
    Serial.println(sample.magneticFieldMicroTesla.z, 6);
}

} // namespace gyrostab
