#include "OrientationTelemetry.h"

#include <Arduino.h>

namespace gyrostab
{

void publishOrientationTelemetry(const OrientationEstimate& estimate)
{
    // timestamp_us, sequence, valid, quaternion_w, quaternion_x, quaternion_y, quaternion_z.
    Serial.print(estimate.timestampUs);
    Serial.print(',');
    Serial.print(estimate.sequence);
    Serial.print(',');
    Serial.print(estimate.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(estimate.quaternion.w, 6);
    Serial.print(',');
    Serial.print(estimate.quaternion.x, 6);
    Serial.print(',');
    Serial.print(estimate.quaternion.y, 6);
    Serial.print(',');
    Serial.println(estimate.quaternion.z, 6);
}

} // namespace gyrostab
