#pragma once

#include "../safety/SafetyManager.h"

#include <stdint.h>

namespace gyrostab
{

// Подготовленный вызывающим кодом диагностический снимок одного момента времени.
// Сырые IMU и quaternion передаются отдельными publishRawImuTelemetry()/publishOrientationTelemetry().
struct TelemetrySnapshot
{
    uint32_t timestampUs;
    uint32_t deltaTimeUs;
    float sampleRateHz;
    bool calibrationValid;
    bool orientationValid;
    float controlErrorRad;
    float angularRateRadPerSec;
    bool deadZoneActive;
    float proportionalCorrectionUs;
    float derivativeCorrectionUs;
    float totalCorrectionUs;
    int targetPulseUs;
    SafetyState safetyState;
    uint16_t faultCode;
    const char* faultReason;
};

// Выводит один агрегированный CSV-рекорд; не читает датчики и не вычисляет метрики.
// faultReason используется только во время вызова; nullptr выводится как пустое поле.
// Частоту вызова определяет вызывающий код.
void publishTelemetry(const TelemetrySnapshot& snapshot);

} // namespace gyrostab
