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

// Caller-owned накопительные счётчики диагностических событий.
// Приращения событий определяет и передаёт вызывающий код.
struct DiagnosticCounters
{
    uint32_t newPackets;
    uint32_t spiErrors;
    uint32_t missedSamples;
    uint32_t timeouts;
};

// Выводит один агрегированный CSV-рекорд; не читает датчики и не вычисляет метрики.
// Если faultReason сохранён через recordFault(), вызывающий код сохраняет строку
// действительной до публикации снимка; nullptr выводится как пустое поле.
// Частоту вызова определяет вызывающий код.
void publishTelemetry(const TelemetrySnapshot& snapshot);

// Сохраняет код и указатель на причину в caller-owned снимке.
// Строка принадлежит вызывающему коду и должна оставаться действительной,
// пока снимок может быть опубликован; nullptr очищает указатель.
// Не переключает SafetyState и не выполняет Serial-вывод.
void recordFault(TelemetrySnapshot& snapshot, uint16_t faultCode, const char* faultReason);

// Добавляет явно переданные приращения к caller-owned диагностическим счётчикам.
// События не определяются автоматически; переполнение uint32_t происходит по модулю 2^32.
void updateCounters(DiagnosticCounters& counters, uint32_t newPackets,
                    uint32_t spiErrors, uint32_t missedSamples, uint32_t timeouts);

} // namespace gyrostab
