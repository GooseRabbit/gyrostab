#pragma once

#include <ESP32Servo.h>

namespace gyrostab
{

// Настраивает период PWM и подключает Servo к явно заданному GPIO.
// Частота и импульсные пределы передаются вызывающим кодом.
// Начальная команда положения не отправляется.
bool beginServo(Servo& servo,
                int signalPin,
                int refreshRateHz,
                int minimumPulseUs,
                int maximumPulseUs);

// Сохраняет исходный запрос; ограничение импульса выполняется отдельно.
void setServoTarget(int requestedPulseUs,
                    int& pendingTargetPulseUs);

// Ограничивает запрос переданными безопасными min/max импульсами.
// При неверных границах возвращает false и не меняет выход.
bool limitServoPulse(int requestedPulseUs,
                     int minimumPulseUs,
                     int maximumPulseUs,
                     int& limitedPulseUs);

} // namespace gyrostab
