#include "ServoDriver.h"

#include <cmath>
#include <stdint.h>

namespace gyrostab
{

bool beginServo(Servo& servo,
                int signalPin,
                int refreshRateHz,
                int minimumPulseUs,
                int maximumPulseUs)
{
    if (servo.attached() || signalPin < 0 || refreshRateHz <= 0 ||
        minimumPulseUs < MIN_PULSE_WIDTH ||
        maximumPulseUs > MAX_PULSE_WIDTH ||
        minimumPulseUs >= maximumPulseUs)
    {
        return false;
    }

    // ESP32Servo::setPeriodHertz() не возвращает код ошибки.
    // Частоту выбирает вызывающий код после проверки целевой платы и сервы.
    servo.setPeriodHertz(refreshRateHz);

    // Нельзя использовать возвращаемый канал как bool: канал 0 допустим.
    servo.attach(signalPin, minimumPulseUs, maximumPulseUs);

    // Проверяется состояние объекта ESP32Servo, но не форма реального PWM.
    return servo.attached();
}

void setServoTarget(int requestedPulseUs,
                    int& pendingTargetPulseUs)
{
    pendingTargetPulseUs = requestedPulseUs;
}

bool limitServoPulse(int requestedPulseUs,
                     int minimumPulseUs,
                     int maximumPulseUs,
                     int& limitedPulseUs)
{
    if (minimumPulseUs < MIN_PULSE_WIDTH ||
        maximumPulseUs > MAX_PULSE_WIDTH ||
        minimumPulseUs >= maximumPulseUs)
    {
        return false;
    }

    int candidatePulseUs = requestedPulseUs;
    if (candidatePulseUs < minimumPulseUs)
    {
        candidatePulseUs = minimumPulseUs;
    }
    else if (candidatePulseUs > maximumPulseUs)
    {
        candidatePulseUs = maximumPulseUs;
    }

    limitedPulseUs = candidatePulseUs;
    return true;
}

bool limitServoSlewRate(int currentPulseUs,
                        int targetPulseUs,
                        float maximumChangeUsPerSecond,
                        uint32_t elapsedTimeUs,
                        int& limitedPulseUs)
{
    if (!std::isfinite(maximumChangeUsPerSecond) ||
        maximumChangeUsPerSecond < 0.0f)
    {
        return false;
    }

    const int64_t deltaPulseUs = static_cast<int64_t>(targetPulseUs) -
                                 static_cast<int64_t>(currentPulseUs);
    if (deltaPulseUs == 0)
    {
        limitedPulseUs = targetPulseUs;
        return true;
    }

    const double maximumChange =
        static_cast<double>(maximumChangeUsPerSecond) *
        static_cast<double>(elapsedTimeUs) / 1000000.0;
    const double absoluteDelta = deltaPulseUs >= 0
                                     ? static_cast<double>(deltaPulseUs)
                                     : -static_cast<double>(deltaPulseUs);

    if (!std::isfinite(maximumChange) || maximumChange < 0.0)
    {
        return false;
    }

    if (maximumChange >= absoluteDelta)
    {
        limitedPulseUs = targetPulseUs;
        return true;
    }

    // Округление вниз гарантирует, что предел скорости не будет превышен.
    const int64_t allowedChange = static_cast<int64_t>(std::floor(maximumChange));
    const int64_t candidatePulseUs = static_cast<int64_t>(currentPulseUs) +
                                     (deltaPulseUs > 0 ? allowedChange : -allowedChange);
    limitedPulseUs = static_cast<int>(candidatePulseUs);
    return true;
}


bool writeServoPulse(Servo& servo,
                     int pulseUs,
                     int minimumPulseUs,
                     int maximumPulseUs)
{
    if (!servo.attached() ||
        minimumPulseUs < MIN_PULSE_WIDTH ||
        maximumPulseUs > MAX_PULSE_WIDTH ||
        minimumPulseUs >= maximumPulseUs ||
        pulseUs < minimumPulseUs ||
        pulseUs > maximumPulseUs)
    {
        return false;
    }

    // writeMicroseconds() не возвращает статус физического PWM-сигнала.
    servo.writeMicroseconds(pulseUs);
    return true;
}

bool disableServoOutput(Servo& servo)
{
    if (servo.attached())
    {
        servo.detach();
    }

    // Подтверждается только состояние объекта ESP32Servo, не механическая фиксация.
    return !servo.attached();
}

} // namespace gyrostab
