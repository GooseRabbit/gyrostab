#include "ServoDriver.h"

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

} // namespace gyrostab
