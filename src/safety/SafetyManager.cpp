#include "SafetyManager.h"

#include "../sensors/SensorManager.h"
#include "../orientation/OrientationEstimator.h"
#include "../servo/ServoDriver.h"

namespace gyrostab
{

bool updateSafetyState(SafetyState& state, const SafetyStateInputs& inputs)
{
    switch (state)
    {
        case SafetyState::Boot:
            state = SafetyState::Initializing;
            return true;

        case SafetyState::Initializing:
            if (inputs.initializationComplete)
            {
                state = SafetyState::Calibrating;
            }
            return true;

        case SafetyState::Calibrating:
            if (inputs.calibrationComplete)
            {
                state = SafetyState::Ready;
            }
            return true;

        case SafetyState::Ready:
            if (inputs.activationRequested)
            {
                state = SafetyState::Active;
            }
            return true;

        case SafetyState::Active:
            if (!inputs.activationRequested)
            {
                state = SafetyState::Ready;
            }
            return true;

        case SafetyState::Fault:
            // Авария остаётся защёлкнутой; автоматическое восстановление запрещено.
            return true;
    }

    return false;
}

bool canRunController(SafetyState state)
{
    return state == SafetyState::Active;
}

bool checkSensorTimeout(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs)
{
    return isSampleFresh(sample, nowUs, maximumAgeUs);
}

bool checkEstimatorHealth(const OrientationEstimate& estimate, uint32_t nowUs,
                          uint32_t maximumAgeUs, float maximumNormDeviation)
{
    // Для однозначного определения возраста интервал должен быть меньше половины uint32_t.
    if (maximumAgeUs >= 0x80000000UL)
    {
        return false;
    }

    // Беззнаковое вычитание корректно учитывает rollover счётчика микросекунд.
    const uint32_t ageUs = nowUs - estimate.timestampUs;
    if (ageUs > maximumAgeUs)
    {
        return false;
    }

    return validateOrientation(estimate, maximumNormDeviation);
}

bool checkCommandLimits(int targetPulseUs, int minimumPulseUs, int maximumPulseUs)
{
    int validatedTargetPulseUs = 0;
    return limitServoPulse(targetPulseUs, minimumPulseUs, maximumPulseUs,
                           validatedTargetPulseUs) &&
           validatedTargetPulseUs == targetPulseUs;
}

void enterFault(SafetyState& state)
{
    // Защёлкивает аварийное состояние; причина и реакция сервопривода обрабатываются отдельно.
    state = SafetyState::Fault;
}

bool applySafeOutputPolicy(SafetyState state, ::Servo& servo,
                           int lastCommandPulseUs, int minimumPulseUs, int maximumPulseUs)
{
    if (state != SafetyState::Fault)
    {
        return false;
    }

    // Сохраняем последнюю принятую команду; не центрируем и не отсоединяем серву.
    return writeServoPulse(servo, lastCommandPulseUs, minimumPulseUs, maximumPulseUs);
}

} // namespace gyrostab
