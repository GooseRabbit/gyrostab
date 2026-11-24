#include "SafetyManager.h"

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

} // namespace gyrostab
