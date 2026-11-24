#include "ServoCommandLimits.h"

#include "../servo/ServoDriver.h"
#include "ControllerOutputValidation.h"

#include <cmath>
#include <stdint.h>
#include <limits>

namespace gyrostab
{

bool applyServoCommandLimits(float correctionUs,
                             int servoCenterUs,
                             int direction,
                             int currentPulseUs,
                             int minimumPulseUs,
                             int maximumPulseUs,
                             float maximumChangeUsPerSecond,
                             uint32_t elapsedTimeUs,
                             int& limitedPulseUs)
{
    if ((direction != 1 && direction != -1) ||
        !validateControllerOutput(correctionUs))
    {
        return false;
    }

    int validatedCenterPulseUs = 0;
    if (!limitServoPulse(servoCenterUs, minimumPulseUs, maximumPulseUs,
                         validatedCenterPulseUs) ||
        validatedCenterPulseUs != servoCenterUs)
    {
        return false;
    }

    int validatedCurrentPulseUs = 0;
    if (!limitServoPulse(currentPulseUs, minimumPulseUs, maximumPulseUs,
                         validatedCurrentPulseUs) ||
        validatedCurrentPulseUs != currentPulseUs)
    {
        return false;
    }

    // PWM API принимает целые микросекунды; округляем поправку до ближайшего
    // целого значения, затем складываем с центром в расширенном типе.
    const double roundedCorrection = std::round(static_cast<double>(correctionUs));
    if (!std::isfinite(roundedCorrection) ||
        roundedCorrection < static_cast<double>(std::numeric_limits<int>::lowest()) ||
        roundedCorrection > static_cast<double>(std::numeric_limits<int>::max()))
    {
        return false;
    }

    const int64_t signedCorrection =
        static_cast<int64_t>(static_cast<int>(roundedCorrection));
    const int64_t rawTargetPulseUs =
        static_cast<int64_t>(servoCenterUs) +
        static_cast<int64_t>(direction) * signedCorrection;

    // Сначала безопасно сужаем расширенный результат до int, затем штатный
    // limitServoPulse() применяет механические min/max из конфигурации.
    int requestedPulseUs = 0;
    if (rawTargetPulseUs > static_cast<int64_t>(std::numeric_limits<int>::max()))
    {
        requestedPulseUs = std::numeric_limits<int>::max();
    }
    else if (rawTargetPulseUs < static_cast<int64_t>(std::numeric_limits<int>::lowest()))
    {
        requestedPulseUs = std::numeric_limits<int>::lowest();
    }
    else
    {
        requestedPulseUs = static_cast<int>(rawTargetPulseUs);
    }

    int positionLimitedPulseUs = 0;
    if (!limitServoPulse(requestedPulseUs, minimumPulseUs, maximumPulseUs,
                         positionLimitedPulseUs))
    {
        return false;
    }

    int candidatePulseUs = 0;
    if (!limitServoSlewRate(currentPulseUs, positionLimitedPulseUs,
                            maximumChangeUsPerSecond, elapsedTimeUs,
                            candidatePulseUs))
    {
        return false;
    }

    // Текущая команда и цель уже внутри min/max; slew limiter двигает
    // только между ними, поэтому результат также остаётся в безопасном диапазоне.
    limitedPulseUs = candidatePulseUs;
    return true;
}

} // namespace gyrostab
