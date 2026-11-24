#include "ControlStep.h"

#include <cmath>

#include "ControlError.h"
#include "ControllerOutputValidation.h"
#include "DampingRate.h"
#include "PdCorrection.h"
#include "ServoCommandLimits.h"
#include "../sensors/SensorManager.h"

namespace gyrostab
{

void resetController(ControllerState& state)
{
    state.hasProcessedSample = false;
    state.lastProcessedSampleSequence = 0U;
    resetDeadZone(state.deadZone);
}

bool runControlStep(const OrientationEstimate& currentOrientation,
                    const ReferenceOrientation& referenceOrientation,
                    const ImuSample& sample,
                    const ControllerConfig& config,
                    uint32_t nowUs,
                    int currentPulseUs,
                    uint32_t elapsedTimeUs,
                    ControllerState& state,
                    int& limitedPulseUs)
{
    const uint32_t halfRange = 0x80000000UL;
    if (!isSampleFresh(sample, nowUs, config.maximumSampleAgeUs) ||
        !currentOrientation.valid ||
        currentOrientation.sequence != sample.sequence ||
        currentOrientation.timestampUs != sample.timestampUs)
    {
        return false;
    }

    if (state.hasProcessedSample)
    {
        const uint32_t sequenceDelta = sample.sequence - state.lastProcessedSampleSequence;
        if (sequenceDelta == 0U || sequenceDelta >= halfRange)
        {
            return false;
        }
    }

    float errorRad = 0.0f;
    if (!computeControlError(currentOrientation, referenceOrientation,
                             config.controlAxis, config.maximumNormDeviation,
                             errorRad))
    {
        return false;
    }

    // Гистерезис меняется в локальной копии и фиксируется только после
    // успешного завершения всей цепочки расчёта.
    ControllerState candidateState = state;
    if (!updateDeadZoneState(std::fabs(errorRad), config.deadZoneEnterThresholdRad,
                             config.deadZoneExitThresholdRad, candidateState.deadZone))
    {
        return false;
    }

    float positionalErrorRad = 0.0f;
    if (!applyDeadZone(errorRad, candidateState.deadZone, positionalErrorRad))
    {
        return false;
    }

    float rateRadPerSec = 0.0f;
    if (!computeDampingRate(sample, config.gyroBiasRadPerSec,
                            config.controlAxis, config.maximumNormDeviation,
                            rateRadPerSec))
    {
        return false;
    }

    float correctionUs = 0.0f;
    if (!computePdCorrection(positionalErrorRad, rateRadPerSec,
                             config.proportionalGainUsPerRad,
                             config.derivativeGainUsPerRadPerSec,
                             correctionUs) ||
        !validateControllerOutput(correctionUs))
    {
        return false;
    }

    int candidatePulseUs = 0;
    if (!applyServoCommandLimits(correctionUs, config.servoCenterUs,
                                 config.mechanicalDirection, currentPulseUs,
                                 config.minimumPulseUs, config.maximumPulseUs,
                                 config.maximumChangeUsPerSecond, elapsedTimeUs,
                                 candidatePulseUs))
    {
        return false;
    }

    // Публикуем результат и помечаем sample обработанным только после того,
    // как вся вычислительная цепочка завершилась успешно.
    candidateState.lastProcessedSampleSequence = sample.sequence;
    candidateState.hasProcessedSample = true;
    limitedPulseUs = candidatePulseUs;
    state = candidateState;
    return true;
}

} // namespace gyrostab
