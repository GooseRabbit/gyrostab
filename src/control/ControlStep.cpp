#include "ControlStep.h"

#include "ControlError.h"
#include "ControllerOutputValidation.h"
#include "DampingRate.h"
#include "PdCorrection.h"
#include "ServoCommandLimits.h"
#include "../sensors/SensorManager.h"

namespace gyrostab
{

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

    float rateRadPerSec = 0.0f;
    if (!computeDampingRate(sample, config.gyroBiasRadPerSec,
                            config.controlAxis, config.maximumNormDeviation,
                            rateRadPerSec))
    {
        return false;
    }

    float correctionUs = 0.0f;
    if (!computePdCorrection(errorRad, rateRadPerSec,
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
    limitedPulseUs = candidatePulseUs;
    state.lastProcessedSampleSequence = sample.sequence;
    state.hasProcessedSample = true;
    return true;
}

} // namespace gyrostab
