#include "../src/control/ControlStep.h"
#include "../src/control/ControlError.h"
#include "../src/control/DampingRate.h"
#include "../src/control/PdCorrection.h"
#include "../src/control/ControllerOutputValidation.h"
#include "../src/control/ServoCommandLimits.h"
#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdio>
#include <cstdint>
#include <limits>

namespace
{

struct TestHooks
{
    bool errorResult;
    bool rateResult;
    bool pdResult;
    bool validationResult;
    bool limitsResult;
    float errorRad;
    float rateRadPerSec;
    float correctionUs;
    int limitedPulseUs;
    int errorCalls;
    int rateCalls;
    int pdCalls;
    int validationCalls;
    int limitsCalls;
    float receivedErrorRad;
    float receivedRateRadPerSec;
    float receivedKp;
    float receivedKd;
    float receivedCorrectionUs;
    int receivedCurrentPulseUs;
    uint32_t receivedElapsedTimeUs;
};

TestHooks hooks = {};

void resetHooks()
{
    const TestHooks defaults = {
        true, true, true, true, true,
        0.25f, 0.5f, 75.0f, 1575,
        0, 0, 0, 0, 0,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0U
    };
    hooks = defaults;
}

gyrostab::ImuSample makeSample(uint32_t timestampUs, uint32_t sequence)
{
    gyrostab::ImuSample sample = {};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.angularVelocityRadPerSec = {1.0f, 2.0f, 3.0f};
    return sample;
}

gyrostab::OrientationEstimate makeOrientation(const gyrostab::ImuSample& sample)
{
    gyrostab::OrientationEstimate estimate = {};
    estimate.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    estimate.timestampUs = sample.timestampUs;
    estimate.sequence = sample.sequence;
    estimate.valid = true;
    return estimate;
}

gyrostab::ReferenceOrientation makeReference()
{
    gyrostab::ReferenceOrientation reference = {};
    reference.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    reference.valid = true;
    return reference;
}

gyrostab::ControllerConfig makeConfig()
{
    gyrostab::ControllerConfig config = {};
    config.controlAxis.axisInSensorFrame = {0.0f, 0.0f, 1.0f};
    config.gyroBiasRadPerSec = {0.01f, -0.02f, 0.03f};
    config.maximumNormDeviation = 0.01f;
    config.maximumSampleAgeUs = 100U;
    config.proportionalGainUsPerRad = 100.0f;
    config.derivativeGainUsPerRadPerSec = 10.0f;
    config.servoCenterUs = 1500;
    config.mechanicalDirection = 1;
    config.minimumPulseUs = 1000;
    config.maximumPulseUs = 2000;
    config.maximumChangeUsPerSecond = 500.0f;
    return config;
}

void testSuccessfulStepRunsAllStagesAndCommitsState()
{
    resetHooks();
    const gyrostab::ImuSample sample = makeSample(1000U, 7U);
    const gyrostab::OrientationEstimate orientation = makeOrientation(sample);
    const gyrostab::ReferenceOrientation reference = makeReference();
    const gyrostab::ControllerConfig config = makeConfig();
    gyrostab::ControllerState state = {};
    int targetPulseUs = -123;

    assert(gyrostab::runControlStep(orientation, reference, sample, config,
                                    1050U, 1500, 20000U, state, targetPulseUs));
    assert(targetPulseUs == hooks.limitedPulseUs);
    assert(state.hasProcessedSample);
    assert(state.lastProcessedSampleSequence == sample.sequence);
    assert(hooks.errorCalls == 1);
    assert(hooks.rateCalls == 1);
    assert(hooks.pdCalls == 1);
    assert(hooks.validationCalls == 1);
    assert(hooks.limitsCalls == 1);
    assert(hooks.receivedErrorRad == hooks.errorRad);
    assert(hooks.receivedRateRadPerSec == hooks.rateRadPerSec);
    assert(hooks.receivedKp == config.proportionalGainUsPerRad);
    assert(hooks.receivedKd == config.derivativeGainUsPerRadPerSec);
    assert(hooks.receivedCorrectionUs == hooks.correctionUs);
    assert(hooks.receivedCurrentPulseUs == 1500);
    assert(hooks.receivedElapsedTimeUs == 20000U);
}

void testRejectsStaleFutureInvalidAndMismatchedSamples()
{
    resetHooks();
    const gyrostab::ReferenceOrientation reference = makeReference();
    const gyrostab::ControllerConfig config = makeConfig();
    gyrostab::ControllerState state = {};
    int targetPulseUs = 222;

    gyrostab::ImuSample sample = makeSample(1000U, 1U);
    gyrostab::OrientationEstimate orientation = makeOrientation(sample);
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1101U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 222);

    sample = makeSample(1001U, 2U);
    orientation = makeOrientation(sample);
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));

    sample = makeSample(1000U, 3U);
    sample.gyroscopeValid = false;
    orientation = makeOrientation(sample);
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));

    sample = makeSample(1000U, 4U);
    orientation = makeOrientation(sample);
    orientation.valid = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));

    sample = makeSample(1000U, 5U);
    orientation = makeOrientation(sample);
    orientation.sequence = 4U;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));

    assert(targetPulseUs == 222);
    assert(!state.hasProcessedSample);
    assert(hooks.errorCalls == 0);
}

void testRejectsRepeatedAndOutOfOrderSequencesButAllowsRollover()
{
    resetHooks();
    const gyrostab::ReferenceOrientation reference = makeReference();
    const gyrostab::ControllerConfig config = makeConfig();
    int targetPulseUs = 456;

    gyrostab::ControllerState state = {true, 10U};
    gyrostab::ImuSample sample = makeSample(1000U, 10U);
    gyrostab::OrientationEstimate orientation = makeOrientation(sample);
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 456);
    assert(state.lastProcessedSampleSequence == 10U);

    sample = makeSample(1001U, 9U);
    orientation = makeOrientation(sample);
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1001U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 456);

    state = {true, std::numeric_limits<uint32_t>::max()};
    sample = makeSample(1002U, 0U);
    orientation = makeOrientation(sample);
    assert(gyrostab::runControlStep(orientation, reference, sample, config,
                                    1002U, 1500, 1000U, state, targetPulseUs));
    assert(state.lastProcessedSampleSequence == 0U);
}

void testEveryFailedControlStagePreservesOutputAndState()
{
    const gyrostab::ImuSample sample = makeSample(1000U, 1U);
    const gyrostab::OrientationEstimate orientation = makeOrientation(sample);
    const gyrostab::ReferenceOrientation reference = makeReference();
    const gyrostab::ControllerConfig config = makeConfig();
    gyrostab::ControllerState state = {};
    int targetPulseUs = 777;

    resetHooks();
    hooks.errorResult = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 777 && !state.hasProcessedSample);
    assert(hooks.errorCalls == 1 && hooks.rateCalls == 0);

    resetHooks();
    hooks.rateResult = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 777 && !state.hasProcessedSample);
    assert(hooks.errorCalls == 1 && hooks.rateCalls == 1 && hooks.pdCalls == 0);

    resetHooks();
    hooks.pdResult = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 777 && !state.hasProcessedSample);
    assert(hooks.validationCalls == 0 && hooks.limitsCalls == 0);

    resetHooks();
    hooks.validationResult = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 777 && !state.hasProcessedSample);
    assert(hooks.validationCalls == 1 && hooks.limitsCalls == 0);

    resetHooks();
    hooks.limitsResult = false;
    assert(!gyrostab::runControlStep(orientation, reference, sample, config,
                                     1000U, 1500, 1000U, state, targetPulseUs));
    assert(targetPulseUs == 777 && !state.hasProcessedSample);
    assert(hooks.limitsCalls == 1);
}

} // namespace

namespace gyrostab
{

bool isSampleFresh(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs)
{
    if (!sample.accelerometerValid || !sample.gyroscopeValid ||
        maximumAgeUs >= 0x80000000UL)
    {
        return false;
    }

    return static_cast<uint32_t>(nowUs - sample.timestampUs) <= maximumAgeUs;
}

bool computeControlError(const OrientationEstimate&, const ReferenceOrientation&,
                         const AxisConfig&, float, float& output)
{
    ++hooks.errorCalls;
    if (!hooks.errorResult)
    {
        return false;
    }

    output = hooks.errorRad;
    return true;
}

bool computeDampingRate(const ImuSample&, const Vector3f&, const AxisConfig&, float, float& output)
{
    ++hooks.rateCalls;
    if (!hooks.rateResult)
    {
        return false;
    }

    output = hooks.rateRadPerSec;
    return true;
}

bool computePdCorrection(float error, float rate, float kp, float kd, float& output)
{
    ++hooks.pdCalls;
    hooks.receivedErrorRad = error;
    hooks.receivedRateRadPerSec = rate;
    hooks.receivedKp = kp;
    hooks.receivedKd = kd;
    if (!hooks.pdResult)
    {
        return false;
    }

    output = hooks.correctionUs;
    return true;
}

bool validateControllerOutput(float correction)
{
    ++hooks.validationCalls;
    hooks.receivedCorrectionUs = correction;
    return hooks.validationResult;
}

bool applyServoCommandLimits(float, int, int, int currentPulse, int, int,
                             float, uint32_t elapsedTime, int& output)
{
    ++hooks.limitsCalls;
    hooks.receivedCurrentPulseUs = currentPulse;
    hooks.receivedElapsedTimeUs = elapsedTime;
    if (!hooks.limitsResult)
    {
        return false;
    }

    output = hooks.limitedPulseUs;
    return true;
}

} // namespace gyrostab

int main()
{
    testSuccessfulStepRunsAllStagesAndCommitsState();
    testRejectsStaleFutureInvalidAndMismatchedSamples();
    testRejectsRepeatedAndOutOfOrderSequencesButAllowsRollover();
    testEveryFailedControlStagePreservesOutputAndState();
    std::puts("runControlStep tests passed");
    return 0;
}
