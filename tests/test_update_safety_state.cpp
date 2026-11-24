#include "../src/safety/SafetyManager.h"

#include <cassert>
#include <cstdio>
#include <cstdint>

namespace
{

gyrostab::SafetyStateInputs makeInputs(bool initialized = false,
                                       bool calibrated = false,
                                       bool activationRequested = false)
{
    gyrostab::SafetyStateInputs inputs = {};
    inputs.initializationComplete = initialized;
    inputs.calibrationComplete = calibrated;
    inputs.activationRequested = activationRequested;
    return inputs;
}

void testBootAdvancesOnlyToInitializing()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Boot;

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, true)));
    assert(state == gyrostab::SafetyState::Initializing);
}

void testInitializationWaitsForCompletion()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Initializing;

    assert(gyrostab::updateSafetyState(state, makeInputs(false)));
    assert(state == gyrostab::SafetyState::Initializing);

    assert(gyrostab::updateSafetyState(state, makeInputs(true)));
    assert(state == gyrostab::SafetyState::Calibrating);
}

void testCalibrationWaitsForCompletion()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Calibrating;

    assert(gyrostab::updateSafetyState(state, makeInputs(true, false)));
    assert(state == gyrostab::SafetyState::Calibrating);

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true)));
    assert(state == gyrostab::SafetyState::Ready);
}

void testReadyRequiresAnExplicitActivationRequest()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Ready;

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, false)));
    assert(state == gyrostab::SafetyState::Ready);

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, true)));
    assert(state == gyrostab::SafetyState::Active);
}

void testActiveReturnsToReadyWhenRequestIsRemoved()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Active;

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, true)));
    assert(state == gyrostab::SafetyState::Active);

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, false)));
    assert(state == gyrostab::SafetyState::Ready);
}

void testFaultRemainsLatched()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Fault;

    assert(gyrostab::updateSafetyState(state, makeInputs(true, true, true)));
    assert(state == gyrostab::SafetyState::Fault);

    assert(gyrostab::updateSafetyState(state, makeInputs(false, false, false)));
    assert(state == gyrostab::SafetyState::Fault);
}

void testRejectsUnknownStateWithoutChangingIt()
{
    const gyrostab::SafetyState invalidState =
        static_cast<gyrostab::SafetyState>(UINT8_MAX);
    gyrostab::SafetyState state = invalidState;

    assert(!gyrostab::updateSafetyState(state, makeInputs(true, true, true)));
    assert(state == invalidState);
}

} // namespace

int main()
{
    testBootAdvancesOnlyToInitializing();
    testInitializationWaitsForCompletion();
    testCalibrationWaitsForCompletion();
    testReadyRequiresAnExplicitActivationRequest();
    testActiveReturnsToReadyWhenRequestIsRemoved();
    testFaultRemainsLatched();
    testRejectsUnknownStateWithoutChangingIt();

    std::puts("updateSafetyState tests passed");
    return 0;
}
