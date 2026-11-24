#include "../src/control/DeadZone.h"

#include <cassert>
#include <cstdio>
#include <limits>

namespace
{

void testPassesSignedErrorWhenInactive()
{
    const gyrostab::DeadZoneState state = {false};
    float outputErrorRad = 12.0f;

    assert(gyrostab::applyDeadZone(0.25f, state, outputErrorRad));
    assert(outputErrorRad == 0.25f);

    assert(gyrostab::applyDeadZone(-0.75f, state, outputErrorRad));
    assert(outputErrorRad == -0.75f);
}

void testSuppressesPositionErrorWhenActive()
{
    const gyrostab::DeadZoneState state = {true};
    float outputErrorRad = 12.0f;

    assert(gyrostab::applyDeadZone(0.25f, state, outputErrorRad));
    assert(outputErrorRad == 0.0f);

    assert(gyrostab::applyDeadZone(-0.75f, state, outputErrorRad));
    assert(outputErrorRad == 0.0f);
}

void testPassesZeroWhenInactiveAndSuppressesZeroWhenActive()
{
    const gyrostab::DeadZoneState inactiveState = {false};
    const gyrostab::DeadZoneState activeState = {true};
    float outputErrorRad = 5.0f;

    assert(gyrostab::applyDeadZone(0.0f, inactiveState, outputErrorRad));
    assert(outputErrorRad == 0.0f);

    outputErrorRad = 5.0f;
    assert(gyrostab::applyDeadZone(0.0f, activeState, outputErrorRad));
    assert(outputErrorRad == 0.0f);
}

void testRejectsNonFiniteErrorWithoutChangingOutputOrState()
{
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    gyrostab::DeadZoneState state = {true};
    const bool originalState = state.active;
    float outputErrorRad = 42.0f;

    assert(!gyrostab::applyDeadZone(nanValue, state, outputErrorRad));
    assert(outputErrorRad == 42.0f);
    assert(state.active == originalState);

    assert(!gyrostab::applyDeadZone(infinity, state, outputErrorRad));
    assert(outputErrorRad == 42.0f);
    assert(state.active == originalState);
}

void testDoesNotModifyState()
{
    gyrostab::DeadZoneState inactiveState = {false};
    gyrostab::DeadZoneState activeState = {true};
    float outputErrorRad = 0.0f;

    assert(gyrostab::applyDeadZone(-0.3f, inactiveState, outputErrorRad));
    assert(!inactiveState.active);
    assert(gyrostab::applyDeadZone(0.3f, activeState, outputErrorRad));
    assert(activeState.active);
}

} // namespace

int main()
{
    testPassesSignedErrorWhenInactive();
    testSuppressesPositionErrorWhenActive();
    testPassesZeroWhenInactiveAndSuppressesZeroWhenActive();
    testRejectsNonFiniteErrorWithoutChangingOutputOrState();
    testDoesNotModifyState();

    std::puts("applyDeadZone tests passed");
    return 0;
}
