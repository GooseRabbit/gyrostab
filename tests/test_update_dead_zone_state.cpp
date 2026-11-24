#include "../src/control/DeadZone.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

void testEntersAtOrInsideEnterThreshold()
{
    gyrostab::DeadZoneState state = {false};

    assert(gyrostab::updateDeadZoneState(0.5f, 0.5f, 0.8f, state));
    assert(state.active);

    state.active = false;
    assert(gyrostab::updateDeadZoneState(0.4f, 0.5f, 0.8f, state));
    assert(state.active);
}

void testRemainsInactiveOutsideEnterThreshold()
{
    gyrostab::DeadZoneState state = {false};

    assert(gyrostab::updateDeadZoneState(0.5001f, 0.5f, 0.8f, state));
    assert(!state.active);
}

void testHysteresisRetainsActiveStateUntilAboveExitThreshold()
{
    gyrostab::DeadZoneState state = {true};

    assert(gyrostab::updateDeadZoneState(0.5f, 0.5f, 0.8f, state));
    assert(state.active);

    assert(gyrostab::updateDeadZoneState(0.8f, 0.5f, 0.8f, state));
    assert(state.active);

    assert(gyrostab::updateDeadZoneState(0.8001f, 0.5f, 0.8f, state));
    assert(!state.active);
}

void testCanEnterAgainAfterLeaving()
{
    gyrostab::DeadZoneState state = {true};

    assert(gyrostab::updateDeadZoneState(0.9f, 0.5f, 0.8f, state));
    assert(!state.active);

    assert(gyrostab::updateDeadZoneState(0.5f, 0.5f, 0.8f, state));
    assert(state.active);
}

void testRejectsInvalidValuesWithoutChangingState()
{
    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    gyrostab::DeadZoneState state = {true};
    assert(!gyrostab::updateDeadZoneState(-0.1f, 0.5f, 0.8f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, -0.1f, 0.8f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, 0.5f, 0.5f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, 0.6f, 0.5f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(nanValue, 0.5f, 0.8f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, infinity, 0.8f, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, 0.5f, nanValue, state));
    assert(state.active);
    assert(!gyrostab::updateDeadZoneState(0.2f, 0.5f, infinity, state));
    assert(state.active);

}

} // namespace

int main()
{
    testEntersAtOrInsideEnterThreshold();
    testRemainsInactiveOutsideEnterThreshold();
    testHysteresisRetainsActiveStateUntilAboveExitThreshold();
    testCanEnterAgainAfterLeaving();
    testRejectsInvalidValuesWithoutChangingState();

    std::puts("updateDeadZoneState tests passed");
    return 0;
}
