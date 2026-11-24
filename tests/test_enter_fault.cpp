#include "../src/safety/SafetyManager.h"
#include "../src/sensors/SensorManager.h"
#include "../src/orientation/OrientationEstimator.h"
#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace gyrostab
{

// Заглушки нужны для линковки других функций SafetyManager.cpp; enterFault() их не вызывает.
bool isSampleFresh(const ImuSample&, uint32_t, uint32_t)
{
    return false;
}

bool validateOrientation(const OrientationEstimate&, float)
{
    return false;
}

bool limitServoPulse(int, int, int, int&)
{
    return false;
}

} // namespace gyrostab

namespace
{

void testEveryKnownStateEntersFault()
{
    const gyrostab::SafetyState states[] = {
        gyrostab::SafetyState::Boot,
        gyrostab::SafetyState::Initializing,
        gyrostab::SafetyState::Calibrating,
        gyrostab::SafetyState::Ready,
        gyrostab::SafetyState::Active,
        gyrostab::SafetyState::Fault
    };

    for (unsigned int i = 0; i < sizeof(states) / sizeof(states[0]); ++i)
    {
        gyrostab::SafetyState state = states[i];
        gyrostab::enterFault(state);
        assert(state == gyrostab::SafetyState::Fault);
    }
}

void testUnknownStateFailsSafeToFault()
{
    gyrostab::SafetyState state =
        static_cast<gyrostab::SafetyState>(UINT8_MAX);

    gyrostab::enterFault(state);

    assert(state == gyrostab::SafetyState::Fault);
}

void testFaultRemainsLatched()
{
    gyrostab::SafetyState state = gyrostab::SafetyState::Active;

    gyrostab::enterFault(state);
    gyrostab::enterFault(state);

    assert(state == gyrostab::SafetyState::Fault);

    const gyrostab::SafetyStateInputs inputs = {true, true, true};
    assert(gyrostab::updateSafetyState(state, inputs));
    assert(state == gyrostab::SafetyState::Fault);
    assert(!gyrostab::canRunController(state));
}

} // namespace

int main()
{
    testEveryKnownStateEntersFault();
    testUnknownStateFailsSafeToFault();
    testFaultRemainsLatched();

    std::puts("enterFault tests passed");
    return 0;
}
