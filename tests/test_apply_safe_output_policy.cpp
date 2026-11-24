#include "../src/safety/SafetyManager.h"
#include "../src/servo/ServoDriver.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace gyrostab
{

// Заглушки используются только для линковки остальных функций SafetyManager.cpp.
struct ImuSample;
struct OrientationEstimate;
bool isSampleFresh(const ImuSample&, uint32_t, uint32_t) { return false; }
bool validateOrientation(const OrientationEstimate&, float) { return false; }

} // namespace gyrostab

namespace
{

void testFaultReappliesLastAcceptedPulse()
{
    Servo servo;
    servo.setAttached(true);

    const int lastCommandPulseUs = 1475;
    assert(gyrostab::writeServoPulse(servo, lastCommandPulseUs, 1000, 2000));
    const int initialWriteCalls = servo.writeCalls();

    gyrostab::SafetyState state = gyrostab::SafetyState::Active;
    gyrostab::enterFault(state);

    assert(gyrostab::applySafeOutputPolicy(
        state, servo, lastCommandPulseUs, 1000, 2000));
    assert(servo.attached());
    assert(servo.writeCalls() == initialWriteCalls + 1);
    assert(servo.lastWrittenPulseUs() == lastCommandPulseUs);
    assert(servo.detachCalls() == 0);
}

void testRepeatedFaultPolicyKeepsSamePulse()
{
    Servo servo;
    servo.setAttached(true);
    const int pulseUs = 1520;
    assert(gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, pulseUs, 1000, 2000));
    assert(gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, pulseUs, 1000, 2000));
    assert(servo.lastWrittenPulseUs() == pulseUs);
    assert(servo.writeCalls() == 2);
    assert(servo.detachCalls() == 0);
}

void testNonFaultStateDoesNotChangeOutput()
{
    Servo servo;
    servo.setAttached(true);
    const int beforeWrites = servo.writeCalls();

    assert(!gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Active, servo, 1500, 1000, 2000));
    assert(servo.attached());
    assert(servo.writeCalls() == beforeWrites);
    assert(servo.detachCalls() == 0);
}

void testDetachedServoIsNotReattached()
{
    Servo servo;
    assert(!servo.attached());

    assert(!gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, 1500, 1000, 2000));
    assert(!servo.attached());
    assert(servo.attachCalls() == 0);
    assert(servo.writeCalls() == 0);
}

void testInvalidPulseOrLimitsAreRejected()
{
    Servo servo;
    servo.setAttached(true);

    assert(!gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, 999, 1000, 2000));
    assert(!gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, 1500, 2000, 1000));
    assert(!gyrostab::applySafeOutputPolicy(
        gyrostab::SafetyState::Fault, servo, 1500, 499, 2000));

    assert(servo.writeCalls() == 0);
    assert(servo.detachCalls() == 0);
}

void testUnknownStateDoesNotChangeOutput()
{
    Servo servo;
    servo.setAttached(true);
    const gyrostab::SafetyState invalidState =
        static_cast<gyrostab::SafetyState>(UINT8_MAX);

    assert(!gyrostab::applySafeOutputPolicy(
        invalidState, servo, 1500, 1000, 2000));
    assert(servo.writeCalls() == 0);
    assert(servo.detachCalls() == 0);
    assert(servo.attached());
}

} // namespace

int main()
{
    testFaultReappliesLastAcceptedPulse();
    testRepeatedFaultPolicyKeepsSamePulse();
    testNonFaultStateDoesNotChangeOutput();
    testDetachedServoIsNotReattached();
    testInvalidPulseOrLimitsAreRejected();
    testUnknownStateDoesNotChangeOutput();

    std::puts("applySafeOutputPolicy tests passed");
    return 0;
}
