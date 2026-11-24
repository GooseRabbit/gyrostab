#include "../src/config/SystemConfig.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

gyrostab::SystemConfig makeValidConfig()
{
    gyrostab::SystemConfig config = {};
    config.board = {18, 19, 23, 5, 25, true};

    config.imu.fullScale.a = 0U;
    config.imu.fullScale.g = 0U;
    config.imu.sampleRateDivider.a = 19U;
    config.imu.sampleRateDivider.g = 19U;
    config.imu.dlpf.a = 0U;
    config.imu.dlpf.g = 0U;
    config.imu.accelerometerDlpfEnabled = true;
    config.imu.gyroscopeDlpfEnabled = true;

    config.controlAxis.axisInSensorFrame = {0.0f, 1.0f, 0.0f};
    config.mounting = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    config.servo = {50, 1500, 1000, 2000, 1, 100.0f};
    config.control = {10.0f, 2.0f, 0.01f, 0.02f};
    config.timing = {20000U, 20000U, 10000U};

    config.imuSettingsConfigured = true;
    config.controlAxisConfigured = true;
    config.mountingConfigured = true;
    config.servoParametersConfigured = true;
    config.controlParametersConfigured = true;
    config.timingParametersConfigured = true;
    return config;
}

void testValidConfigurationIsAccepted()
{
    gyrostab::SystemConfig config = makeValidConfig();
    assert(gyrostab::validateConfig(config));

    // Zero gains and zero slew rate are explicit non-negative settings allowed by their APIs.
    config.control.proportionalGainUsPerRad = 0.0f;
    config.control.derivativeGainUsPerRadPerSec = 0.0f;
    config.servo.maximumChangeUsPerSecond = 0.0f;
    assert(gyrostab::validateConfig(config));

    // ESP32Servo API endpoints are valid when the min/max interval is non-empty.
    config.servo.minimumPulseUs = 500;
    config.servo.maximumPulseUs = 2500;
    config.servo.centerPulseUs = 1500;
    assert(gyrostab::validateConfig(config));

    config.servo.mechanicalDirection = -1;
    assert(gyrostab::validateConfig(config));
}

void testDefaultProfileIsNotReadyForUse()
{
    gyrostab::SystemConfig config = makeValidConfig();
    gyrostab::loadDefaultConfig(config);
    assert(!gyrostab::validateConfig(config));
    assert(config.board.pinsVerified);
    assert(!config.imuSettingsConfigured);
    assert(!config.controlAxisConfigured);
    assert(!config.mountingConfigured);
    assert(!config.servoParametersConfigured);
    assert(!config.controlParametersConfigured);
    assert(!config.timingParametersConfigured);
}

void testRejectsBoardProblems()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.board.pinsVerified = false;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.board.spiSckPin = -1;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.board.spiSckPin = 128;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.board.servoSignalPin = config.board.imuChipSelectPin;
    assert(!gyrostab::validateConfig(config));
}

void testRejectsUnconfiguredGroups()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.imuSettingsConfigured = false;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.controlAxisConfigured = false;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.mountingConfigured = false;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servoParametersConfigured = false;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.controlParametersConfigured = false;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.timingParametersConfigured = false;
    assert(!gyrostab::validateConfig(config));
}

void testRejectsInvalidImuSettings()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.imu.fullScale.a = 4U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.imu.fullScale.g = 4U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.imu.fullScale.a = 3U;
    config.imu.fullScale.g = 3U;
    config.imu.sampleRateDivider.a = 0x0FFFU;
    config.imu.dlpf.a = 7U;
    config.imu.dlpf.g = 7U;
    assert(gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.imu.sampleRateDivider.a = 0x1000U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.imu.dlpf.a = 8U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.imu.dlpf.g = 8U;
    assert(!gyrostab::validateConfig(config));
}

void testRejectsInvalidAxisAndMounting()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.controlAxis.axisInSensorFrame = {0.0f, 0.0f, 0.0f};
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.controlAxis.axisInSensorFrame = {0.0f, 1.01f, 0.0f};
    assert(gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.controlAxis.axisInSensorFrame = {0.0f, 1.02f, 0.0f};
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.controlAxis.axisInSensorFrame.x = std::numeric_limits<float>::quiet_NaN();
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.mounting.platformYInSensorFrame.y = -1.0f;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.mounting.platformYInSensorFrame.x = 0.1f;
    assert(!gyrostab::validateConfig(config));
}

void testRejectsInvalidServoSettings()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.servo.refreshRateHz = 0;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig(); config.servo.minimumPulseUs = 499;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.maximumPulseUs = 2501;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.minimumPulseUs = config.servo.maximumPulseUs;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.centerPulseUs = 999;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.mechanicalDirection = 0;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.maximumChangeUsPerSecond = -1.0f;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.servo.maximumChangeUsPerSecond = std::numeric_limits<float>::infinity();
    assert(!gyrostab::validateConfig(config));
}

void testRejectsInvalidControlSettings()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.control.proportionalGainUsPerRad = -0.1f;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.control.derivativeGainUsPerRadPerSec = std::numeric_limits<float>::quiet_NaN();
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig(); config.control.deadZoneEnterThresholdRad = -0.01f;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.control.deadZoneExitThresholdRad = config.control.deadZoneEnterThresholdRad;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.control.deadZoneExitThresholdRad = 0.01f;
    assert(!gyrostab::validateConfig(config));
}

void testRejectsInvalidTimingSettings()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.timing.maximumSampleAgeUs = 0U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig(); config.timing.maximumSampleAgeUs = 0x80000000UL;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.timing.maximumOrientationAgeUs = 0x80000001UL;
    assert(!gyrostab::validateConfig(config));
    config = makeValidConfig(); config.timing.maximumControlDeltaUs = 0U;
    assert(!gyrostab::validateConfig(config));

    config = makeValidConfig();
    config.timing = {0x7FFFFFFFUL, 0x7FFFFFFFUL, 0x7FFFFFFFUL};
    assert(gyrostab::validateConfig(config));
}

void testValidationDoesNotMutateConfiguration()
{
    gyrostab::SystemConfig config = makeValidConfig();
    config.servo.mechanicalDirection = 0;
    const gyrostab::SystemConfig saved = config;
    assert(!gyrostab::validateConfig(config));
    assert(config.board.spiSckPin == saved.board.spiSckPin);
    assert(config.board.pinsVerified == saved.board.pinsVerified);
    assert(config.servo.mechanicalDirection == saved.servo.mechanicalDirection);
    assert(config.control.deadZoneEnterThresholdRad == saved.control.deadZoneEnterThresholdRad);
    assert(config.timing.maximumSampleAgeUs == saved.timing.maximumSampleAgeUs);
}

} // namespace

int main()
{
    testValidConfigurationIsAccepted();
    testDefaultProfileIsNotReadyForUse();
    testRejectsBoardProblems();
    testRejectsUnconfiguredGroups();
    testRejectsInvalidImuSettings();
    testRejectsInvalidAxisAndMounting();
    testRejectsInvalidServoSettings();
    testRejectsInvalidControlSettings();
    testRejectsInvalidTimingSettings();
    testValidationDoesNotMutateConfiguration();

    std::puts("validateConfig tests passed");
    return 0;
}
