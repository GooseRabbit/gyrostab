#include "../src/config/SystemConfig.h"

#include <cassert>
#include <cstdio>

int main()
{
    using namespace gyrostab;

    SystemConfig config = {};
    config.board.spiSckPin = -10;
    config.board.spiMisoPin = -11;
    config.board.spiMosiPin = -12;
    config.board.imuChipSelectPin = -13;
    config.board.servoSignalPin = -14;
    config.board.pinsVerified = true;

    config.imu.fullScale.a = 1U;
    config.imu.fullScale.g = 2U;
    config.imu.sampleRateDivider.a = 3U;
    config.imu.sampleRateDivider.g = 4U;
    config.imu.dlpf.a = 5U;
    config.imu.dlpf.g = 6U;
    config.imu.accelerometerDlpfEnabled = true;
    config.imu.gyroscopeDlpfEnabled = true;

    config.controlAxis.axisInSensorFrame.x = 1.0f;
    config.controlAxis.axisInSensorFrame.y = 2.0f;
    config.controlAxis.axisInSensorFrame.z = 3.0f;
    config.mounting.platformXInSensorFrame.x = 1.0f;
    config.mounting.platformYInSensorFrame.y = 1.0f;
    config.mounting.platformZInSensorFrame.z = 1.0f;

    config.servo = {50, 1500, 1000, 2000, 1, 100.0f};
    config.control = {1.0f, 2.0f, 0.01f, 0.02f};
    config.timing = {10000U, 10000U, 5000U};
    config.imuSettingsConfigured = true;
    config.controlAxisConfigured = true;
    config.mountingConfigured = true;
    config.servoParametersConfigured = true;
    config.controlParametersConfigured = true;
    config.timingParametersConfigured = true;

    loadDefaultConfig(config);

    // Загружается распайка, подтверждённая пользователем для этой платы.
    assert(config.board.spiSckPin == 18);
    assert(config.board.spiMisoPin == 19);
    assert(config.board.spiMosiPin == 23);
    assert(config.board.imuChipSelectPin == 5);
    assert(config.board.servoSignalPin == 25);
    assert(config.board.pinsVerified);

    // Неутверждённые параметры IMU не заменяются произвольным режимом.
    assert(config.imu.fullScale.a == 0U);
    assert(config.imu.fullScale.g == 0U);
    assert(config.imu.sampleRateDivider.a == 0U);
    assert(config.imu.sampleRateDivider.g == 0U);
    assert(config.imu.dlpf.a == 0U);
    assert(config.imu.dlpf.g == 0U);
    assert(!config.imu.accelerometerDlpfEnabled);
    assert(!config.imu.gyroscopeDlpfEnabled);

    // Физическая ось и базис монтажа остаются незаданными.
    assert(config.controlAxis.axisInSensorFrame.x == 0.0f);
    assert(config.controlAxis.axisInSensorFrame.y == 0.0f);
    assert(config.controlAxis.axisInSensorFrame.z == 0.0f);
    assert(config.mounting.platformXInSensorFrame.x == 0.0f);
    assert(config.mounting.platformXInSensorFrame.y == 0.0f);
    assert(config.mounting.platformXInSensorFrame.z == 0.0f);
    assert(config.mounting.platformYInSensorFrame.x == 0.0f);
    assert(config.mounting.platformYInSensorFrame.y == 0.0f);
    assert(config.mounting.platformYInSensorFrame.z == 0.0f);
    assert(config.mounting.platformZInSensorFrame.x == 0.0f);
    assert(config.mounting.platformZInSensorFrame.y == 0.0f);
    assert(config.mounting.platformZInSensorFrame.z == 0.0f);

    // Неизмеренные параметры сервы, регулятора и времени не получают фиктивных defaults.
    assert(config.servo.refreshRateHz == 0);
    assert(config.servo.centerPulseUs == 0);
    assert(config.servo.minimumPulseUs == 0);
    assert(config.servo.maximumPulseUs == 0);
    assert(config.servo.mechanicalDirection == 0);
    assert(config.servo.maximumChangeUsPerSecond == 0.0f);
    assert(config.control.proportionalGainUsPerRad == 0.0f);
    assert(config.control.derivativeGainUsPerRadPerSec == 0.0f);
    assert(config.control.deadZoneEnterThresholdRad == 0.0f);
    assert(config.control.deadZoneExitThresholdRad == 0.0f);
    assert(config.timing.maximumSampleAgeUs == 0U);
    assert(config.timing.maximumOrientationAgeUs == 0U);
    assert(config.timing.maximumControlDeltaUs == 0U);

    // Все группы, требующие измерения/подтверждения, явно помечены не настроенными.
    assert(!config.imuSettingsConfigured);
    assert(!config.controlAxisConfigured);
    assert(!config.mountingConfigured);
    assert(!config.servoParametersConfigured);
    assert(!config.controlParametersConfigured);
    assert(!config.timingParametersConfigured);

    // Повторный вызов полностью сбрасывает ранее изменённые значения.
    config.board.spiSckPin = 99;
    config.control.proportionalGainUsPerRad = 10.0f;
    config.servo.centerPulseUs = 1500;
    loadDefaultConfig(config);
    assert(config.board.spiSckPin == 18);
    assert(config.control.proportionalGainUsPerRad == 0.0f);
    assert(config.servo.centerPulseUs == 0);
    assert(config.board.pinsVerified);

    std::puts("loadDefaultConfig tests passed");
    return 0;
}
