#pragma once

#include "../geometry/AxisConfig.h"
#include "../geometry/MountingConfig.h"
#include "../sensors/ImuDriver.h"

#include <stdint.h>

namespace gyrostab
{

// Номинальная распиновка из текущей схемы проекта.
// pinsVerified остаётся false до сверки с фактической проводкой.
struct BoardConfig
{
    int spiSckPin;
    int spiMisoPin;
    int spiMosiPin;
    int imuChipSelectPin;
    int servoSignalPin;
    bool pinsVerified;
};

// Механические параметры сервопривода; нулевые значения означают,
// что безопасные пределы и динамика ещё не утверждены.
struct ServoConfig
{
    int refreshRateHz;
    int centerPulseUs;
    int minimumPulseUs;
    int maximumPulseUs;
    int mechanicalDirection;
    float maximumChangeUsPerSecond;
};

// Параметры P-D регулятора и гистерезисной dead-zone.
struct ControlConfig
{
    float proportionalGainUsPerRad;
    float derivativeGainUsPerRadPerSec;
    float deadZoneEnterThresholdRad;
    float deadZoneExitThresholdRad;
};

// Ограничения времени; значения заполняются после проверки частоты IMU и цикла.
struct TimingConfig
{
    uint32_t maximumSampleAgeUs;
    uint32_t maximumOrientationAgeUs;
    uint32_t maximumControlDeltaUs;
};

// Полная caller-owned конфигурация прошивки.
// Групповые флаги не позволяют принять нулевые placeholders за настроенные данные.
struct SystemConfig
{
    BoardConfig board;
    ImuConfig imu;
    AxisConfig controlAxis;
    MountingConfig mounting;
    ServoConfig servo;
    ControlConfig control;
    TimingConfig timing;
    bool imuSettingsConfigured;
    bool controlAxisConfigured;
    bool mountingConfigured;
    bool servoParametersConfigured;
    bool controlParametersConfigured;
    bool timingParametersConfigured;
};

// Загружает известную номинальную распиновку и сбрасывает
// неподтверждённые настройки в явное не настроенное состояние.
void loadDefaultConfig(SystemConfig& config);

// Проверяет заполненность, диапазоны и согласованность всей конфигурации.
// Не нормирует и не исправляет входные поля; false означает, что конфигурацию нельзя использовать.
bool validateConfig(const SystemConfig& config);

} // namespace gyrostab
