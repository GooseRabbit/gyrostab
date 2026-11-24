#include "SystemConfig.h"

#include "../geometry/MountingConfig.h"
#include "../servo/ServoDriver.h"

#include <cmath>

namespace gyrostab
{
namespace
{

// Допуск только для float-представления единичных векторов/базиса;
// он не заменяет физическое измерение геометрии установки.
const double kGeometryTolerance = 0.01;
const uint32_t kHalfUint32Range = 0x80000000UL;

bool hasDistinctNonNegativePins(const BoardConfig& board)
{
    const int pins[] = {
        board.spiSckPin,
        board.spiMisoPin,
        board.spiMosiPin,
        board.imuChipSelectPin,
        board.servoSignalPin
    };

    for (unsigned int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i)
    {
        if (pins[i] < 0)
        {
            return false;
        }

        for (unsigned int j = i + 1U; j < sizeof(pins) / sizeof(pins[0]); ++j)
        {
            if (pins[i] == pins[j])
            {
                return false;
            }
        }
    }

    return true;
}

bool isUnitAxis(const AxisConfig& axis)
{
    const double x = axis.axisInSensorFrame.x;
    const double y = axis.axisInSensorFrame.y;
    const double z = axis.axisInSensorFrame.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
        return false;
    }

    const double length = std::sqrt(x * x + y * y + z * z);
    return std::isfinite(length) && std::fabs(length - 1.0) <= kGeometryTolerance;
}

bool isValidAgeLimit(uint32_t maximumAgeUs)
{
    return maximumAgeUs > 0U && maximumAgeUs < kHalfUint32Range;
}

} // namespace

void loadDefaultConfig(SystemConfig& config)
{
    // Сначала создаём полностью обнулённый снимок: неподтверждённые
    // настройки остаются placeholders и не должны использоваться как рабочие.
    SystemConfig candidate = {};

    // Номинальная распиновка из текущего драйвера и архитектуры проекта.
    candidate.board.spiSckPin = 18;
    candidate.board.spiMisoPin = 19;
    candidate.board.spiMosiPin = 23;
    candidate.board.imuChipSelectPin = 5;
    candidate.board.servoSignalPin = 25;
    candidate.board.pinsVerified = false;

    // Только после полной подготовки заменяем переданную конфигурацию.
    config = candidate;
}

bool validateConfig(const SystemConfig& config)
{
    // Номинальные GPIO нельзя использовать до сверки с реальной проводкой.
    if (!config.board.pinsVerified || !hasDistinctNonNegativePins(config.board))
    {
        return false;
    }

    // Все группы должны быть намеренно заполнены после loadDefaultConfig().
    if (!config.imuSettingsConfigured || !config.controlAxisConfigured ||
        !config.mountingConfigured || !config.servoParametersConfigured ||
        !config.controlParametersConfigured || !config.timingParametersConfigured)
    {
        return false;
    }

    // Ограничения соответствуют API ICM-20948 и существующему configureImu().
    if (config.imu.fullScale.a > 3U || config.imu.fullScale.g > 3U ||
        config.imu.sampleRateDivider.a > 0x0FFFU ||
        config.imu.dlpf.a > 7U || config.imu.dlpf.g > 7U)
    {
        return false;
    }

    if (!isUnitAxis(config.controlAxis) ||
        !validateMountingConfig(config.mounting, static_cast<float>(kGeometryTolerance)))
    {
        return false;
    }

    // Центр может совпадать с границей: это соответствует контракту ограничителя.
    if (config.servo.refreshRateHz <= 0 ||
        config.servo.minimumPulseUs < MIN_PULSE_WIDTH ||
        config.servo.maximumPulseUs > MAX_PULSE_WIDTH ||
        config.servo.minimumPulseUs >= config.servo.maximumPulseUs ||
        config.servo.centerPulseUs < config.servo.minimumPulseUs ||
        config.servo.centerPulseUs > config.servo.maximumPulseUs ||
        (config.servo.mechanicalDirection != 1 && config.servo.mechanicalDirection != -1) ||
        !std::isfinite(config.servo.maximumChangeUsPerSecond) ||
        config.servo.maximumChangeUsPerSecond < 0.0f)
    {
        return false;
    }

    if (!std::isfinite(config.control.proportionalGainUsPerRad) ||
        !std::isfinite(config.control.derivativeGainUsPerRadPerSec) ||
        config.control.proportionalGainUsPerRad < 0.0f ||
        config.control.derivativeGainUsPerRadPerSec < 0.0f ||
        !std::isfinite(config.control.deadZoneEnterThresholdRad) ||
        !std::isfinite(config.control.deadZoneExitThresholdRad) ||
        config.control.deadZoneEnterThresholdRad < 0.0f ||
        config.control.deadZoneExitThresholdRad <= config.control.deadZoneEnterThresholdRad)
    {
        return false;
    }

    if (!isValidAgeLimit(config.timing.maximumSampleAgeUs) ||
        !isValidAgeLimit(config.timing.maximumOrientationAgeUs) ||
        !isValidAgeLimit(config.timing.maximumControlDeltaUs))
    {
        return false;
    }

    return true;
}

} // namespace gyrostab
