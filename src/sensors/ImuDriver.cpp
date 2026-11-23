#include "ImuDriver.h"

#include <Arduino.h>
#include <SPI.h>

namespace gyrostab
{
namespace
{
// Текущая распиновка SPI из конфигурации проекта.
// Перед аппаратным запуском необходимо сверить её с фактическим подключением.
constexpr uint8_t kImuSckPin = 18;
constexpr uint8_t kImuMisoPin = 19;
constexpr uint8_t kImuMosiPin = 23;
constexpr uint8_t kImuCsPin = 5;

// Один экземпляр датчика используется всеми функциями драйвера IMU.
ICM_20948_SPI imu;
bool imuInitialized = false;
} // namespace

bool beginImu()
{
    imuInitialized = false;

    // Назначаем SPI-пины ESP32 явно, не полагаясь на выводы по умолчанию платы.
    if (!SPI.begin(kImuSckPin, kImuMisoPin, kImuMosiPin, kImuCsPin))
    {
        return false;
    }

    // SparkFun begin() проверяет идентификатор ICM-20948 и выполняет
    // штатную начальную настройку библиотеки. Частота SPI по умолчанию — 4 МГц.
    const ICM_20948_Status_e status = imu.begin(kImuCsPin, SPI);
    imuInitialized = (status == ICM_20948_Stat_Ok);

    return imuInitialized;
}

bool configureImu(const ImuConfig& config)
{
    if (!imuInitialized)
    {
        return false;
    }

    // Библиотека записывает делитель акселерометра в 12-битные регистры,
    // а значения DLPF для обоих датчиков должны находиться в диапазоне 0..7.
    if (config.sampleRateDivider.a > 0x0FFF ||
        config.dlpf.a > 7U ||
        config.dlpf.g > 7U)
    {
        return false;
    }

    constexpr uint8_t kMotionSensors =
        static_cast<uint8_t>(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr);

    // Сначала задаём непрерывный режим, затем диапазоны, делители и фильтры.
    if (imu.setSampleMode(kMotionSensors, ICM_20948_Sample_Mode_Continuous) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    if (imu.setFullScale(kMotionSensors, config.fullScale) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    if (imu.setSampleRate(kMotionSensors, config.sampleRateDivider) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    if (imu.setDLPFcfg(kMotionSensors, config.dlpf) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    if (imu.enableDLPF(ICM_20948_Internal_Acc, config.accelerometerDlpfEnabled) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    if (imu.enableDLPF(ICM_20948_Internal_Gyr, config.gyroscopeDlpfEnabled) != ICM_20948_Stat_Ok)
    {
        return false;
    }

    return true;
}

} // namespace gyrostab
