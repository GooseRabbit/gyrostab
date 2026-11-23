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

constexpr float kMilliGToMetersPerSecondSquared = 0.00980665f;
constexpr float kDegreesToRadians = 0.01745329252f;

// Один экземпляр датчика используется всеми функциями драйвера IMU.
ICM_20948_SPI imu;
bool imuInitialized = false;
bool imuConfigured = false;
bool imuBeginAttempted = false;
uint32_t sampleSequence = 0;
} // namespace

bool beginImu()
{
    imuBeginAttempted = true;
    imuInitialized = false;
    imuConfigured = false;
    sampleSequence = 0;

    // Назначаем SPI-пины ESP32 явно, не полагаясь на выводы по умолчанию платы.
    if (!SPI.begin(kImuSckPin, kImuMisoPin, kImuMosiPin, kImuCsPin))
    {
        imu.status = ICM_20948_Stat_Err;
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
    imuConfigured = false;

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

    imuConfigured = true;
    return true;
}

ICM_20948_Status_e checkImuStatus()
{
    if (!imuBeginAttempted)
    {
        return ICM_20948_Stat_Unknown;
    }

    return imu.status;
}

bool readImuSample(ImuSample& sample)
{
    if (!imuConfigured || !imu.dataReady())
    {
        return false;
    }

    // getAGMT() обновляет внутреннюю структуру библиотеки и её status.
    const ICM_20948_AGMT_t agmt = imu.getAGMT();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        return false;
    }

    // Библиотека возвращает ускорение в milli-g, скорость в град/с,
    // магнитное поле в микротеслах. Переводим первые два канала в единицы проекта.
    const ImuSample candidate = {
        {
            imu.accX() * kMilliGToMetersPerSecondSquared,
            imu.accY() * kMilliGToMetersPerSecondSquared,
            imu.accZ() * kMilliGToMetersPerSecondSquared
        },
        {
            imu.gyrX() * kDegreesToRadians,
            imu.gyrY() * kDegreesToRadians,
            imu.gyrZ() * kDegreesToRadians
        },
        {
            imu.magX(),
            imu.magY(),
            imu.magZ()
        },
        static_cast<uint32_t>(micros()),
        sampleSequence + 1U,
        true,
        true,
        ((agmt.magStat1 & 0x01U) != 0U) && ((agmt.magStat2 & 0x08U) == 0U)
    };

    // Публикуем образец целиком только после успешного чтения.
    sample = candidate;
    sampleSequence = candidate.sequence;
    return true;
}

} // namespace gyrostab
