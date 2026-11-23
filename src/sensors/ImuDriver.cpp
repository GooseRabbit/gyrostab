#include "ImuDriver.h"

#include <Arduino.h>
#include <SPI.h>
#include <ICM_20948.h>

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

// Один экземпляр датчика используется последующими функциями драйвера IMU.
ICM_20948_SPI imu;
} // namespace

bool beginImu()
{
    // Назначаем SPI-пины ESP32 явно, не полагаясь на выводы по умолчанию платы.
    if (!SPI.begin(kImuSckPin, kImuMisoPin, kImuMosiPin, kImuCsPin))
    {
        return false;
    }

    // SparkFun begin() проверяет идентификатор ICM-20948 и выполняет
    // штатную начальную настройку библиотеки. Частота SPI по умолчанию — 4 МГц.
    const ICM_20948_Status_e status = imu.begin(kImuCsPin, SPI);

    return status == ICM_20948_Stat_Ok;
}

} // namespace gyrostab
