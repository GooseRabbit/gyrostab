#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdio>

uint32_t mockMicrosValue = 0;
SPIClass SPI;
bool mockSpiBeginResult = true;
bool SPIClass::begin(int8_t, int8_t, int8_t, int8_t) { return mockSpiBeginResult; }
ICM_20948_Status_e mockBeginStatus = ICM_20948_Stat_Ok;
ICM_20948_Status_e mockReadStatus = ICM_20948_Stat_Ok;
bool mockDataReady = false;
ICM_20948_AGMT_t mockAgmt = {};

namespace {
gyrostab::ImuConfig makeConfig()
{
    gyrostab::ImuConfig config = {};
    config.fullScale.a = 0U;
    config.fullScale.g = 0U;
    config.sampleRateDivider.a = 19U;
    config.sampleRateDivider.g = 19U;
    config.dlpf.a = 0U;
    config.dlpf.g = 0U;
    config.accelerometerDlpfEnabled = true;
    config.gyroscopeDlpfEnabled = true;
    return config;
}
void publishOneSample(const gyrostab::ImuConfig& config, uint32_t timestamp)
{
    mockMicrosValue = timestamp;
    mockDataReady = true;
    assert(gyrostab::updateSensorData(config));
}
}

int main()
{
    using namespace gyrostab;
    const ImuConfig config = makeConfig();
    mockAgmt.acc.z = 1000;
    mockAgmt.gyr.x = 20;

    // Создаём валидный сохранённый образец.
    assert(beginImu());
    assert(configureImu(config));
    publishOneSample(config, 1000U);
    ImuSample cached = {};
    assert(getLatestSample(cached, 1000U, 100U));
    assert(cached.sequence == 1U);
    assert(getSensorHealth(1000U, 100U).hasValidSample);

    // Успешный сброс удаляет кэш и оставляет драйвер ненастроенным.
    assert(resetImuDataPath());
    assert(!getLatestSample(cached, 1000U, 100U));
    SensorHealth health = getSensorHealth(1000U, 100U);
    assert(!health.hasValidSample && !health.sampleFresh);
    assert(!readImuSample(cached));
    assert(checkImuStatus() == ICM_20948_Stat_Ok);

    // После перезапуска configureImu() обязателен; sequence начинается с единицы.
    assert(configureImu(config));
    publishOneSample(config, 2000U);
    assert(getLatestSample(cached, 2000U, 100U));
    assert(cached.sequence == 1U && cached.timestampUs == 2000U);

    // Даже неудачный запуск SPI инвалидирует образец, полученный до сброса.
    mockSpiBeginResult = false;
    assert(!resetImuDataPath());
    assert(checkImuStatus() == ICM_20948_Stat_Err);
    assert(!getLatestSample(cached, 2000U, 100U));
    health = getSensorHealth(2000U, 100U);
    assert(!health.hasValidSample);
    assert(!configureImu(config));

    // Ошибка идентификации датчика также видна вызывающему коду.
    mockSpiBeginResult = true;
    mockBeginStatus = ICM_20948_Stat_WrongID;
    assert(!resetImuDataPath());
    assert(checkImuStatus() == ICM_20948_Stat_WrongID);
    assert(!getLatestSample(cached, 2000U, 100U));

    // После устранения причины отказа повторный запуск восстанавливает тракт.
    mockBeginStatus = ICM_20948_Stat_Ok;
    assert(resetImuDataPath());
    assert(configureImu(config));
    publishOneSample(config, 3000U);
    assert(getLatestSample(cached, 3000U, 100U));
    assert(cached.sequence == 1U && cached.timestampUs == 3000U);

    std::puts("resetImuDataPath tests passed");
    return 0;
}
