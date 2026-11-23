#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace
{

bool mockReadImuResult = false;
gyrostab::ImuSample mockReadSample = {};
ICM_20948_Status_e mockDriverStatus = ICM_20948_Stat_Unknown;

gyrostab::ImuConfig makeConfig()
{
    gyrostab::ImuConfig config = {};
    config.fullScale.a = 0;
    config.fullScale.g = 0;
    return config;
}

gyrostab::ImuSample makeValidSample(uint32_t timestampUs, uint32_t sequence)
{
    gyrostab::ImuSample sample = {};
    sample.accelerationMps2 = {1.0f, 2.0f, 9.80665f};
    sample.angularVelocityRadPerSec = {0.1f, -0.2f, 0.3f};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    return sample;
}

} // namespace

namespace gyrostab
{

bool readImuSample(ImuSample& sample)
{
    if (!mockReadImuResult)
    {
        return false;
    }

    sample = mockReadSample;
    return true;
}

ICM_20948_Status_e checkImuStatus()
{
    return mockDriverStatus;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;

    // До первого валидного образца кэш пуст, статус драйвера ещё неизвестен.
    SensorHealth health = getSensorHealth(100U, 50U);
    assert(health.driverStatus == ICM_20948_Stat_Unknown);
    assert(!health.hasValidSample);
    assert(!health.sampleFresh);
    assert(health.lastSampleTimestampUs == 0U);
    assert(health.lastSampleSequence == 0U);

    // После update кэш содержит валидный свежий образец.
    mockDriverStatus = ICM_20948_Stat_Ok;
    mockReadImuResult = true;
    mockReadSample = makeValidSample(1000U, 7U);
    assert(updateSensorData(makeConfig()));

    health = getSensorHealth(1050U, 50U);
    assert(health.driverStatus == ICM_20948_Stat_Ok);
    assert(health.hasValidSample);
    assert(health.sampleFresh);
    assert(health.lastSampleTimestampUs == 1000U);
    assert(health.lastSampleSequence == 7U);

    // Устаревание не удаляет кэш: валидность и свежесть остаются разными признаками.
    health = getSensorHealth(1051U, 50U);
    assert(health.hasValidSample);
    assert(!health.sampleFresh);
    assert(health.lastSampleTimestampUs == 1000U);
    assert(health.lastSampleSequence == 7U);

    // Текущая ошибка драйвера отображается отдельно от ранее сохранённого образца.
    mockDriverStatus = ICM_20948_Stat_Err;
    health = getSensorHealth(1020U, 50U);
    assert(health.driverStatus == ICM_20948_Stat_Err);
    assert(health.hasValidSample);
    assert(health.sampleFresh);

    // Неоднозначный интервал свежести отклоняется, но кэш и его метаданные сохраняются.
    health = getSensorHealth(1020U, 0x80000000UL);
    assert(health.hasValidSample);
    assert(!health.sampleFresh);
    assert(health.lastSampleSequence == 7U);

    std::puts("getSensorHealth tests passed");
    return 0;
}
