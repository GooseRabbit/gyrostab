#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

namespace
{

bool mockReadImuResult = false;
gyrostab::ImuSample mockReadSample = {};

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
    sample.magneticFieldMicroTesla = {25.0f, -10.0f, 40.0f};
    sample.timestampUs = timestampUs;
    sample.sequence = sequence;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = true;
    return sample;
}

gyrostab::ImuSample makeSentinelSample()
{
    gyrostab::ImuSample sample = {};
    sample.accelerationMps2 = {77.0f, 78.0f, 79.0f};
    sample.angularVelocityRadPerSec = {11.0f, 12.0f, 13.0f};
    sample.timestampUs = 777U;
    sample.sequence = 777U;
    sample.accelerometerValid = false;
    sample.gyroscopeValid = false;
    return sample;
}

void assertSentinelUnchanged(const gyrostab::ImuSample& sample)
{
    assert(sample.accelerationMps2.x == 77.0f);
    assert(sample.accelerationMps2.y == 78.0f);
    assert(sample.accelerationMps2.z == 79.0f);
    assert(sample.angularVelocityRadPerSec.x == 11.0f);
    assert(sample.timestampUs == 777U);
    assert(sample.sequence == 777U);
    assert(!sample.accelerometerValid && !sample.gyroscopeValid);
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
    return ICM_20948_Stat_Ok;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;
    const ImuConfig config = makeConfig();

    // До первого валидного update кэш пуст, выходной объект не меняется.
    ImuSample output = makeSentinelSample();
    assert(!getLatestSample(output, 100U, 100U));
    assertSentinelUnchanged(output);

    // После успешного update getter возвращает сохранённый образец.
    mockReadImuResult = true;
    mockReadSample = makeValidSample(1000U, 1U);
    assert(updateSensorData(config));
    assert(getLatestSample(output, 1050U, 50U));
    assert(output.sequence == 1U);
    assert(output.timestampUs == 1000U);
    assert(output.accelerationMps2.x == 1.0f);
    assert(output.angularVelocityRadPerSec.z == 0.3f);
    assert(output.magnetometerValid);

    // Образец старше лимита не выдаётся, а выходной объект остаётся нетронутым.
    output = makeSentinelSample();
    assert(!getLatestSample(output, 1051U, 50U));
    assertSentinelUnchanged(output);

    // Отсутствие новых данных не сбрасывает предыдущий кэш.
    mockReadImuResult = false;
    assert(!updateSensorData(config));
    assert(getLatestSample(output, 1050U, 50U));
    assert(output.sequence == 1U);

    // Невалидный кандидат тоже не заменяет предыдущий образец.
    mockReadImuResult = true;
    mockReadSample = makeValidSample(1060U, 2U);
    mockReadSample.accelerometerValid = false;
    assert(!updateSensorData(config));
    assert(getLatestSample(output, 1050U, 50U));
    assert(output.sequence == 1U);

    // Следующий корректный образец заменяет кэш.
    mockReadSample = makeValidSample(2000U, 3U);
    assert(updateSensorData(config));
    assert(getLatestSample(output, 2010U, 10U));
    assert(output.sequence == 3U && output.timestampUs == 2000U);

    // Свежесть остаётся корректной на переполнении micros().
    mockReadSample = makeValidSample(UINT32_MAX - 4U, 4U);
    assert(updateSensorData(config));
    assert(getLatestSample(output, 5U, 10U));
    assert(output.sequence == 4U);

    // Магнитометр может отсутствовать: обязательные каналы и свежесть достаточны.
    mockReadSample = makeValidSample(3000U, 5U);
    mockReadSample.magnetometerValid = false;
    assert(updateSensorData(config));
    assert(getLatestSample(output, 3000U, 0U));
    assert(output.sequence == 5U && !output.magnetometerValid);

    std::puts("getLatestSample tests passed");
    return 0;
}
