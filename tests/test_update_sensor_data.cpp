#include "../src/sensors/SensorManager.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{

bool mockReadImuResult = false;
int mockReadCount = 0;
gyrostab::ImuSample mockReadSample = {};

gyrostab::ImuConfig makeConfig()
{
    gyrostab::ImuConfig config = {};
    config.fullScale.a = 0;
    config.fullScale.g = 0;
    return config;
}

gyrostab::ImuSample makeValidSample()
{
    gyrostab::ImuSample sample = {};
    sample.accelerationMps2 = {0.0f, 0.0f, 9.80665f};
    sample.angularVelocityRadPerSec = {0.0f, 0.0f, 0.0f};
    sample.magneticFieldMicroTesla = {25.0f, -10.0f, 40.0f};
    sample.timestampUs = 1000U;
    sample.sequence = 1U;
    sample.accelerometerValid = true;
    sample.gyroscopeValid = true;
    sample.magnetometerValid = true;
    return sample;
}

} // namespace

namespace gyrostab
{

bool readImuSample(ImuSample& sample)
{
    ++mockReadCount;
    if (!mockReadImuResult)
    {
        return false;
    }

    sample = mockReadSample;
    return true;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;
    const ImuConfig config = makeConfig();

    // Драйвер сообщил, что нового образца нет или чтение не удалось.
    mockReadImuResult = false;
    assert(!updateSensorData(config));
    assert(mockReadCount == 1);

    // Новый образец с валидными обязательными каналами принимается.
    mockReadImuResult = true;
    mockReadSample = makeValidSample();
    assert(updateSensorData(config));
    assert(mockReadCount == 2);

    // Невалидный ускорительный канал не должен заменять сохранённый образец.
    mockReadSample = makeValidSample();
    mockReadSample.accelerometerValid = false;
    assert(!updateSensorData(config));

    // Не конечные значения и выход за выбранный full-scale также отклоняются.
    mockReadSample = makeValidSample();
    mockReadSample.angularVelocityRadPerSec.x = std::numeric_limits<float>::quiet_NaN();
    assert(!updateSensorData(config));

    mockReadSample = makeValidSample();
    mockReadSample.accelerationMps2.x = 25.0f;
    assert(!updateSensorData(config));

    // Ошибочный диапазон конфигурации не пропускает новый образец.
    ImuConfig invalidConfig = config;
    invalidConfig.fullScale.a = 4;
    mockReadSample = makeValidSample();
    assert(!updateSensorData(invalidConfig));

    // Магнитометр необязателен, как и в validateSensorSample().
    mockReadSample = makeValidSample();
    mockReadSample.magnetometerValid = false;
    mockReadSample.magneticFieldMicroTesla.x = std::numeric_limits<float>::quiet_NaN();
    assert(updateSensorData(config));

    std::puts("updateSensorData tests passed");
    return 0;
}
