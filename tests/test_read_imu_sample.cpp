#include "../src/sensors/ImuDriver.h"
#include <cassert>
#include <cmath>
#include <cstdio>

uint32_t mockMicrosValue = 0;
SPIClass SPI;
bool mockSpiBeginResult = true;
bool SPIClass::begin(int8_t, int8_t, int8_t, int8_t) { return mockSpiBeginResult; }
ICM_20948_Status_e mockBeginStatus = ICM_20948_Stat_Ok;
ICM_20948_Status_e mockReadStatus = ICM_20948_Stat_Ok;
bool mockDataReady = false;
ICM_20948_AGMT_t mockAgmt = {};

bool near(float a, float b, float epsilon = 0.0002f)
{
    return std::fabs(a - b) <= epsilon;
}

int main()
{
    using namespace gyrostab;

    ImuSample sample = {};
    sample.sequence = 77;
    sample.timestampUs = 1234;
    sample.accelerationMps2.x = 42.0f;

    // Чтение запрещено до начала инициализации/настройки.
    assert(!readImuSample(sample));
    assert(sample.sequence == 77 && sample.timestampUs == 1234);

    mockSpiBeginResult = false;
    assert(!beginImu());
    mockSpiBeginResult = true;
    assert(beginImu());
    assert(!readImuSample(sample)); // beginImu() недостаточно без configureImu()
    assert(sample.sequence == 77);

    ImuConfig config = {};
    config.fullScale.a = 0;
    config.fullScale.g = 0;
    config.sampleRateDivider.a = 0;
    config.sampleRateDivider.g = 0;
    config.dlpf.a = 0;
    config.dlpf.g = 0;
    config.accelerometerDlpfEnabled = true;
    config.gyroscopeDlpfEnabled = true;
    ImuConfig invalidConfig = config;
    invalidConfig.sampleRateDivider.a = 0x1000;
    assert(!configureImu(invalidConfig));
    assert(!readImuSample(sample));
    assert(configureImu(config));

    // Нет нового измерения: старый образец не публикуется повторно.
    mockDataReady = false;
    assert(!readImuSample(sample));
    assert(sample.sequence == 77 && sample.timestampUs == 1234);
    assert(sample.accelerationMps2.x == 42.0f);

    mockAgmt.acc.x = 1000;
    mockAgmt.acc.y = -500;
    mockAgmt.acc.z = 0;
    mockAgmt.gyr.x = 180;
    mockAgmt.gyr.y = -90;
    mockAgmt.gyr.z = 0;
    mockAgmt.mag.x = 10;
    mockAgmt.mag.y = -20;
    mockAgmt.mag.z = 30;
    mockAgmt.magStat1 = 0x01;
    mockAgmt.magStat2 = 0x00;
    mockReadStatus = ICM_20948_Stat_Ok;
    mockDataReady = true;
    mockMicrosValue = 987654;

    assert(readImuSample(sample));
    assert(sample.sequence == 1);
    assert(sample.timestampUs == 987654);
    assert(near(sample.accelerationMps2.x, 9.80665f));
    assert(near(sample.accelerationMps2.y, -4.903325f));
    assert(near(sample.angularVelocityRadPerSec.x, 3.14159265f));
    assert(near(sample.angularVelocityRadPerSec.y, -1.5707963f));
    assert(near(sample.magneticFieldMicroTesla.x, 1.5f));
    assert(sample.accelerometerValid && sample.gyroscopeValid && sample.magnetometerValid);

    // Повторное чтение без нового data-ready не публикует тот же образец повторно.
    const ImuSample afterFirstRead = sample;
    assert(!readImuSample(sample));
    assert(sample.sequence == afterFirstRead.sequence);

    // Следующая аппаратная выборка получает следующий sequence.
    mockDataReady = true;
    mockMicrosValue += 10000;
    assert(readImuSample(sample));
    assert(sample.sequence == 2);
    assert(sample.timestampUs == 997654);

    mockAgmt.magStat1 = 0x00;
    mockDataReady = true;
    assert(readImuSample(sample));
    assert(sample.accelerometerValid && sample.gyroscopeValid);
    assert(!sample.magnetometerValid);

    mockAgmt.magStat1 = 0x01;
    mockAgmt.magStat2 = 0x08;
    mockDataReady = true;
    assert(readImuSample(sample));
    assert(!sample.magnetometerValid);

    // Ошибка чтения не меняет опубликованный образец и не увеличивает sequence.
    const ImuSample beforeError = sample;
    mockReadStatus = ICM_20948_Stat_Err;
    mockDataReady = true;
    assert(!readImuSample(sample));
    assert(sample.sequence == beforeError.sequence);
    assert(sample.timestampUs == beforeError.timestampUs);
    assert(sample.accelerationMps2.x == beforeError.accelerationMps2.x);

    std::puts("readImuSample tests passed");
    return 0;
}
