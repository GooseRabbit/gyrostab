#include "../src/sensors/ImuDriver.h"
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

int main()
{
    using namespace gyrostab;

    // До первой попытки инициализации статус ещё неизвестен.
    assert(checkImuStatus() == ICM_20948_Stat_Unknown);

    // Ошибка запуска SPI становится видимой через status API.
    mockSpiBeginResult = false;
    assert(!beginImu());
    assert(checkImuStatus() == ICM_20948_Stat_Err);

    // Ошибка идентификатора датчика сохраняется из SparkFun begin().
    mockSpiBeginResult = true;
    mockBeginStatus = ICM_20948_Stat_WrongID;
    assert(!beginImu());
    assert(checkImuStatus() == ICM_20948_Stat_WrongID);

    // После успешной инициализации возвращается текущий статус библиотеки.
    mockBeginStatus = ICM_20948_Stat_Ok;
    assert(beginImu());
    assert(checkImuStatus() == ICM_20948_Stat_Ok);

    ImuConfig config = {};
    assert(configureImu(config));
    assert(checkImuStatus() == ICM_20948_Stat_Ok);

    // Отсутствие нового образца — это NoData, а не аппаратный отказ.
    ImuSample sample = {};
    mockDataReady = false;
    assert(!readImuSample(sample));
    assert(checkImuStatus() == ICM_20948_Stat_NoData);

    // Ошибка чтения сохраняется и доступна через checkImuStatus().
    mockDataReady = true;
    mockReadStatus = ICM_20948_Stat_Err;
    assert(!readImuSample(sample));
    assert(checkImuStatus() == ICM_20948_Stat_Err);

    // После успешного чтения статус снова становится OK.
    mockReadStatus = ICM_20948_Stat_Ok;
    mockDataReady = true;
    assert(readImuSample(sample));
    assert(checkImuStatus() == ICM_20948_Stat_Ok);

    std::puts("checkImuStatus tests passed");
    return 0;
}
