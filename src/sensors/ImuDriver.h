#pragma once

#include <ICM_20948.h>

#include "../geometry/Vector3.h"

namespace gyrostab
{

struct BoardConfig;

// Параметры акселерометра и гироскопа в формате библиотеки SparkFun.
// sampleRateDivider — делители частоты, а не значения частоты в герцах.
struct ImuConfig
{
    ICM_20948_fss_t fullScale;
    ICM_20948_smplrt_t sampleRateDivider;
    ICM_20948_dlpcfg_t dlpf;
    bool accelerometerDlpfEnabled;
    bool gyroscopeDlpfEnabled;
};

// Измерения одного нового пакета ICM-20948.
// timestampUs — время получения пакета микроконтроллером, не аппаратная метка IMU.
// Угловая скорость хранится в рад/с, ускорение — в м/с², магнитное поле — в мкТл.
struct ImuSample
{
    Vector3f accelerationMps2;
    Vector3f angularVelocityRadPerSec;
    Vector3f magneticFieldMicroTesla;
    uint32_t timestampUs;
    uint32_t sequence;
    bool accelerometerValid;
    bool gyroscopeValid;
    bool magnetometerValid;
};

// Запускает SPI и ICM-20948 по GPIO из подтверждённой BoardConfig.
// GPIO передаются в SPI.begin() как номера GPIO, а не номера Arduino-пинов.
// Возвращает false при неподтверждённой/некорректной распиновке или ошибке датчика.
bool beginImu(const BoardConfig& board);

// Применяет настройки акселерометра и гироскопа.
// Перед вызовом должен успешно завершиться beginImu().
// Возвращает false при недопустимых параметрах или ошибке библиотеки.
bool configureImu(const ImuConfig& config);

// Возвращает последний статус библиотеки SparkFun без дополнительного обмена SPI.
// Unknown означает, что beginImu() ещё не вызывалась; NoData — что нового образца пока нет.
ICM_20948_Status_e checkImuStatus();

// Считывает только новый образец акселерометра, гироскопа и магнитометра.
// Возвращает false, если драйвер не настроен, нового образца нет или чтение завершилось ошибкой.
// При false выходной sample остаётся без изменений.
bool readImuSample(ImuSample& sample);

} // namespace gyrostab
