#pragma once

#include <ICM_20948.h>

namespace gyrostab
{

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

// Запускает SPI и инициализирует ICM-20948 через библиотеку SparkFun.
// Возвращает true только при успешной инициализации датчика.
bool beginImu();

// Применяет настройки акселерометра и гироскопа.
// Перед вызовом должен успешно завершиться beginImu().
// Возвращает false при недопустимых параметрах или ошибке библиотеки.
bool configureImu(const ImuConfig& config);

} // namespace gyrostab
