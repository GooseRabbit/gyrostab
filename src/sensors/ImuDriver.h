#pragma once

namespace gyrostab
{

// Запускает SPI и инициализирует ICM-20948 через библиотеку SparkFun.
// Возвращает true только при успешной инициализации датчика.
bool beginImu();

} // namespace gyrostab
