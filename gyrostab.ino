#include <Arduino.h>

#include "src/config/SystemConfig.h"
#include "src/sensors/ImuDriver.h"

namespace
{
gyrostab::SystemConfig systemConfig = {};
}

// Минимальная точка входа Arduino.
// На этом этапе запускается только IMU; управление сервоприводом отключено.
void setup()
{
    Serial.begin(115200);

    gyrostab::loadDefaultConfig(systemConfig);

    if (gyrostab::beginImu(systemConfig.board))
    {
        Serial.println("ICM-20948 initialization: OK");
    }
    else
    {
        Serial.println("ICM-20948 initialization: FAILED");
    }
}

void loop()
{
    // Полный цикл управления будет подключён после target-сборки и проверки IMU.
}
