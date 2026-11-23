#include <Arduino.h>
#include "src/geometry/Vector3.h"
#include "src/sensors/ImuDriver.h"

// Минимальная точка входа Arduino.
// На этом этапе запускается только IMU; управление сервоприводом отключено.
void setup()
{
    Serial.begin(115200);

    if (gyrostab::beginImu())
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
    // Чтение измерений добавляется отдельной функцией readImuSample().
}
