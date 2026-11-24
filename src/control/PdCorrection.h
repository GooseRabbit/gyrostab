#pragma once

namespace gyrostab
{

// Рассчитывает P-D-поправку deltaUs = Kp * errorRad - Kd * rateRadPerSec.
// Kp измеряется в мкс/рад, Kd — в мкс/(рад/с); коэффициенты неотрицательны.
// Направление механики и центр серво применяются на следующем этапе.
// При некорректных входах или непредставимом результате correctionUs не меняется.
bool computePdCorrection(float errorRad,
                         float rateRadPerSec,
                         float proportionalGainUsPerRad,
                         float derivativeGainUsPerRadPerSec,
                         float& correctionUs);

} // namespace gyrostab
