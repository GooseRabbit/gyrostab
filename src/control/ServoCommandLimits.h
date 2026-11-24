#pragma once

#include <stdint.h>

namespace gyrostab
{

// Собирает целевой импульс из центра серво и P-D-поправки,
// ограничивает его безопасными min/max и скоростью изменения.
// direction должен быть +1 или -1; центр и текущий импульс должны
// находиться внутри переданного безопасного диапазона.
// При отказе limitedPulseUs остаётся без изменения; PWM не выдаётся.
bool applyServoCommandLimits(float correctionUs,
                             int servoCenterUs,
                             int direction,
                             int currentPulseUs,
                             int minimumPulseUs,
                             int maximumPulseUs,
                             float maximumChangeUsPerSecond,
                             uint32_t elapsedTimeUs,
                             int& limitedPulseUs);

} // namespace gyrostab
