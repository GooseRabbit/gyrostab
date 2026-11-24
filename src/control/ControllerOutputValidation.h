#pragma once

namespace gyrostab
{

// Проверяет P-D-поправку до преобразования в целочисленную PWM-команду.
// Значение должно быть конечным и представимым типом int.
// Физические пределы серво проверяются отдельно в applyServoCommandLimits().
bool validateControllerOutput(float correctionUs);

} // namespace gyrostab
