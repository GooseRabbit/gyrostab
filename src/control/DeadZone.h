#pragma once

namespace gyrostab
{

// Caller-owned состояние гистерезисной dead-zone.
struct DeadZoneState
{
    bool active;
};

// Обновляет состояние dead-zone по абсолютной позиционной ошибке в радианах.
// Неактивная зона включается при error <= enterThresholdRad.
// Активная зона выключается только при error > exitThresholdRad.
// exitThresholdRad должен быть строго больше enterThresholdRad.
// При некорректных входах состояние остаётся без изменения.
bool updateDeadZoneState(float absoluteErrorRad,
                         float enterThresholdRad,
                         float exitThresholdRad,
                         DeadZoneState& state);

} // namespace gyrostab
