#pragma once

#include <stdint.h>

#include "../geometry/AxisConfig.h"
#include "../geometry/ReferenceOrientation.h"
#include "DeadZone.h"
#include "../orientation/OrientationEstimator.h"
#include "../sensors/ImuDriver.h"

namespace gyrostab
{

// Набор уже настроенных параметров одного шага стабилизации.
struct ControllerConfig
{
    AxisConfig controlAxis;
    Vector3f gyroBiasRadPerSec;
    float maximumNormDeviation;
    uint32_t maximumSampleAgeUs;
    float proportionalGainUsPerRad;
    float derivativeGainUsPerRadPerSec;
    int servoCenterUs;
    int mechanicalDirection;
    int minimumPulseUs;
    int maximumPulseUs;
    float maximumChangeUsPerSecond;
    float deadZoneEnterThresholdRad;
    float deadZoneExitThresholdRad;
};

// Caller-owned состояние последнего IMU sample и гистерезиса dead-zone.
// Обнуляется инициализацией ControllerState state = {}.
struct ControllerState
{
    bool hasProcessedSample;
    uint32_t lastProcessedSampleSequence;
    DeadZoneState deadZone;
};

// Сбрасывает отметку sample и состояние dead-zone.
// Повторный вызов безопасен; параметры серво и PWM не затрагиваются.
void resetController(ControllerState& state);

// Вычисляет один P-D-шаг только по свежему, ещё не обработанному sample.
// Dead-zone подавляет только позиционную ошибку; gyro damping остаётся активным.
// OrientationEstimate должна быть получена именно из этого sample.
// currentPulseUs — последняя успешно отправленная команда; elapsedTimeUs —
// время с момента её отправки. Функция возвращает ограниченную цель, но
// не вызывает writeServoPulse() и не изменяет currentPulseUs.
// При отказе limitedPulseUs и state остаются без изменения.
bool runControlStep(const OrientationEstimate& currentOrientation,
                    const ReferenceOrientation& referenceOrientation,
                    const ImuSample& sample,
                    const ControllerConfig& config,
                    uint32_t nowUs,
                    int currentPulseUs,
                    uint32_t elapsedTimeUs,
                    ControllerState& state,
                    int& limitedPulseUs);

} // namespace gyrostab
