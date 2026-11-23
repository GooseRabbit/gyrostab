#pragma once

#include "Vector3.h"

namespace gyrostab
{

// Ось стабилизации в системе координат самого IMU.
// Направление вектора задаёт положительное направление вращения
// по правилу правой руки. Физическое направление будет подтверждено
// отдельным измерением; здесь не предполагается конкретная ось X/Y/Z.
struct AxisConfig
{
    Vector3f axisInSensorFrame;
};

// Проверяет ось и возвращает конфигурацию с единичным вектором.
// Возвращает false для нулевых компонентов или NaN/Inf.
// При ошибке normalized остаётся без изменений.
// Разрешена передача одного объекта как input и normalized.
bool normalizeAxisConfig(const AxisConfig& input, AxisConfig& normalized);

} // namespace gyrostab
