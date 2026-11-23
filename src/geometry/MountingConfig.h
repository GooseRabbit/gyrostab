#pragma once

#include "Vector3.h"

namespace gyrostab
{

// Оси платформы, выраженные в системе координат самого IMU.
// Вектор каждой оси задаёт её положительное направление.
struct MountingConfig
{
    Vector3f platformXInSensorFrame;
    Vector3f platformYInSensorFrame;
    Vector3f platformZInSensorFrame;
};

// Проверяет единичность, взаимную ортогональность и правую ориентацию осей.
// maximumDeviation задаёт допуск для длины осей и попарных скалярных произведений.
bool validateMountingConfig(const MountingConfig& config, float maximumDeviation);

} // namespace gyrostab
