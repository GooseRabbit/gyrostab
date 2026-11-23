#pragma once

#include "../orientation/OrientationEstimator.h"

namespace gyrostab
{

// Выводит одну CSV-строку с переданной оценкой ориентации.
// Частоту вызова определяет вызывающий код; функция не меняет оценку.
void publishOrientationTelemetry(const OrientationEstimate& estimate);

} // namespace gyrostab
