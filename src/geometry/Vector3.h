#pragma once

namespace gyrostab
{

// Трёхмерный вектор в декартовой системе координат.
// Единицы измерения задаются контекстом: например, ось безразмерна,
// угловая скорость может быть выражена в рад/с.
struct Vector3f
{
    float x;
    float y;
    float z;
};

// Нормирует input до единичной длины и записывает результат в normalized.
// Возвращает false для нулевого вектора или компонентов NaN/Inf.
// При ошибке normalized остаётся без изменений.
// Разрешено передавать один и тот же объект как input и normalized.
bool normalizeVector3(const Vector3f& input, Vector3f& normalized);

} // namespace gyrostab
