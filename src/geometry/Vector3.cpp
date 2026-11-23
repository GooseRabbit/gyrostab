#include "Vector3.h"

#include <math.h>

namespace gyrostab
{

bool normalizeVector3(const Vector3f& input, Vector3f& normalized)
{
    // Не допускаем NaN и бесконечности в геометрические вычисления.
    if (!isfinite(input.x) || !isfinite(input.y) || !isfinite(input.z))
    {
        return false;
    }

    // Сначала масштабируем компоненты относительно максимума.
    // Это предотвращает переполнение/потерю точности при вычислении длины
    // для очень больших или очень маленьких, но конечных значений.
    float scale = fabsf(input.x);
    if (fabsf(input.y) > scale)
    {
        scale = fabsf(input.y);
    }
    if (fabsf(input.z) > scale)
    {
        scale = fabsf(input.z);
    }

    // Нулевой вектор не имеет определённого направления.
    if (scale == 0.0f)
    {
        return false;
    }

    const float scaledX = input.x / scale;
    const float scaledY = input.y / scale;
    const float scaledZ = input.z / scale;

    const float scaledLength = sqrtf(
        scaledX * scaledX +
        scaledY * scaledY +
        scaledZ * scaledZ);

    if (!isfinite(scaledLength) || scaledLength <= 0.0f)
    {
        return false;
    }

    // Записываем результат только после успешного завершения всех проверок.
    const Vector3f candidate = {
        scaledX / scaledLength,
        scaledY / scaledLength,
        scaledZ / scaledLength
    };

    if (!isfinite(candidate.x) || !isfinite(candidate.y) || !isfinite(candidate.z))
    {
        return false;
    }

    normalized = candidate;
    return true;
}

} // namespace gyrostab
