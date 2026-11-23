#include "AxisConfig.h"

namespace gyrostab
{

bool normalizeAxisConfig(const AxisConfig& input, AxisConfig& normalized)
{
    Vector3f unitAxis = {};

    // Повторно используем проверенную общую математику векторов.
    if (!normalizeVector3(input.axisInSensorFrame, unitAxis))
    {
        return false;
    }

    // Сначала полностью готовим результат, затем меняем выходную конфигурацию.
    const AxisConfig candidate = { unitAxis };
    normalized = candidate;
    return true;
}

} // namespace gyrostab
