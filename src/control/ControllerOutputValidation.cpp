#include "ControllerOutputValidation.h"

#include <cmath>
#include <limits>

namespace gyrostab
{

bool validateControllerOutput(float correctionUs)
{
    if (!std::isfinite(correctionUs))
    {
        return false;
    }

    const double candidate = static_cast<double>(correctionUs);
    return candidate >= static_cast<double>(std::numeric_limits<int>::lowest()) &&
           candidate <= static_cast<double>(std::numeric_limits<int>::max());
}

} // namespace gyrostab
