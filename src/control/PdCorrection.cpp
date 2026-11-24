#include "PdCorrection.h"

#include <cmath>
#include <limits>

namespace gyrostab
{

bool computePdCorrection(float errorRad,
                         float rateRadPerSec,
                         float proportionalGainUsPerRad,
                         float derivativeGainUsPerRadPerSec,
                         float& correctionUs)
{
    if (!std::isfinite(errorRad) ||
        !std::isfinite(rateRadPerSec) ||
        !std::isfinite(proportionalGainUsPerRad) ||
        !std::isfinite(derivativeGainUsPerRadPerSec) ||
        proportionalGainUsPerRad < 0.0f ||
        derivativeGainUsPerRadPerSec < 0.0f)
    {
        return false;
    }

    const double proportionalCorrection =
        static_cast<double>(proportionalGainUsPerRad) *
        static_cast<double>(errorRad);
    const double derivativeCorrection =
        static_cast<double>(derivativeGainUsPerRadPerSec) *
        static_cast<double>(rateRadPerSec);
    const double candidateCorrection =
        proportionalCorrection - derivativeCorrection;

    if (!std::isfinite(candidateCorrection) ||
        std::fabs(candidateCorrection) >
            static_cast<double>(std::numeric_limits<float>::max()))
    {
        return false;
    }

    const float candidate = static_cast<float>(candidateCorrection);
    if (!std::isfinite(candidate))
    {
        return false;
    }

    correctionUs = candidate;
    return true;
}

} // namespace gyrostab
