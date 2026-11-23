#include "OrientationEstimator.h"

namespace gyrostab
{

void initializeEstimator(OrientationEstimate& estimate)
{
    const OrientationEstimate initial = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        0U,
        0U,
        false
    };

    estimate = initial;
}

} // namespace gyrostab
