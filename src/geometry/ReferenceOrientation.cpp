#include "ReferenceOrientation.h"

namespace gyrostab
{

bool captureReferenceOrientation(const OrientationEstimate& estimate,
                                 float maximumNormDeviation,
                                 ReferenceOrientation& reference)
{
    if (!validateOrientation(estimate, maximumNormDeviation))
    {
        return false;
    }

    const ReferenceOrientation candidate = {estimate.quaternion, true};
    reference = candidate;
    return true;
}

} // namespace gyrostab
