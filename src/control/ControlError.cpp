#include "ControlError.h"

#include "../geometry/ControlAxisProjection.h"
#include "../geometry/RelativeOrientation.h"

namespace gyrostab
{

bool computeControlError(const OrientationEstimate& current,
                         const ReferenceOrientation& reference,
                         const AxisConfig& controlAxis,
                         float maximumNormDeviation,
                         float& errorRad)
{
    Quaternionf relativeQuaternion;
    if (!computeRelativeOrientation(current, reference, maximumNormDeviation,
                                    relativeQuaternion))
    {
        return false;
    }

    return projectErrorOnControlAxis(relativeQuaternion, controlAxis,
                                     maximumNormDeviation, errorRad);
}

} // namespace gyrostab
