#include "RelativeOrientation.h"

#include <cmath>

namespace gyrostab
{

bool computeRelativeOrientation(const OrientationEstimate& current,
                                const ReferenceOrientation& reference,
                                float maximumNormDeviation,
                                Quaternionf& relativeQuaternion)
{
    if (!validateOrientation(current, maximumNormDeviation))
    {
        return false;
    }

    const OrientationEstimate referenceEstimate = {
        reference.quaternion,
        0U,
        0U,
        reference.valid
    };
    if (!validateOrientation(referenceEstimate, maximumNormDeviation))
    {
        return false;
    }

    const double referenceW = reference.quaternion.w;
    const double referenceX = reference.quaternion.x;
    const double referenceY = reference.quaternion.y;
    const double referenceZ = reference.quaternion.z;
    const double referenceNormSquared =
        referenceW * referenceW + referenceX * referenceX +
        referenceY * referenceY + referenceZ * referenceZ;

    // Используем полную обратную величину, а не только сопряжение:
    // валидатор допускает небольшую погрешность нормы исходных кватернионов.
    const double inverseW = referenceW / referenceNormSquared;
    const double inverseX = -referenceX / referenceNormSquared;
    const double inverseY = -referenceY / referenceNormSquared;
    const double inverseZ = -referenceZ / referenceNormSquared;

    const double currentW = current.quaternion.w;
    const double currentX = current.quaternion.x;
    const double currentY = current.quaternion.y;
    const double currentZ = current.quaternion.z;

    // q_relative = inverse(q_reference) * q_current.
    const double resultW =
        inverseW * currentW - inverseX * currentX -
        inverseY * currentY - inverseZ * currentZ;
    const double resultX =
        inverseW * currentX + inverseX * currentW +
        inverseY * currentZ - inverseZ * currentY;
    const double resultY =
        inverseW * currentY - inverseX * currentZ +
        inverseY * currentW + inverseZ * currentX;
    const double resultZ =
        inverseW * currentZ + inverseX * currentY -
        inverseY * currentX + inverseZ * currentW;

    const double resultNorm = std::sqrt(
        resultW * resultW + resultX * resultX +
        resultY * resultY + resultZ * resultZ);
    if (!std::isfinite(resultNorm) || resultNorm <= 0.0)
    {
        return false;
    }

    const Quaternionf candidate = {
        static_cast<float>(resultW / resultNorm),
        static_cast<float>(resultX / resultNorm),
        static_cast<float>(resultY / resultNorm),
        static_cast<float>(resultZ / resultNorm)
    };
    if (!std::isfinite(candidate.w) || !std::isfinite(candidate.x) ||
        !std::isfinite(candidate.y) || !std::isfinite(candidate.z))
    {
        return false;
    }

    relativeQuaternion = candidate;
    return true;
}

} // namespace gyrostab
