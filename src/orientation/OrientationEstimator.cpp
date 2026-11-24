#include "OrientationEstimator.h"

#include <cmath>

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

void resetOrientation(OrientationEstimate& estimate)
{
    initializeEstimator(estimate);
}

bool updateOrientation(OrientationEstimate& estimate, const ImuSample& sample,
                       const Vector3f& gyroBiasRadPerSec,
                       uint32_t maximumDeltaTimeUs)
{
    const uint32_t halfRange = 0x80000000UL;
    if (maximumDeltaTimeUs == 0U || maximumDeltaTimeUs >= halfRange ||
        !sample.gyroscopeValid ||
        !std::isfinite(sample.angularVelocityRadPerSec.x) ||
        !std::isfinite(sample.angularVelocityRadPerSec.y) ||
        !std::isfinite(sample.angularVelocityRadPerSec.z) ||
        !std::isfinite(gyroBiasRadPerSec.x) ||
        !std::isfinite(gyroBiasRadPerSec.y) ||
        !std::isfinite(gyroBiasRadPerSec.z))
    {
        return false;
    }

    const double qNorm = std::sqrt(
        static_cast<double>(estimate.quaternion.w) * estimate.quaternion.w +
        static_cast<double>(estimate.quaternion.x) * estimate.quaternion.x +
        static_cast<double>(estimate.quaternion.y) * estimate.quaternion.y +
        static_cast<double>(estimate.quaternion.z) * estimate.quaternion.z);
    if (!std::isfinite(qNorm) || qNorm <= 0.0)
    {
        return false;
    }

    const double qw = estimate.quaternion.w / qNorm;
    const double qx = estimate.quaternion.x / qNorm;
    const double qy = estimate.quaternion.y / qNorm;
    const double qz = estimate.quaternion.z / qNorm;

    if (!estimate.valid)
    {
        const OrientationEstimate candidate = {
            {static_cast<float>(qw), static_cast<float>(qx),
             static_cast<float>(qy), static_cast<float>(qz)},
            sample.timestampUs,
            sample.sequence,
            true
        };
        estimate = candidate;
        return true;
    }

    const uint32_t sequenceDelta = sample.sequence - estimate.sequence;
    const uint32_t deltaTimeUs = sample.timestampUs - estimate.timestampUs;
    if (sequenceDelta == 0U || sequenceDelta >= halfRange ||
        deltaTimeUs == 0U || deltaTimeUs >= halfRange ||
        deltaTimeUs > maximumDeltaTimeUs)
    {
        return false;
    }

    const double omegaX = static_cast<double>(sample.angularVelocityRadPerSec.x) - gyroBiasRadPerSec.x;
    const double omegaY = static_cast<double>(sample.angularVelocityRadPerSec.y) - gyroBiasRadPerSec.y;
    const double omegaZ = static_cast<double>(sample.angularVelocityRadPerSec.z) - gyroBiasRadPerSec.z;
    const double omegaMagnitude = std::sqrt(
        omegaX * omegaX + omegaY * omegaY + omegaZ * omegaZ);
    const double deltaTimeSeconds = static_cast<double>(deltaTimeUs) * 1.0e-6;
    const double halfAngle = 0.5 * omegaMagnitude * deltaTimeSeconds;

    double deltaW = 1.0;
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaZ = 0.0;
    if (omegaMagnitude > 0.0)
    {
        const double vectorScale = std::sin(halfAngle) / omegaMagnitude;
        deltaW = std::cos(halfAngle);
        deltaX = omegaX * vectorScale;
        deltaY = omegaY * vectorScale;
        deltaZ = omegaZ * vectorScale;
    }

    // q_new = q_old * delta_q for body-frame angular velocity.
    const double candidateW = qw * deltaW - qx * deltaX - qy * deltaY - qz * deltaZ;
    const double candidateX = qw * deltaX + qx * deltaW + qy * deltaZ - qz * deltaY;
    const double candidateY = qw * deltaY - qx * deltaZ + qy * deltaW + qz * deltaX;
    const double candidateZ = qw * deltaZ + qx * deltaY - qy * deltaX + qz * deltaW;
    const double candidateNorm = std::sqrt(
        candidateW * candidateW + candidateX * candidateX +
        candidateY * candidateY + candidateZ * candidateZ);
    if (!std::isfinite(candidateNorm) || candidateNorm <= 0.0)
    {
        return false;
    }

    const Quaternionf quaternion = {
        static_cast<float>(candidateW / candidateNorm),
        static_cast<float>(candidateX / candidateNorm),
        static_cast<float>(candidateY / candidateNorm),
        static_cast<float>(candidateZ / candidateNorm)
    };
    if (!std::isfinite(quaternion.w) || !std::isfinite(quaternion.x) ||
        !std::isfinite(quaternion.y) || !std::isfinite(quaternion.z))
    {
        return false;
    }

    const OrientationEstimate candidate = {
        quaternion,
        sample.timestampUs,
        sample.sequence,
        true
    };
    estimate = candidate;
    return true;
}

bool validateOrientation(const OrientationEstimate& estimate,
                         float maximumNormDeviation)
{
    if (!estimate.valid || !std::isfinite(maximumNormDeviation) || maximumNormDeviation < 0.0f ||
        !std::isfinite(estimate.quaternion.w) ||
        !std::isfinite(estimate.quaternion.x) ||
        !std::isfinite(estimate.quaternion.y) ||
        !std::isfinite(estimate.quaternion.z))
    {
        return false;
    }

    const double norm = std::sqrt(
        static_cast<double>(estimate.quaternion.w) * estimate.quaternion.w +
        static_cast<double>(estimate.quaternion.x) * estimate.quaternion.x +
        static_cast<double>(estimate.quaternion.y) * estimate.quaternion.y +
        static_cast<double>(estimate.quaternion.z) * estimate.quaternion.z);
    if (!std::isfinite(norm) || norm <= 0.0)
    {
        return false;
    }

    return std::fabs(norm - 1.0) <= static_cast<double>(maximumNormDeviation);
}

bool getOrientation(const OrientationEstimate& latestEstimate,
                    float maximumNormDeviation,
                    OrientationEstimate& output)
{
    if (!validateOrientation(latestEstimate, maximumNormDeviation))
    {
        return false;
    }

    output = latestEstimate;
    return true;
}

} // namespace gyrostab
