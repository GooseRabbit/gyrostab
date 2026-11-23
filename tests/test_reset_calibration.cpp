#include "../src/calibration/CalibrationManager.h"

#include <cassert>
#include <cstdio>

namespace gyrostab
{

// CalibrationManager.cpp also contains detectStationary(); this test isolates
// resetCalibration() and provides the SensorManager dependency for linking.
bool isSampleFresh(const ImuSample&, uint32_t, uint32_t)
{
    return false;
}

} // namespace gyrostab

int main()
{
    using namespace gyrostab;

    Vector3f bias = {0.012f, -0.034f, 0.056f};
    bool calibrationValid = true;
    resetCalibration(bias, calibrationValid);
    assert(bias.x == 0.0f);
    assert(bias.y == 0.0f);
    assert(bias.z == 0.0f);
    assert(!calibrationValid);

    // Reset is idempotent, including when the calibration was already invalid.
    resetCalibration(bias, calibrationValid);
    assert(bias.x == 0.0f && bias.y == 0.0f && bias.z == 0.0f);
    assert(!calibrationValid);

    // A previously zero bias is still explicitly marked as not calibrated.
    bias = {0.0f, 0.0f, 0.0f};
    calibrationValid = true;
    resetCalibration(bias, calibrationValid);
    assert(!calibrationValid);

    std::puts("resetCalibration tests passed");
    return 0;
}
