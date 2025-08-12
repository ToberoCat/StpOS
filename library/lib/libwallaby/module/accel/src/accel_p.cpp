#include "accel_p.hpp"
#include "kipr/core/platform.hpp"
#include "kipr/core/registers.hpp"
#include "kipr/time/time.h"

constexpr float sensitvity = 8192.0;
constexpr float gravity = 9.80665;
constexpr float scale_factor = gravity / sensitvity;

using namespace kipr;
using namespace kipr::accel;

using kipr::core::Platform;

namespace
{
    double Biasx = 0;
    double Biasy = 0;
    double Biasz = 0;
}

/*
 * Outputs the internal accel x reading
 * Pulls accelerometer data from the I2C registers. The MPU 9250 outputs high and low registers that need to be combined.
 */
float local_accel_x()
{
    return Platform::instance()->readRegisterFloat(REG_RW_ACCEL_X_0);
}

/*
 * Outputs the internal accel y reading
 * Pulls accelerometer data from the I2C registers. The MPU 9250 outputs high and low registers that need to be combined.
 */
float local_accel_y()
{
    return Platform::instance()->readRegisterFloat(REG_RW_ACCEL_Y_0);
}

/*
 * Outputs the internal accel z reading
 * Pulls accelerometer data from the I2C registers. The MPU 9250 outputs high and low registers that need to be combined.
 */
float local_accel_z()
{
    return Platform::instance()->readRegisterFloat(REG_RW_ACCEL_Z_0);
}

/*
 * Returns the accel_x in the NED frame. +x is forward (North). -x is backward (South).
 */
float kipr::accel::accel_x()
{
    return -local_accel_y();
}

/*
 * Returns the accel_y in the NED frame. +y is right (East). -y is left (West).
 */
float kipr::accel::accel_y()
{
    return local_accel_x();
}

/*
 * Returns the accel_z in the NED frame. +z is up. -z is down.
 */
float kipr::accel::accel_z()
{
    return local_accel_z();
}

// Simple low-pass filter for accelerometer
bool kipr::accel::accel_calibrate()
{
    int samples = 50;

    // Find the average noise
    int i = 0;
    double sumX = 0, sumY = 0, sumZ = 0;
    while (i < samples)
    {
        sumX += accel_z();
        sumY += accel_y();
        sumZ += accel_x();

        msleep(10);
        i++;
    }

    Biasx = sumX / samples;
    Biasy = sumY / samples;
    Biasz = sumZ / samples + 512; // Z axis should be detecting gravity

    printf("[Accel Calibration]: Bias Z: %f | Bias Y: %f | Bias X: %f \n", Biasz, Biasy, Biasx);
    return 0;
}
