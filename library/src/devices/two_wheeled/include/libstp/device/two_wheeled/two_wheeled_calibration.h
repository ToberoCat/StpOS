//
// Created by tobias on 12/28/24.
//

#pragma once
#include "two_wheeled_device.h"

namespace libstp::device::two_wheeled
{
    /**
     * Calibrate the ticks per revolution of the robot
     * @param device The two-wheeled device
     * @param coveredDistance The distance the robot has covered in meters
     * @param maxRetries The maximum number of retries
     */
    void calibrateTicksPerRevolution(const TwoWheeledDevice& device, float coveredDistance, int maxRetries);

    /**
     * Calibrate the wheelbase of the robot.
     * @param device The two-wheeled device
     * @param maxRetries The maximum number of retries
     */
    void calibrateWheelBase(const TwoWheeledDevice& device, int maxRetries);
}
