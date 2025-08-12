//
// Created by tobias on 01/10/25.
//

#pragma once

#include "omni_wheeled_device.h"

namespace libstp::device::omni_wheeled
{
    /**
     * @brief Calibrate ticks per revolution for each wheel or as an average. 
     * 
     * This version lets the robot drive forward a known distance. 
     * We average across all four wheels, but you might measure each 
     * wheel individually if they differ.
     */
    void calibrateTicksPerRevolution(const OmniWheeledDevice& device, float coveredDistance, int maxRetries);
}
