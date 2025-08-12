//
// Created by tobias on 12/26/24.
//

#pragma once
#include "libstp/device/device.h"
#include "libstp/sensor/sensor.h"

namespace libstp::motion
{
    async::AsyncAlgorithm<int> forward_line_up(device::Device& device,
                         sensor::LightSensor& left_sensor,
                         sensor::LightSensor& right_sensor);

    async::AsyncAlgorithm<int> backward_line_up(device::Device& device,
                          sensor::LightSensor& left_sensor,
                          sensor::LightSensor& right_sensor);
}
