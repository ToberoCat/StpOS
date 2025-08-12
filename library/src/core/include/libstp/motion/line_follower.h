//
// Created by tobias on 12/26/24.
//

#pragma once

#include "libstp/datatype/functions.h"
#include "libstp/device/device.h"
#include "libstp/sensor/sensor.h"

namespace libstp::motion
{
    async::AsyncAlgorithm<int> follow_line(device::Device& device,
                     sensor::LightSensor leftSensor,
                     sensor::LightSensor rightSensor,
                     const datatype::ConditionalFunction& condition,
                     datatype::SpeedFunction speedFunction);
}
