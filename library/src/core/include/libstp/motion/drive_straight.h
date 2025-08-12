//
// Created by tobias on 12/26/24.
//
#pragma once

#include "libstp/datatype/functions.h"
#include "libstp/device/device.h"

namespace libstp::motion
{
    async::AsyncAlgorithm<int> drive_straight(device::Device& device,
                                               const datatype::ConditionalFunction& condition,
                                               const datatype::SpeedFunction& speedFunction);
}
