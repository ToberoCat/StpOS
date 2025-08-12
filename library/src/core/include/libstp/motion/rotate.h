//
// Created by tobias on 12/26/24.
//

#pragma once
#include "libstp/device/device.h"

namespace libstp::motion
{
    async::AsyncAlgorithm<int> rotate(device::Device& device,
                const datatype::ConditionalFunction& condition,
                const datatype::SpeedFunction& speedFunction);
}
