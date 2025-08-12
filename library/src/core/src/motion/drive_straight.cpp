//
// Created by tobias on 12/27/24.
//
#include <utility>

#include "libstp/motion/drive_straight.h"

libstp::async::AsyncAlgorithm<int> libstp::motion::drive_straight(device::Device& device, const datatype::ConditionalFunction& condition,
                                                                   const datatype::SpeedFunction& speedFunction)
{
    return device.setSpeedWhile(condition, [speedFunction](std::shared_ptr<datatype::ConditionalResult> result) {
        return datatype::Speed(speedFunction(std::move(result)).forwardPercent, 0);
    });
}
