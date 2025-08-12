//
// Created by tobias on 12/27/24.
//
#include "libstp/motion/rotate.h"

libstp::async::AsyncAlgorithm<int> libstp::motion::rotate(device::Device& device, const datatype::ConditionalFunction& condition,
                            const datatype::SpeedFunction& speedFunction)
{
    return device.setSpeedWhile(condition, [speedFunction](const std::shared_ptr<datatype::ConditionalResult>& result)
    {
        return datatype::Speed(0, speedFunction(result).angularPercent);
    });
}
