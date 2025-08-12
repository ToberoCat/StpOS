//
// Created by tobias on 12/27/24.
//

#include "libstp/motion/line_follower.h"

#include "libstp/math/math.h"

libstp::async::AsyncAlgorithm<int> libstp::motion::follow_line(device::Device& device,
                                 sensor::LightSensor leftSensor,
                                 sensor::LightSensor rightSensor,
                                 const datatype::ConditionalFunction& condition,
                                 datatype::SpeedFunction speedFunction)
{
    return device.setSpeedWhile(
        condition, [&leftSensor, &rightSensor, &speedFunction](std::shared_ptr<datatype::ConditionalResult> result)
        {
            const auto currentSpeed = speedFunction(result);
            const float direction = math::signf(currentSpeed.forwardPercent);
            const bool leftBlack = leftSensor.isOnBlack();
            const bool rightBlack = rightSensor.isOnBlack();
            float angular = 0.0;
            if (leftBlack)
            {
                angular = direction * 0.26f;
            }
            else if (rightBlack)
            {
                angular = direction * -0.26f;
            }

            return datatype::Speed{currentSpeed.forwardPercent, angular};
        });
}
