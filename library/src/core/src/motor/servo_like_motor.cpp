//
// Created by tobias on 12/26/24.
//
#include "libstp/motor/servo_like_motor.h"
#include "libstp/_config.h"

void libstp::motor::ServoLikeMotor::disable()
{
    stop();
}

void libstp::motor::ServoLikeMotor::enable()
{
    setVelocity(0);
}

int libstp::motor::ServoLikeMotor::getPosition()
{
    return getCurrentPositionEstimate();
}

void libstp::motor::ServoLikeMotor::setPosition(const int position)
{
    setPositionVelocity(position, 500);
}

void libstp::motor::ServoLikeMotor::setPositionVelocity(int position, int velocity)
{
    if (position < -2047 || position > 2047) {
        SPDLOG_ERROR("Position out of bounds: {}", position);
        return;
    }

    if (abs(getCurrentPositionEstimate() - position) < 5) {
        SPDLOG_WARN("Position already reached. At {}, targeting {}", getCurrentPositionEstimate(), position);
        return;
    }

    moveToTicks(position, velocity);
    enable();
}
