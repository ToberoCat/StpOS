//
// Created by tobias on 12/26/24.
//

#pragma once
#include "motor.h"
#include "libstp/servo/servo.h"

namespace libstp::motor
{
    class ServoLikeMotor final : public Motor, public servo::Servo
    {
    public:
        explicit ServoLikeMotor(const int port, const bool reversePolarity = false) : Motor(port, reversePolarity),
            Servo(port)
        {
        }

        void disable() override;

        void enable() override;

        int getPosition() override;

        void setPosition(int position) override;

        void setPositionVelocity(int position, int velocity);
    };
}
