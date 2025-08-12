//
// Created by tobias on 12/26/24.
//

#pragma once


namespace libstp::datatype
{
    struct Speed
    {
        // Forward is positive, backward is negative
        float forwardPercent; // -1.0 to 1.0

        // Left is positive, right is negative (Used for omni-wheeled devices only)
        float strafePercent; // -1.0 to 1.0

        // Counter-clockwise is positive, clockwise is negative
        float angularPercent; // -1.0 to 1.0

        Speed() : forwardPercent(0), strafePercent(0), angularPercent(0)
        {
        }

        Speed(const float forwardSpeedPercent, const float angularSpeedPercent) : forwardPercent(forwardSpeedPercent),
            strafePercent(0), angularPercent(angularSpeedPercent)
        {
        }

        Speed(const float forwardSpeedPercent, const float strafeSpeedPercent, const float angularSpeedPercent) :
            forwardPercent(forwardSpeedPercent), angularPercent(angularSpeedPercent), strafePercent(strafeSpeedPercent)
        {
        }

        [[nodiscard]] Speed backward() const
        {
            return {-forwardPercent, -strafePercent, -angularPercent};
        }

        static Speed wheels(const float left, const float right)
        {
            return {(left + right) / 2.0f, (right - left) / 2.0f};
        }

        static Speed stop()
        {
            return {0, 0};
        }

        static Speed Slowest;
        static Speed Slow;
        static Speed Medium;
        static Speed Fast;
        static Speed Fastest;
    };

    struct AbsoluteSpeed
    {
        float forwardMs; // m/s
        float strafeMs; // m/s
        float angularRad; // rad/s

        AbsoluteSpeed(const float forward_ms, const float strafe_ms, const float angular_rad)
            : forwardMs(forward_ms),
              strafeMs(strafe_ms),
              angularRad(angular_rad)
        {
        }
    };
}
