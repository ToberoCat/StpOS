//
// Created by tobias on 01/10/25.
//

#pragma once

#include <cmath>
#include <tuple>
#include <utility>

#include "libstp/datatype/axis.h"
#include "libstp/device/device.h"
#include "libstp/math/math.h"
#include "libstp/motor/motor.h"

namespace libstp::device::omni_wheeled
{
    /**
     * @brief OmniWheeledDevice with 4 wheels at ~45-degree angles (Mecanum style) or another known geometry.
     * 
     * Now with:
     *  - Multi-axis PID (for vx, vy, w).
     *  - Inverse kinematics using a standard 4-wheel formula.
     *  - Expanded calibration routines for ticks, wheel base, and optionally wheel orientation offsets.
     */
    class OmniWheeledDevice final : public Device
    {
    public:
        motor::Motor frontLeftMotor;
        motor::Motor frontRightMotor;
        motor::Motor rearLeftMotor;
        motor::Motor rearRightMotor;

        int lastFrontLeftTicks = 0;
        int initialFrontLeftTicks = 0;

        int lastFrontRightTicks = 0;
        int initialFrontRightTicks = 0;

        int lastRearLeftTicks = 0;
        int initialRearLeftTicks = 0;

        int lastRearRightTicks = 0;
        int initialRearRightTicks = 0;

        // Robot geometry and encoder calibration
        // If your wheels are angled, each wheel might have a rotation offset or local transform.
        mutable float ticksPerRevolution = 1582.0f; // measure this
        mutable float wheelRadius = 0.035f; // measure this (m)
        mutable float wheelDistanceFromCenter = 0.1f; // measure this (m)

        OmniWheeledDevice(const datatype::Axis orientation,
                          const datatype::Direction direction,
                          const motor::Motor& flMotor,
                          const motor::Motor& frMotor,
                          const motor::Motor& rlMotor,
                          const motor::Motor& rrMotor)
            : Device(orientation, direction),
              frontLeftMotor(flMotor),
              frontRightMotor(frMotor),
              rearLeftMotor(rlMotor),
              rearRightMotor(rrMotor)
        {
        }

         async::AsyncAlgorithm<int> strafe(datatype::ConditionalFunction condition, datatype::Speed speed)
        {
            return setSpeedWhile(std::move(condition), [speed](const std::shared_ptr<datatype::ConditionalResult>& result)
            {
                return datatype::Speed{0.0f, speed.strafePercent, 0.0f};
            });
        }

        async::AsyncAlgorithm<int> strafe(datatype::ConditionalFunction condition, datatype::SpeedFunction speedFunction)
        {
            return setSpeedWhile(std::move(condition), [speedFunction](const std::shared_ptr<datatype::ConditionalResult>& result)
            {
                const auto speed = speedFunction(result);
                return datatype::Speed{0.0f, speed.strafePercent, 0.0f};
            });
        }

         async::AsyncAlgorithm<int> strafe(datatype::ConditionalFunction condition, const float& strafeAngle, const float& speedPercent)
        {
            const float angleRad = DEG_TO_RAD * strafeAngle;

            float vxPercent = speedPercent * std::cos(angleRad);
            float vyPercent = speedPercent * std::sin(angleRad);
            return setSpeedWhile(std::move(condition), datatype::constant({vxPercent, vyPercent, 0.0f}));
        }

         async::AsyncAlgorithm<int> strafe(datatype::ConditionalFunction condition, std::function<float()> strafeAngleFunction,
                    const float& speedPercent)
        {
            return setSpeedWhile(
                std::move(condition), [strafeAngleFunction, speedPercent](const std::shared_ptr<datatype::ConditionalResult>&)
                {
                    const float angleRad = DEG_TO_RAD * strafeAngleFunction();
                    const float vxPercent = speedPercent * std::cos(angleRad);
                    const float vyPercent = speedPercent * std::sin(angleRad);
                    return datatype::Speed{vxPercent, vyPercent, 0.0f};
                });
        }

        datatype::Speed speedByWheels(const std::array<double, 4>& wheelSpeeds);
        datatype::Speed speedByWheelSides(double leftSpeed, double rightSpeed);

        float getDrivenDistanceForward() const;
        float getDrivenDistanceStrafe() const;

    protected:
        void applyKinematicsModel(const datatype::AbsoluteSpeed& speed) override;
        std::tuple<float, float, float> getWheelVelocities(float dtSeconds) override;
        void stopDevice() override;
        std::tuple<float, float, float> computeMaxSpeeds() override;
        void initializeKinematicDriveController() override;
        [[nodiscard]] std::pair<float, float> computeDrivenDistance() const override;
    };
}
