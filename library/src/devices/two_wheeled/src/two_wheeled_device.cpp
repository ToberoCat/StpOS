//
// Created by tobias on 12/26/24.
//

#include "libstp/device/two_wheeled/two_wheeled_device.h"

#include <cmath>
#include <chrono>
#include <memory>

#include "libstp/math/math.h"
#include "libstp/_config.h"
#include "libstp/motion/differential_drive.h"
#include "libstp/utility/constants.h"

namespace libstp::device::two_wheeled
{
    void TwoWheeledDevice::initializeKinematicDriveController()
    {
        Device::initializeKinematicDriveController();

        lastLeftTicks = leftMotor.getCurrentPositionEstimate();
        initialLeftTicks = lastLeftTicks;

        lastRightTicks = rightMotor.getCurrentPositionEstimate();
        initialRightTicks = lastRightTicks;
    }

    std::tuple<float, float, float> TwoWheeledDevice::computeMaxSpeeds()
    {
        const float vWheelMax = 2.0f * M_PIf * wheelRadius * utility::MAX_MOTOR_SPEED / ticksPerRevolution;
        const float omegaMax = 2.0f * vWheelMax / wheelBase;

        // Two wheeled devices only have linear and angular speeds, no strafing
        return std::make_tuple(vWheelMax, 0, omegaMax);
    }

    void TwoWheeledDevice::applyKinematicsModel(const datatype::AbsoluteSpeed& speed)
    {
        const auto vCMd = speed.forwardMs;
        const auto omegaCmd = speed.angularRad;
        const float leftMotorSpeed_mps = vCMd - omegaCmd * wheelBase / 2.0f;
        const float rightMotorSpeed_mps = vCMd + omegaCmd * wheelBase / 2.0f;

        
        SPDLOG_TRACE("Left Motor Speed: {}, Right Motor Speed: {}", leftMotorSpeed_mps, rightMotorSpeed_mps);

        const float leftMotorCmdTicks = leftMotorSpeed_mps / (2.0f * M_PIf * wheelRadius) * ticksPerRevolution;
        const float rightMotorCmdTicks = rightMotorSpeed_mps / (2.0f * M_PIf * wheelRadius) * ticksPerRevolution;

        leftMotor.setVelocity(static_cast<int>(leftMotorCmdTicks));
        rightMotor.setVelocity(static_cast<int>(rightMotorCmdTicks));
    }

    std::tuple<float, float, float> TwoWheeledDevice::getWheelVelocities(const float dtSeconds)
    {
        const auto currentRightTicks = rightMotor.getCurrentPositionEstimate();
        const auto deltaRightTicks = currentRightTicks - lastRightTicks;
        lastRightTicks = currentRightTicks;

        const auto currentLeftTicks = leftMotor.getCurrentPositionEstimate();
        const auto deltaLeftTicks = currentLeftTicks - lastLeftTicks;
        lastLeftTicks = currentLeftTicks;

        SPDLOG_TRACE("Delta Left Ticks: {}, Delta Right Ticks: {}", deltaLeftTicks, deltaRightTicks);
        const float leftRot = static_cast<float>(deltaLeftTicks) / ticksPerRevolution;
        const float rightRot = static_cast<float>(deltaRightTicks) / ticksPerRevolution;
        SPDLOG_TRACE("Left Rot: {}, Right Rot: {}", leftRot, rightRot);

        const float vLeft = 2.0f * M_PIf * wheelRadius * leftRot / dtSeconds;
        const float vRight = 2.0f * M_PIf * wheelRadius * rightRot / dtSeconds;
        SPDLOG_TRACE("VLeft: {}, VRight: {}", vLeft, vRight);
        
        const float vx = (vLeft + vRight) / 2.0f;
        const float omega = (vRight - vLeft) / wheelBase;
        SPDLOG_TRACE("Vx: {}, Omega: {}", vx, omega);
        return std::make_tuple(vx, 0, omega);
    }

    void TwoWheeledDevice::stopDevice()
    {
        leftMotor.stop();
        rightMotor.stop();
    }

    std::pair<float, float> TwoWheeledDevice::computeDrivenDistance() const
    {
        const auto deltaLeft = static_cast<float>(leftMotor.getCurrentPositionEstimate() - initialLeftTicks);
        const auto deltaRight = static_cast<float>(rightMotor.getCurrentPositionEstimate() - initialRightTicks);

        const float leftDistance = deltaLeft / ticksPerRevolution * 2.0f * M_PIf * wheelRadius;
        const float rightDistance = deltaRight / ticksPerRevolution * 2.0f * M_PIf * wheelRadius;

        const float Vx = (leftDistance + rightDistance) / 2.0f;
        const float Vy = 0.0f;

        return std::make_pair(Vx, Vy);
    }
}
