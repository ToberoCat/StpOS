//
// Created by tobias on 12/26/24.
//

#include <utility>

#include "libstp/device/device.h"

#include <bits/cxxabi_tweaks.h>

#include "libstp/_config.h"

#include "kipr/motor/motor.h"
#include "kipr/servo/servo.h"
#include "libstp/motion/differential_drive.h"
#include "libstp/utility/timing.h"

std::unique_ptr<libstp::motion::DifferentialDrive> differentialDrive;

void libstp::device::Device::initializeKinematicDriveController()
{
}

libstp::device::Device::Device(const datatype::Axis orientation, const datatype::Direction direction):
    orientation(orientation), direction(direction),
    attitudeEstimator(ahrs::AttitudeEstimator(orientation))
{
    imu = sensor::IMU();
    differentialDrive = std::make_unique<motion::DifferentialDrive>(this, attitudeEstimator);
}

libstp::device::Device::~Device()
{
    shutdown();
}

void libstp::device::Device::shutdown()
{
    if (hasShutdown)
        return;

    hasShutdown = true;
    attitudeEstimator.stopEstimation();
    disable_servos();
    ao();
}

void libstp::device::Device::setVxPid(const float kp, const float ki, const float kd)
{
    vxPidParameters = utility::PidParameters(kp, ki, kd);
}

void libstp::device::Device::setVyPid(const float kp, const float ki, const float kd)
{
    vyPidParameters = utility::PidParameters(kp, ki, kd);
}

void libstp::device::Device::setWPid(const float kp, const float ki, const float kd)
{
    wPidParameters = utility::PidParameters(kp, ki, kd);
}

void libstp::device::Device::setHeadingPid(const float kp, const float ki, const float kd)
{
    headingPidParameters = utility::PidParameters(kp, ki, kd);
}

void libstp::device::Device::setMaxAccel(const float maxForwardAccel, const float maxStrafeAccel,
                                         const float maxAngularAccel)
{
    forwardMaxAccel = maxForwardAccel;
    strafeMaxAccel = maxStrafeAccel;
    angularMaxAccel = maxAngularAccel;
}

void libstp::device::Device::setMaxSpeeds(float maxForwardSpeed, float maxStrafeSpeed, float maxAngularSpeed)
{
    maxVx = maxForwardSpeed;
    maxVy = maxStrafeSpeed;
    maxW = maxAngularSpeed;
}

void libstp::device::Device::resetState() const
{
    const auto previousState = differentialDrive->state;
    differentialDrive->state = motion::DifferentialDriveState([this]
    {
        return computeDrivenDistance();
    });
    differentialDrive->state.rampedForwardMs = previousState.rampedForwardMs;
    differentialDrive->state.rampedStrafeMs = previousState.rampedStrafeMs;
    differentialDrive->state.rampedOmegaRad = previousState.rampedOmegaRad;
}

float libstp::device::Device::getCurrentHeading()
{
    return differentialDrive->state.currentHeading;
}

libstp::async::AsyncAlgorithm<int> libstp::device::Device::setSpeedWhile(datatype::ConditionalFunction condition,
                                                                         const datatype::Speed constantSpeed)
{
    SPDLOG_DEBUG("Set speed while called with constant speed ({}, {}, {})",
                 constantSpeed.forwardPercent, constantSpeed.strafePercent, constantSpeed.angularPercent);
    return setSpeedWhile(std::move(condition), constant(constantSpeed));
}


void libstp::device::Device::resetRamps() const
{
    differentialDrive->state.rampedForwardMs = 0.0f;
    differentialDrive->state.rampedStrafeMs = 0.0f;
    differentialDrive->state.rampedOmegaRad = 0.0f;
}

libstp::datatype::AbsoluteSpeed libstp::device::Device::toAbsoluteSpeed(datatype::Speed speed, bool throttleMaxSpeed)
{
    auto [maxForward, maxStrafe, maxAngular] = computeMaxSpeeds();
    if (throttleMaxSpeed)
    {
        maxForward *= 0.95;
        maxStrafe *= 0.95;
        maxAngular *= 0.95;
    }
    return {
        speed.forwardPercent * maxForward,
        speed.strafePercent * maxStrafe,
        speed.angularPercent * maxAngular
    };
}

libstp::async::AsyncAlgorithm<int> libstp::device::Device::driveArc(
    datatype::ConditionalFunction condition,
    const float radiusCentiMeters, const float maxForwardPercentage, const datatype::Direction direction)
{
    const auto [maxForward, maxStrafe, maxAngular] = computeMaxSpeeds();

    const float radius = radiusCentiMeters / 100.0f;
    const float maxLinearSpeed = maxForward * maxForwardPercentage;

    return setSpeedWhile(
        std::move(condition),
        [=](const std::shared_ptr<datatype::ConditionalResult>&)
        {
            const float omega = maxLinearSpeed / radius;
            float angularSpeedPercent = omega / maxAngular;
            const float forwardSpeedPercent = std::clamp(maxForwardPercentage, -1.0f, 1.0f);
            angularSpeedPercent = std::clamp(angularSpeedPercent, -1.0f, 1.0f);

            return datatype::Speed{forwardSpeedPercent, 0.0f, angularSpeedPercent};
        }
    );
}

libstp::async::AsyncAlgorithm<int> libstp::device::Device::setSpeedWhile(
    const datatype::ConditionalFunction condition,
    const datatype::SpeedFunction speedFunction,
    const bool doCorrection,
    const bool autoStopDevice,
    const bool resetRamps)
{
    initializeKinematicDriveController();
    auto lastTime = std::chrono::steady_clock::now();

    SPDLOG_TRACE("Max Speeds - Vx: {}, Vy: {}, Omega: {}", vWheelMax, strafeMax, omegaMax);
    differentialDrive->setPIDParameters(vxPidParameters, vyPidParameters, wPidParameters, headingPidParameters);
    differentialDrive->state.currentHeading -= differentialDrive->state.desiredHeading;
    differentialDrive->state.desiredHeading = 0;

    if (resetRamps)
    {
        differentialDrive->state.rampedForwardMs = 0.0f;
        differentialDrive->state.rampedStrafeMs = 0.0f;
        differentialDrive->state.rampedOmegaRad = 0.0f;
    }
    while (true)
    {
        SPDLOG_TRACE("Condition Loop Running");
        if (!condition)
        {
            SPDLOG_ERROR("Condition function is null");
            break;
        }

        const auto conditionResult = condition(false);
        if (!conditionResult)
        {
            SPDLOG_ERROR("Condition result is null");
            break;
        }

        conditionResult->update(differentialDrive->state);

        SPDLOG_TRACE("Condition Loop Running: {}", conditionResult->is_loop_running());
        if (!conditionResult->is_loop_running())
        {
            SPDLOG_TRACE("Condition met to exit loop.");
            break;
        }

        SPDLOG_TRACE("Condition Loop Running: {}", conditionResult->is_loop_running());
        if (speedFunction == nullptr)
        {
            SPDLOG_ERROR("Speed function is null");
            break;
        }

        const auto desiredSpeed = speedFunction(conditionResult);
        SPDLOG_TRACE("received desired speed");
        SPDLOG_TRACE("Desired Speed - Vx: {}, Vy: {}, Omega: {}", desiredSpeed.forwardPercent,
                     desiredSpeed.strafePercent, desiredSpeed.angularPercent);

        const auto absoluteSpeed = toAbsoluteSpeed(desiredSpeed, doCorrection);

        SPDLOG_TRACE("Absolute Speed - Vx: {}, Vy: {}, Omega: {}", absoluteSpeed.forwardMs, absoluteSpeed.strafeMs,
                     absoluteSpeed.angularRad);

        const auto now = std::chrono::steady_clock::now();
        const float dtSeconds = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        SPDLOG_TRACE("Delta Time: {}", dtSeconds);

        auto [vx_meas, vy_meas, omega_meas] = differentialDrive->measureVelocities(dtSeconds);
        SPDLOG_TRACE("Measured Velocities - Vx: {}, Vy: {}, Omega: {}", vx_meas, vy_meas, omega_meas);

        // Only update heading with gyro data if doCorrection is true
        if (doCorrection)
        {
            differentialDrive->state.currentHeading += omega_meas * dtSeconds;
            SPDLOG_TRACE("Heading: {}, Desired Heading: {}",
                         differentialDrive->state.currentHeading,
                         differentialDrive->state.desiredHeading
            );
        }

        {
            float forwardDelta = absoluteSpeed.forwardMs - differentialDrive->state.rampedForwardMs;
            float maxForwardDelta = forwardMaxAccel * dtSeconds;
            forwardDelta = std::clamp(forwardDelta, -maxForwardDelta, maxForwardDelta);
            differentialDrive->state.rampedForwardMs += forwardDelta;

            float strafeDelta = absoluteSpeed.strafeMs - differentialDrive->state.rampedStrafeMs;
            float maxStrafeDelta = strafeMaxAccel * dtSeconds;
            strafeDelta = std::clamp(strafeDelta, -maxStrafeDelta, maxStrafeDelta);
            differentialDrive->state.rampedStrafeMs += strafeDelta;

            float omegaDelta = absoluteSpeed.angularRad - differentialDrive->state.rampedOmegaRad;
            float maxOmegaDelta = angularMaxAccel * dtSeconds;
            omegaDelta = std::clamp(omegaDelta, -maxOmegaDelta, maxOmegaDelta);
            differentialDrive->state.rampedOmegaRad += omegaDelta;
        }

        const datatype::AbsoluteSpeed rampedSpeed(
            differentialDrive->state.rampedForwardMs,
            differentialDrive->state.rampedStrafeMs,
            differentialDrive->state.rampedOmegaRad
        );

        float finalVx, finalVy, finalOmega;

        if (doCorrection)
        {
            // Apply full correction with gyro data
            std::tie(finalVx, finalVy, finalOmega) = differentialDrive->calculateControls(
                rampedSpeed,
                vx_meas, vy_meas,
                omega_meas
            );
        }
        else
        {
            // Use only encoder feedback, skip heading correction
            // Use ramped speed directly for omega when no gyro correction
            finalVx = rampedSpeed.forwardMs;
            finalVy = rampedSpeed.strafeMs;
            finalOmega = rampedSpeed.angularRad;

            // Still apply PID for encoder-based movement but not for heading
            std::tie(finalVx, finalVy, std::ignore) = differentialDrive->calculateControls(
                rampedSpeed,
                vx_meas, vy_meas,
                0.0f // Ignore angular velocity correction
            );
        }

        SPDLOG_TRACE("Final Commands - Vx: {}, Vy: {}, Omega: {}", finalVx, finalVy, finalOmega);

        if (maxVx != 0 && maxVy != 0 && maxW != 0)
        {
            // Clamp final speeds to max values
            finalVx = std::clamp(finalVx, -maxVx, maxVx);
            finalVy = std::clamp(finalVy, -maxVy, maxVy);
            finalOmega = std::clamp(finalOmega, -maxW, maxW);
        }
        
        // +vx => forward
        // -vx => backward
        // +vy => right
        // -vy => left
        // +omega => clockwise
        // -omega => counterclockwise
        applyKinematicsModel(datatype::AbsoluteSpeed(finalVx, finalVy, finalOmega));

        SPDLOG_TRACE("==============================================");
        co_yield 1;
    }

    if (autoStopDevice)
    {
        SPDLOG_DEBUG("Stopping device after setSpeedWhile loop");
        stopDevice();
    }
}
