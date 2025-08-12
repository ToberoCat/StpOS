//
// Created by tobias on 1/12/25.
//

#include "libstp/motion/differential_drive.h"

#include "../../include/libstp/_config.h"
#include "libstp/datatype/speed.h"
#include "libstp/device/device.h"
#include "libstp/math/math.h"
#include "libstp/utility/constants.h"

float fuseAngularVelocity(const float omega_gyro, const float omega_encoder, const float alpha = 0.98f)
{
    return alpha * omega_gyro + (1.0f - alpha) * omega_encoder;
}

std::tuple<float, float, float> libstp::motion::DifferentialDrive::measureVelocities(const float dtSeconds) const
{
    auto [vX, vY, omega] = device->getWheelVelocities(dtSeconds);
    SPDLOG_TRACE("Measured Wheel Velocities - Vx: {}, Vy: {}, Omega: {}", vX, vY, omega);

    auto fusedOmega = fuseMeasuredWheelSpeedWithAttitude(omega);
    SPDLOG_TRACE("Fused Omega: {}", fusedOmega);

    return std::make_tuple(vX, vY, fusedOmega);
}

float libstp::motion::DifferentialDrive::fuseMeasuredWheelSpeedWithAttitude(float omegaEncoder) const
{
    const auto omegaGyro = attitudeEstimator.getGyroReading(device->imu);
    SPDLOG_TRACE("Gyro Omega: {}, Encoder Omega: {}", omegaGyro, omegaEncoder);
    return omegaGyro;
}

float libstp::motion::DifferentialDrive::combineToOmega(
    datatype::AbsoluteSpeed absoluteSpeed,
    float correctionOmega,
    const float headingCorrection) const
{
    const auto hasRotation = std::fabs(absoluteSpeed.angularRad) > utility::EPSILON;
    const auto shouldTargetHeading = std::fabs(state.desiredHeading) > utility::EPSILON;
    if (hasRotation && !shouldTargetHeading)
    {
        SPDLOG_TRACE("Combining to AS + Correction: {} + {}", absoluteSpeed.angularRad, correctionOmega);
        return absoluteSpeed.angularRad + correctionOmega;
    }

    const auto computedFinalOmega = headingCorrection; //math::clampf(headingCorrection, -omega_cmd, omega_cmd);
    SPDLOG_TRACE("Combining to Heading Correction: {}", computedFinalOmega);
    return computedFinalOmega;
}

std::tuple<float, float, float> libstp::motion::DifferentialDrive::calculateControls(
    const datatype::AbsoluteSpeed absoluteSpeed,
    const float vx_meas,
    const float vy_meas,
    const float omega_meas)
{
    const float desiredHeading = state.desiredHeading * (device->direction == datatype::Direction::Forward ? 1.0f : -1.0f);
    const float angleDifference = math::minimalAngleDifference(desiredHeading, state.currentHeading);
    const float headingError = angleDifference * math::signf(desiredHeading - state.currentHeading);
    float headingCorrection = headingPid.calculate(headingError);
    SPDLOG_TRACE("Heading Correction: {}. Heading Error: {}. Angle Difference: {}, DirectDelta: {}", headingCorrection,
                 headingError,
                 angleDifference,
                 desiredHeading - state.currentHeading);

    const float errorVx = absoluteSpeed.forwardMs - vx_meas;
    const float errorVy = absoluteSpeed.strafeMs - vy_meas;
    const float errorOmega = absoluteSpeed.angularRad - omega_meas;
    SPDLOG_TRACE("Errors - Vx: {}, Vy: {}, Omega: {}", errorVx, errorVy, errorOmega);

    const float correctionVx = vXPid.calculate(errorVx);
    const float correctionVy = vYPid.calculate(errorVy);
    const float correctionOmega = wPid.calculate(errorOmega);
    SPDLOG_TRACE("Corrections - Vx: {}, Vy: {}, Omega: {}", correctionVx, correctionVy, correctionOmega);

    float finalVx = absoluteSpeed.forwardMs + correctionVx;
    float finalVy = absoluteSpeed.strafeMs + correctionVy;
    float finalOmega = combineToOmega(absoluteSpeed, correctionOmega, headingCorrection);

    return std::make_tuple(finalVx, finalVy, finalOmega);
}
