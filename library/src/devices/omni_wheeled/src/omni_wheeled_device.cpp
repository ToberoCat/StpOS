//
// Created by tobias on 01/10/25.
//

#include "libstp/device/omni_wheeled/omni_wheeled_device.h"
#include "libstp/_config.h"

#include <Eigen/Dense>


constexpr float radToRev = 1.0 / (2.0 * M_PI);
constexpr int maxMotorTicksPerSecond = 1500;


auto calculateWheelSpeeds(const double Vx, const double Vy, const double omega, const double R,
                          const double L)
{
    Eigen::MatrixXd inv_kinematics_matrix(4, 3);
    inv_kinematics_matrix << 1, 1, -L, // Front-right wheel (w1)
        1, -1, L, // Front-left wheel (w2)
        1, 1, L, // Back-left wheel (w3)
        1, -1, -L; // Back-right wheel (w4)

    Eigen::Vector3d velocities;
    velocities << Vx, Vy, omega;

    Eigen::Vector4d wheel_speeds = 1.0 / R * inv_kinematics_matrix * velocities;
    std::vector wheel_speeds_std(wheel_speeds.data(), wheel_speeds.data() + wheel_speeds.size());
    return wheel_speeds_std;
}


libstp::datatype::Speed libstp::device::omni_wheeled::OmniWheeledDevice::speedByWheels(
    const std::array<double, 4>& wheelSpeeds)
{
    double frontRight = wheelSpeeds[0];
    double frontLeft = wheelSpeeds[1];
    double rearLeft = wheelSpeeds[2];
    double rearRight = wheelSpeeds[3];

    float forwardSpeed = (frontRight + frontLeft + rearLeft + rearRight) * (wheelRadius / 4.0);
    float strafeSpeed = (-frontRight + frontLeft + rearLeft - rearRight) * (wheelRadius / 4.0);
    float angularSpeed = (-frontRight + frontLeft - rearLeft + rearRight) * (wheelRadius / (4.0 *
        wheelDistanceFromCenter));

    return {forwardSpeed, strafeSpeed, angularSpeed};
}

libstp::datatype::Speed libstp::device::omni_wheeled::OmniWheeledDevice::speedByWheelSides(double leftSpeed,
    double rightSpeed)
{
    float forwardSpeed = (leftSpeed + rightSpeed) * (wheelRadius / 2.0);
    float angularSpeed = (rightSpeed - leftSpeed) * (wheelRadius / (2.0 * wheelDistanceFromCenter));

    return {forwardSpeed, 0.0, angularSpeed};
}

float libstp::device::omni_wheeled::OmniWheeledDevice::getDrivenDistanceForward() const
{
    const double radPerTick = 2.0 * M_PI / ticksPerRevolution;

    const double deltaFrontRightTicks = frontRightMotor.getCurrentPositionEstimate() - initialFrontRightTicks;
    const double deltaFrontLeftTicks = frontLeftMotor.getCurrentPositionEstimate() - initialFrontLeftTicks;
    const double deltaRearLeftTicks = rearLeftMotor.getCurrentPositionEstimate() - initialRearLeftTicks;
    const double deltaRearRightTicks = rearRightMotor.getCurrentPositionEstimate() - initialRearRightTicks;

    const double theta1 = deltaFrontRightTicks * radPerTick; // Front-right
    const double theta2 = deltaFrontLeftTicks * radPerTick; // Front-left
    const double theta3 = deltaRearLeftTicks * radPerTick; // Rear-left
    const double theta4 = deltaRearRightTicks * radPerTick; // Rear-right

    const double Vx = (theta1 + theta2 + theta3 + theta4) * (wheelRadius / 4.0);
    return static_cast<float>(Vx);
}

float libstp::device::omni_wheeled::OmniWheeledDevice::getDrivenDistanceStrafe() const
{
    const double radPerTick = 2.0 * M_PI / ticksPerRevolution;

    const double deltaFrontRightTicks = frontRightMotor.getCurrentPositionEstimate() - initialFrontRightTicks;
    const double deltaFrontLeftTicks = frontLeftMotor.getCurrentPositionEstimate() - initialFrontLeftTicks;
    const double deltaRearLeftTicks = rearLeftMotor.getCurrentPositionEstimate() - initialRearLeftTicks;
    const double deltaRearRightTicks = rearRightMotor.getCurrentPositionEstimate() - initialRearRightTicks;

    const double theta1 = deltaFrontRightTicks * radPerTick; // Front-right
    const double theta2 = deltaFrontLeftTicks * radPerTick; // Front-left
    const double theta3 = deltaRearLeftTicks * radPerTick; // Rear-left
    const double theta4 = deltaRearRightTicks * radPerTick; // Rear-right

    const double Vy = (theta1 - theta2 + theta3 - theta4) * (wheelRadius / 4.0);
    return static_cast<float>(Vy);
}

void libstp::device::omni_wheeled::OmniWheeledDevice::applyKinematicsModel(const datatype::AbsoluteSpeed& speed)
{
    const auto wheelSpeeds = calculateWheelSpeeds(
        speed.forwardMs,
        speed.strafeMs,
        speed.angularRad,
        wheelRadius,
        wheelDistanceFromCenter
    );

    frontRightMotor.setVelocity(static_cast<int>(std::round(wheelSpeeds[0] * radToRev * ticksPerRevolution)));
    frontLeftMotor.setVelocity(static_cast<int>(std::round(wheelSpeeds[1] * radToRev * ticksPerRevolution)));
    rearLeftMotor.setVelocity(static_cast<int>(std::round(wheelSpeeds[2] * radToRev * ticksPerRevolution)));
    rearRightMotor.setVelocity(static_cast<int>(std::round(wheelSpeeds[3] * radToRev * ticksPerRevolution)));
}

std::tuple<float, float, float> libstp::device::omni_wheeled::OmniWheeledDevice::getWheelVelocities(
    const float dtSeconds)
{
    const auto radPerTick = 2.0 * M_PI / ticksPerRevolution;
    Eigen::Vector4d angularDisplacements;

    const auto currentFrontRightTicks = frontRightMotor.getCurrentPositionEstimate();
    const auto deltaFrontRightTicks = currentFrontRightTicks - lastFrontRightTicks;
    angularDisplacements[0] = deltaFrontRightTicks * radPerTick;
    lastFrontRightTicks = currentFrontRightTicks;

    const auto currentFrontLeftTicks = frontLeftMotor.getCurrentPositionEstimate();
    const auto deltaFrontLeftTicks = currentFrontLeftTicks - lastFrontLeftTicks;
    angularDisplacements[1] = deltaFrontLeftTicks * radPerTick;
    lastFrontLeftTicks = currentFrontLeftTicks;

    const auto currentRearLeftTicks = rearLeftMotor.getCurrentPositionEstimate();
    const auto deltaRearLeftTicks = currentRearLeftTicks - lastRearLeftTicks;
    angularDisplacements[2] = deltaRearLeftTicks * radPerTick;
    lastRearLeftTicks = currentRearLeftTicks;

    const auto currentRearRightTicks = rearRightMotor.getCurrentPositionEstimate();
    const auto deltaRearRightTicks = currentRearRightTicks - lastRearRightTicks;
    angularDisplacements[3] = deltaRearRightTicks * radPerTick;
    lastRearRightTicks = currentRearRightTicks;

    const Eigen::Vector4d angularVelocities = angularDisplacements / dtSeconds;

    Eigen::MatrixXd forwardKinematicsMatrix(3, 4);

    forwardKinematicsMatrix << 1, 1, 1, 1, // Vx
        1, -1, 1, -1, // Vy
        -wheelDistanceFromCenter, wheelDistanceFromCenter, wheelDistanceFromCenter, -wheelDistanceFromCenter; // Omega

    Eigen::Vector3d velocities = (wheelRadius / 4.0) * forwardKinematicsMatrix * angularVelocities;

    SPDLOG_TRACE("Velocities: Vx: {}, Vy: {}, Omega: {}", velocities[0], velocities[1], velocities[2]);
    SPDLOG_TRACE("FL: {}, FR: {}, RL: {}, RR: {}", deltaFrontLeftTicks, deltaFrontRightTicks, deltaRearLeftTicks,
                 deltaRearRightTicks);
    SPDLOG_TRACE("FL: {}, FR: {}, RL: {}, RR: {}", lastFrontLeftTicks, lastFrontRightTicks, lastRearLeftTicks,
                 lastRearRightTicks);
    return std::make_tuple(static_cast<float>(velocities[0]),
                           static_cast<float>(velocities[1]),
                           static_cast<float>(velocities[2])
    );
}

void libstp::device::omni_wheeled::OmniWheeledDevice::stopDevice()
{
    frontLeftMotor.stop();
    frontRightMotor.stop();
    rearLeftMotor.stop();
    rearRightMotor.stop();
}

std::tuple<float, float, float> libstp::device::omni_wheeled::OmniWheeledDevice::computeMaxSpeeds()
{
    const double w_max = (maxMotorTicksPerSecond / ticksPerRevolution) * 2.0 * M_PI; // rad/s

    auto Vx_max = static_cast<float>(w_max * wheelRadius); // m/s
    auto Vy_max = static_cast<float>(w_max * wheelRadius); // m/s
    auto omega_max = static_cast<float>(w_max * wheelRadius / wheelDistanceFromCenter); // rad/s

    return std::make_tuple(Vx_max, Vy_max, omega_max);
}

void libstp::device::omni_wheeled::OmniWheeledDevice::initializeKinematicDriveController()
{
    Device::initializeKinematicDriveController();

    lastFrontLeftTicks = frontLeftMotor.getCurrentPositionEstimate();
    initialFrontLeftTicks = lastFrontLeftTicks;

    lastFrontRightTicks = frontRightMotor.getCurrentPositionEstimate();
    initialFrontRightTicks = lastFrontRightTicks;

    lastRearLeftTicks = rearLeftMotor.getCurrentPositionEstimate();
    initialRearLeftTicks = lastRearLeftTicks;

    lastRearRightTicks = rearRightMotor.getCurrentPositionEstimate();
    initialRearRightTicks = lastRearRightTicks;
}

std::pair<float, float> libstp::device::omni_wheeled::OmniWheeledDevice::computeDrivenDistance() const
{
    const double radPerTick = 2.0 * M_PI / ticksPerRevolution;

    const double deltaFrontRightTicks = frontRightMotor.getCurrentPositionEstimate() - initialFrontRightTicks;
    const double deltaFrontLeftTicks = frontLeftMotor.getCurrentPositionEstimate() - initialFrontLeftTicks;
    const double deltaRearLeftTicks = rearLeftMotor.getCurrentPositionEstimate() - initialRearLeftTicks;
    const double deltaRearRightTicks = rearRightMotor.getCurrentPositionEstimate() - initialRearRightTicks;

    const double theta1 = deltaFrontRightTicks * radPerTick; // Front-right
    const double theta2 = deltaFrontLeftTicks * radPerTick; // Front-left
    const double theta3 = deltaRearLeftTicks * radPerTick; // Rear-left
    const double theta4 = deltaRearRightTicks * radPerTick; // Rear-right

    const double Vx = (theta1 + theta2 + theta3 + theta4) * (wheelRadius / 4.0);
    const double Vy = (theta1 - theta2 + theta3 - theta4) * (wheelRadius / 4.0);

    return std::make_pair(static_cast<float>(Vx), static_cast<float>(Vy));
}
