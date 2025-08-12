//
// Created by tobias on 12/28/24.
//

#include "libstp/device/two_wheeled/two_wheeled_calibration.h"

#include "libstp/_config.h"
#include "libstp/sensor/sensor.h"
#include "libstp/math/math.h"

void libstp::device::two_wheeled::calibrateTicksPerRevolution(const TwoWheeledDevice& device, float coveredDistance,
                                                              int maxRetries)
{
    if (coveredDistance <= 0.0f)
    {
        SPDLOG_ERROR("Covered distance must be greater than zero. Provided: {}", coveredDistance);
        return;
    }

    if (device.ticksPerRevolution > 0.0f)
    {
        SPDLOG_WARN("Ticks per revolution is already calibrated: {}. Overwriting.", device.ticksPerRevolution);
    }

    SPDLOG_INFO("Starting calibration of ticks per revolution.");

    for (int attempt = 1; attempt <= maxRetries; ++attempt)
    {
        device.leftMotor.resetPositionEstimate();
        device.rightMotor.resetPositionEstimate();

        SPDLOG_INFO("Calibration Attempt {}/{}", attempt, maxRetries);
        SPDLOG_INFO("Please move the robot forward for {:.2f} meters.", coveredDistance);
        SPDLOG_INFO("Ensure both wheels rotate smoothly without obstruction.");
        SPDLOG_INFO("Press the button once the robot has moved the specified distance.");

        sensor::waitForButtonClick();

        const auto left_ticks = device.leftMotor.getCurrentPositionEstimate();
        const auto right_ticks = device.rightMotor.getCurrentPositionEstimate();
        SPDLOG_INFO("Encoder Readings - Left: {}, Right: {}", left_ticks, right_ticks);

        // Invalid encoder ticks if not the same sign
        if (math::sign(left_ticks) != math::sign(right_ticks))
        {
            SPDLOG_ERROR("Invalid encoder ticks detected. Left: {}, Right: {}. Please retry.", left_ticks, right_ticks);
            continue;
        }

        const float left_ticks_per_meter = static_cast<float>(abs(left_ticks)) / coveredDistance;
        const float right_ticks_per_meter = static_cast<float>(abs(right_ticks)) / coveredDistance;
        SPDLOG_INFO("Ticks per Meter - Left: {:.2f}, Right: {:.2f}", left_ticks_per_meter, right_ticks_per_meter);

        const float wheelCircumference = 2.0f * static_cast<float>(M_PI) * device.wheelRadius; // meters
        const float left_ticks_per_revolution = left_ticks_per_meter * wheelCircumference;
        const float right_ticks_per_revolution = right_ticks_per_meter * wheelCircumference;
        SPDLOG_INFO("Ticks per Revolution - Left: {:.2f}, Right: {:.2f}", left_ticks_per_revolution,
                    right_ticks_per_revolution);

        const float difference = std::abs(left_ticks_per_revolution - right_ticks_per_revolution);
        if (const float tolerance = 0.05f * ((left_ticks_per_revolution + right_ticks_per_revolution) / 2.0f);
            difference > tolerance)
        {
            SPDLOG_ERROR("Discrepancy between left and right ticks per revolution exceeds tolerance.");
            SPDLOG_ERROR("Difference: {:.2f}, Tolerance: {:.2f}. Please ensure both wheels are functioning correctly.",
                         difference, tolerance);
            continue;
        }

        device.ticksPerRevolution = (left_ticks_per_revolution + right_ticks_per_revolution) / 2.0f;
        SPDLOG_INFO("Calibration Successful. Final Ticks per Revolution: {:.2f}", device.ticksPerRevolution);
        return;
    }

    SPDLOG_ERROR("Calibration failed after {} attempts. Please check the robot's hardware and try again.", maxRetries);
}

void libstp::device::two_wheeled::calibrateWheelBase(const TwoWheeledDevice& device, int maxRetries)
{
    if (device.ticksPerRevolution == 0)
    {
        SPDLOG_ERROR("Ticks per revolution must be calibrated first.");
        return;
    }

    SPDLOG_INFO("Calibrating wheel base.");
    int attempt = 0;

    while (attempt < maxRetries)
    {
        device.leftMotor.resetPositionEstimate();
        device.rightMotor.resetPositionEstimate();

        SPDLOG_INFO("Calibration Attempt {}/{}", attempt + 1, maxRetries);
        SPDLOG_INFO("Please lock the right wheel so it does not move.");
        SPDLOG_INFO("Rotate the robot 360 degrees by moving the left wheel.");
        SPDLOG_INFO("Press the button once the rotation is complete.");
        sensor::waitForButtonClick();

        const auto left_ticks = device.leftMotor.getCurrentPositionEstimate();
        const auto right_ticks = device.rightMotor.getCurrentPositionEstimate();
        SPDLOG_INFO("Left ticks: {}, Right ticks: {}", left_ticks, right_ticks);

        if (left_ticks <= 0)
        {
            SPDLOG_ERROR("Invalid left encoder ticks: {}. Ensure the left wheel was rotated.", left_ticks);
            attempt++;
            continue;
        }

        if (right_ticks > left_ticks * 0.1)
        {
            SPDLOG_ERROR("Right wheel should not move during calibration. Detected ticks: {}. Retry...", right_ticks);
            attempt++;
            continue;
        }

        const float left_revolutions = static_cast<float>(left_ticks) / device.ticksPerRevolution;
        const float distance_traveled = 2.0f * M_PIf * device.wheelRadius * left_revolutions;

        device.wheelBase = distance_traveled / (2.0f * M_PIf);

        SPDLOG_INFO("Calibration successful. Calculated wheel base: {:.4f} meters.", device.wheelBase);
        return;
    }

    SPDLOG_ERROR("Wheel base calibration failed after {} attempts.", maxRetries);
}
