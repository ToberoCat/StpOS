//
// Created by tobias on 01/10/25.
//
#include "libstp/device/omni_wheeled/omni_wheeled_calibration.h"

#include "libstp/_config.h"
#include "libstp/sensor/sensor.h"
#include "libstp/math/math.h"

namespace libstp::device::omni_wheeled
{
    void calibrateTicksPerRevolution(const OmniWheeledDevice& device, float coveredDistance, int maxRetries)
    {
        if (coveredDistance <= 0.0f)
        {
            SPDLOG_ERROR("Covered distance must be > 0. Provided: {}", coveredDistance);
            return;
        }

        SPDLOG_INFO("Starting advanced calibration of ticks per revolution (omni).");

        float bestTicksPerRev = 0;
        bool success = false;

        for (int attempt = 1; attempt <= maxRetries; ++attempt)
        {
            // 1) Reset all four encoders
            device.frontLeftMotor.resetPositionEstimate();
            device.frontRightMotor.resetPositionEstimate();
            device.rearLeftMotor.resetPositionEstimate();
            device.rearRightMotor.resetPositionEstimate();

            SPDLOG_INFO("Attempt {}/{}. Move the robot forward exactly {:.2f} m. Then press the button.", 
                        attempt, maxRetries, coveredDistance);

            sensor::waitForButtonClick();

            // 2) Read final encoder ticks
            int fl = device.frontLeftMotor.getCurrentPositionEstimate();
            int fr = device.frontRightMotor.getCurrentPositionEstimate();
            int rl = device.rearLeftMotor.getCurrentPositionEstimate();
            int rr = device.rearRightMotor.getCurrentPositionEstimate();

            SPDLOG_INFO("Encoders => FL: {}, FR: {}, RL: {}, RR: {}", fl, fr, rl, rr);

            // 3) Average absolute value
            float avg = 0.25f * (std::fabs((float)fl) + std::fabs((float)fr)
                               + std::fabs((float)rl) + std::fabs((float)rr));
            if (avg < 1.0f)
            {
                SPDLOG_WARN("Average ticks are < 1. Possibly no movement? Retrying...");
                continue;
            }

            float ticksPerMeter = avg / coveredDistance;
            float wheelCircumference = 2.0f * M_PIf * device.wheelRadius;
            float newTicksPerRev = ticksPerMeter * wheelCircumference;

            // Check for reasonableness
            if (newTicksPerRev <= 0.0f || std::isinf(newTicksPerRev) || std::isnan(newTicksPerRev))
            {
                SPDLOG_ERROR("Computed invalid Ticks/Rev: {:.2f}. Retry...", newTicksPerRev);
                continue;
            }

            bestTicksPerRev = newTicksPerRev;
            success = true;
            SPDLOG_INFO("Success. Found ticks/rev = {:.2f}", newTicksPerRev);
            break;
        }

        if (success)
        {
            device.ticksPerRevolution = bestTicksPerRev;
            SPDLOG_INFO("Final Ticks per Revolution = {:.2f}", bestTicksPerRev);
        }
        else
        {
            SPDLOG_ERROR("Failed to calibrate ticks per revolution after {} attempts.", maxRetries);
        }
    }
}