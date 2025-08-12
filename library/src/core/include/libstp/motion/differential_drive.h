//
// Created by tobias on 1/15/25.
//

#pragma once
#include "differential_drive_state.h"
#include "libstp/device/device.h"
#include "../ahrs/attitude.h"
#include "libstp/utility/pid.h"

namespace libstp::datatype
{
    struct AbsoluteSpeed;
}

namespace libstp::device
{
    class Device;
}

namespace libstp::motion
{
    class DifferentialDrive
    {
        const ahrs::AttitudeEstimator& attitudeEstimator;
        utility::PIDController vXPid, vYPid, wPid, headingPid;

    public:
        device::Device* device;
        DifferentialDriveState state = DifferentialDriveState([this]
        {
            return device->computeDrivenDistance();
        });

        DifferentialDrive(
            device::Device* device,
            const ahrs::AttitudeEstimator& attitude_estimator)
            : attitudeEstimator(attitude_estimator),
              vXPid(utility::PIDController()),
              vYPid(utility::PIDController()),
              wPid(utility::PIDController()),
              headingPid(utility::PIDController()),
              device(device)
        {
        }

        void setPIDParameters(
            const utility::PidParameters& vx_pid_parameters,
            const utility::PidParameters& vy_pid_parameters,
            const utility::PidParameters& w_pid_parameters,
            const utility::PidParameters& heading_pid_parameters)
        {
            vXPid.setParameters(vx_pid_parameters);
            vYPid.setParameters(vy_pid_parameters);
            wPid.setParameters(w_pid_parameters);
            headingPid.setParameters(heading_pid_parameters);
        }

        [[nodiscard]] std::tuple<float, float, float> measureVelocities(float dtSeconds) const;

        [[nodiscard]] float fuseMeasuredWheelSpeedWithAttitude(float omegaEncoder) const;

        [[nodiscard]] float combineToOmega(
            datatype::AbsoluteSpeed absoluteSpeed,
            float correctionOmega,
            float headingCorrection) const;

        std::tuple<float, float, float> calculateControls(
            datatype::AbsoluteSpeed absoluteSpeed,
            float vx_meas,
            float vy_meas,
            float omega_meas);
    };
}
