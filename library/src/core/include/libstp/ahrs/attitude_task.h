//
// Created by tobias on 2/9/25.
//

#pragma once
#include <atomic>
#include "ekf.h"

namespace libstp::ahrs
{
    class AttitudeEstimatorTask
    {
    public:
        std::atomic<bool> running{false};

        void calibrateBiases();
        void estimationLoop();
        void startEstimation();
        void stopEstimation();
        
    private:
        Eigen::Vector3d gyroOffset = Eigen::Vector3d::Zero();
        Eigen::Vector3d accelOffset = Eigen::Vector3d::Zero();
        Eigen::Vector3d magnetoOffset = Eigen::Vector3d::Zero();

        [[nodiscard]] Eigen::Vector3d applyMagnetoCalibration(const Eigen::Vector3d& mag) const;
        [[nodiscard]] Eigen::MatrixXd applyMagnetoCalibrationMatrix(const Eigen::MatrixXd& magMatrix) const;
        void getSensorReadings(Eigen::Vector3d& gyro, Eigen::Vector3d& accel, Eigen::Vector3d& mag) const;
        ExtendedKalmanFilter ekf;
    };
}
