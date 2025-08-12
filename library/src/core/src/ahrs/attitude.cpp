//
// Created by tobias on 1/11/25.
//

#include "libstp/ahrs/attitude.h"
#include "libstp/_config.h"

#include <tuple>
#include <stdexcept>
#include <cmath>
#include <iostream>

#include "kipr/gyro/gyro.h"
#include "libstp/math/math.h"
#include "libstp/sensor/imu.h"


constexpr float gravity = 9.81f;

namespace libstp::ahrs
{
    float yaw = 0.0f;
    // positive velocity: clockwise
    // negative velocity: counter-clockwise
    float AttitudeEstimator::getGyroReading(const sensor::IMU& imu) const
    {
        return static_cast<float>((*imu.gyro.getValue())[orientation]);
    }

    AttitudeEstimator::~AttitudeEstimator()
    {
        stopEstimation();
    }

    void AttitudeEstimator::setQuaternion(float w, float x, float y, float z)
    {
        //quaternion = Eigen::Quaternion(w, x, y, z);
        yaw = z;
    }

    AttitudeEstimator::AttitudeEstimator(const datatype::Axis axis) : orientation(axis) {
        //startEstimation();
    }

    float AttitudeEstimator::getCurrentHeading() const
    {
        return yaw;
        //return quaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2];
    }

    void AttitudeEstimator::startEstimation()
    {
        // if (task.running)
        //     return;
        //
        // SPDLOG_INFO("Starting attitude estimation");
        // task.startEstimation();
        // estimationThread = std::thread(&AttitudeEstimatorTask::estimationLoop, &task);
        // estimationThread.detach(); // Make it a daemon thread
    }

    void AttitudeEstimator::stopEstimation()
    {
        // SPDLOG_INFO("Stopping attitude estimation");
        // task.stopEstimation();
    }
}
