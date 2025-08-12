//
// Created by tobias on 1/11/25.
//

#pragma once
#include <atomic>
#include <thread>

#include "attitude_task.h"
#include "../datatype/axis.h"
#include "libstp/sensor/imu.h"

namespace libstp::ahrs
{
    class AttitudeEstimator
    {
    public:
        ~AttitudeEstimator();
        void setQuaternion(float w, float x, float y, float z);

        explicit AttitudeEstimator(datatype::Axis axis);
        
        [[nodiscard]] float getCurrentHeading() const;

        [[nodiscard]] float getGyroReading(const sensor::IMU& imu) const;

        void startEstimation();

        void stopEstimation();
    private:
        datatype::Axis orientation;
        AttitudeEstimatorTask task;

        std::thread estimationThread;
        Eigen::Quaternion<float> quaternion;
    };
}
