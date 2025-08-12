//
// Created by tobias on 12/26/24.
//

#pragma once
#include "libstp/ahrs/attitude.h"
#include "libstp/async/algorithm.h"
#include "libstp/datatype/axis.h"
#include "libstp/datatype/speed.h"
#include "libstp/datatype/functions.h"
#include "libstp/utility/pid.h"

namespace libstp::device
{
    class Device
    {
        bool hasShutdown = false;
        datatype::Axis orientation;
        utility::PidParameters vxPidParameters, vyPidParameters, wPidParameters, headingPidParameters;
        ahrs::AttitudeEstimator attitudeEstimator;

        float forwardMaxAccel = 0.5f; // m/s^2
        float strafeMaxAccel = 0.5f; // m/s^2
        
        float angularMaxAccel = 1.0f; // rad/s^2

        float maxVx = 0.0, maxVy = 0.0, maxW = 0.0; // m/s, m/s, rad/s
    public:
        sensor::IMU imu;
        datatype::Direction direction;

        explicit Device(datatype::Axis orientation, datatype::Direction direction);

        virtual ~Device();
        void shutdown();

        void setVxPid(float kp, float ki, float kd);
        void setVyPid(float kp, float ki, float kd);
        void setWPid(float kp, float ki, float kd);
        void setHeadingPid(float kp, float ki, float kd);
        void setMaxAccel(float maxForwardAccel, float maxStrafeAccel, float maxAngularAccel);
        void setMaxSpeeds(float maxForwardSpeed, float maxStrafeSpeed, float maxAngularSpeed);
        void resetState() const;
        float getCurrentHeading();

        void setQuaternion(float w, float x, float y, float z)
        {
            attitudeEstimator.setQuaternion(w, x, y, z);
        }

        virtual void initializeKinematicDriveController();

        virtual std::tuple<float, float, float> computeMaxSpeeds()
        {
            throw std::runtime_error("Not implemented");
        }

        [[nodiscard]] virtual std::pair<float, float> computeDrivenDistance() const
        {
            throw std::runtime_error("Not implemented");
        }

        virtual void stopDevice()
        {
            throw std::runtime_error("Not implemented");
        }

        virtual void applyKinematicsModel(const datatype::AbsoluteSpeed& speed)
        {
            throw std::runtime_error("Not implemented");
        }

        virtual std::tuple<float, float, float> getWheelVelocities(float dtSeconds)
        {
            throw std::runtime_error("Not implemented");
        }

        void resetRamps() const;
        
        datatype::AbsoluteSpeed toAbsoluteSpeed(datatype::Speed speed, bool throttleMaxSpeed);

        async::AsyncAlgorithm<int> driveArc(
            datatype::ConditionalFunction condition,
            float radiusCentiMeters,
            float maxForwardPercentage,
            datatype::Direction direction);

        virtual async::AsyncAlgorithm<int> setSpeedWhile(datatype::ConditionalFunction condition,
                                                         datatype::SpeedFunction speedFunction,
                                                         bool doCorrection = true,
                                                         bool autoStopDevice = true,
                                                         bool resetRamps = true);

        async::AsyncAlgorithm<int> setSpeedWhile(datatype::ConditionalFunction condition,
                                                 datatype::Speed constantSpeed);

        void debugApplyKinematicsModel(float forward, float strafe, float angular)
        {
            applyKinematicsModel({forward, strafe, angular});
        }
    };
}
