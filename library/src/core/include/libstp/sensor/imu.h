#pragma once

#include "libstp/async/algorithm.h"
#include <Eigen/Dense>
#include <memory>

namespace libstp::sensor
{
    using namespace Eigen;

    class GyroSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> getBias();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
    };

    class AccelSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> getBias();
        std::shared_ptr<Vector3d> getGravity();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
        std::shared_ptr<Vector3d> gravity = std::make_shared<Vector3d>(0.0, 0.0, 9.81);
    };

    class MagnetoSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        void setHardIronOffset(const std::shared_ptr<Vector3d>& newOffset);
        void setSoftIronMatrix(const std::shared_ptr<Matrix3d>& matrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
        std::shared_ptr<Matrix3d> softIronMatrix = std::make_shared<Matrix3d>(Matrix3d::Identity());
    };

    class IMU
    {
    public:
        IMU() = default;
        async::AsyncAlgorithm<int> calibrate(int sampleCount = 100);
        std::tuple<std::shared_ptr<Vector3d>, std::shared_ptr<Vector3d>, std::shared_ptr<Vector3d>> getReading() const;

        GyroSensor gyro;
        AccelSensor accel;
        MagnetoSensor magneto;
    };
}
