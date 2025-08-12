//
// Created by tobias on 12/26/24.
//

#include "libstp/sensor/imu.h"

#include "kipr/accel/accel.h"
#include "kipr/gyro/gyro.h"
#include "kipr/magneto/magneto.h"
#include "libstp/math/math.h"
#include "libstp/_config.h"

void libstp::sensor::GyroSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    const int rows = calibrationMatrix->rows();
    const int cols = calibrationMatrix->cols();

    auto median = std::make_shared<Vector3d>();
    for (int i = 0; i < cols; ++i)
    {
        std::vector<double> colData(rows);
        for (int j = 0; j < rows; ++j)
        {
            colData[j] = (*calibrationMatrix)(j, i);
        }
        std::ranges::sort(colData);
        (*median)[i] = (rows % 2 == 0) ? (colData[rows / 2 - 1] + colData[rows / 2]) / 2.0 : colData[rows / 2];
    }

    offset = median;

    auto applied = std::make_shared<MatrixX3d>(rows, 3);
    for (int i = 0; i < rows; ++i)
    {
        applied->row(i) = calibrationMatrix->row(i) - offset->transpose();
    }

    variance = std::make_shared<Vector3d>(applied->array().square().colwise().mean());

    SPDLOG_INFO("[IMU] Calibrated gyro sensor with bias: ({}, {}, {}), variance: ({}, {}, {})", 
                (*offset)[0], (*offset)[1], (*offset)[2], 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getValue() const
{
    const auto rawGyro = std::make_shared<Vector3d>(
        gyro_x() * DEG_TO_RAD,
        gyro_y() * DEG_TO_RAD,
        gyro_z() * DEG_TO_RAD
        );
    return applyCalibration(rawGyro);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getBias()
{
    return offset;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    return std::make_shared<Vector3d>(*sample - *offset);
}

void libstp::sensor::AccelSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    const int rows = calibrationMatrix->rows();
    const int cols = calibrationMatrix->cols();

    auto median = std::make_shared<Vector3d>();
    for (int i = 0; i < cols; ++i)
    {
        std::vector<double> colData(rows);
        for (int j = 0; j < rows; ++j)
        {
            colData[j] = (*calibrationMatrix)(j, i);
        }
        std::ranges::sort(colData);
        (*median)[i] = (rows % 2 == 0) ? (colData[rows / 2 - 1] + colData[rows / 2]) / 2.0 : colData[rows / 2];
    }

    offset = median;

    int gravity_axis;
    offset->maxCoeff(&gravity_axis);
    const double gravity_sign = ((*offset)[gravity_axis] > 0) ? 1.0 : -1.0;
    (*offset)[gravity_axis] -= 9.81 * gravity_sign;

    gravity = std::make_shared<Vector3d>(Vector3d::Zero());
    (*gravity)[gravity_axis] = 9.81 * gravity_sign;

    SPDLOG_INFO("[IMU] Detected gravity axis: {} ({})", gravity_axis, gravity_sign > 0 ? "+" : "-");

    auto diffMatrix = std::make_shared<MatrixX3d>(rows, 3);
    for (int i = 0; i < rows; ++i) {
        diffMatrix->row(i) = calibrationMatrix->row(i) - offset->transpose();
    }
    variance = std::make_shared<Vector3d>(diffMatrix->array().square().colwise().mean());
    
    SPDLOG_INFO("[IMU] Calibrated accel sensor with bias: ({}, {}, {}), variance: ({}, {}, {})", 
                (*offset)[0], (*offset)[1], (*offset)[2], 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getValue() const
{
    return applyCalibration(std::make_shared<Vector3d>(Vector3d(accel_x(), accel_y(), accel_z())));
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getBias()
{
    return offset;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getGravity()
{
    return gravity;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    return std::make_shared<Vector3d>(*sample - *offset);
}

void libstp::sensor::MagnetoSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    variance = std::make_shared<Vector3d>(calibrationMatrix->colwise().squaredNorm());
    SPDLOG_INFO("[IMU] Calibrated magneto sensor with variance: ({}, {}, {})", 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

void libstp::sensor::MagnetoSensor::setHardIronOffset(const std::shared_ptr<Vector3d>& offset)
{
    this->offset = offset;
}

void libstp::sensor::MagnetoSensor::setSoftIronMatrix(const std::shared_ptr<Matrix3d>& matrix)
{
    softIronMatrix = matrix;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::getValue() const
{
    const auto raw = std::make_shared<Vector3d>(Vector3d(magneto_x(), magneto_y(), magneto_z()));
    return applyCalibration(raw);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    const auto corrected = std::make_shared<Vector3d>(*sample - *offset);
    return std::make_shared<Vector3d>(*softIronMatrix * *corrected);
}

libstp::async::AsyncAlgorithm<int> libstp::sensor::IMU::calibrate(const int sampleCount)
{
    auto samples_gyro = std::make_shared<MatrixX3d>(sampleCount, 3);
    auto samples_accel = std::make_shared<MatrixX3d>(sampleCount, 3);
    auto samples_mag = std::make_shared<MatrixX3d>(sampleCount, 3);

    SPDLOG_INFO("[IMU] Calibrating IMU... Please keep the device still.");
    for (int i = 0; i < sampleCount; ++i)
    {
        samples_gyro->row(i) = *gyro.getValue();
        samples_accel->row(i) = *accel.getValue();
        samples_mag->row(i) = *magneto.getValue();
        co_yield 1;
    }

    gyro.calibrate(samples_gyro);
    accel.calibrate(samples_accel);
    magneto.calibrate(samples_mag);

    SPDLOG_INFO("[IMU] Calibration complete.");
}

std::tuple<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Vector3d>> libstp::sensor::IMU::getReading() const
{
    return std::make_tuple(
        gyro.getValue(),
        accel.getValue(),
        magneto.getValue()
    );
}

