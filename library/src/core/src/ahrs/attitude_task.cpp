//
// Created by tobias on 2/9/25.
//
#include "libstp/ahrs/attitude_task.h"

#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

#include "kipr/accel/accel.h"
#include "kipr/gyro/gyro.h"
#include "kipr/magneto/magneto.h"
#include "libstp/_config.h"
#include "libstp/ahrs/ekf.h"
#include "libstp/math/math.h"
#include "libstp/sensor/imu.h"

using namespace Eigen;
using namespace std::chrono;

Matrix3f gyroscopeMisalignment = Matrix3f::Identity();
Matrix3f accelerometerMisalignment = Matrix3f::Identity();
Vector3f gyroscopeSensitivity{1.0f, 1.0f, 1.0f};
Vector3f accelerometerSensitivity{1.0f, 1.0f, 1.0f};
Vector3f hardIronOffset{-6.961966f, -8.508221f, 19.059037f};
Matrix3f softIronMatrix = (Matrix3f() <<
    0.080397f, -0.003086f, -0.008319f,
    -0.003086f, 0.068542f, -0.002196f,
    -0.008319f, -0.002196f, 0.048982f).finished();

constexpr int ESTIMATION_FREQUENCY = 100;
constexpr int CALIBRATION_SAMPLES = 5;
constexpr int UT2NT = 1000;
constexpr double GRAVITY = 9.81;


void eigenToString2(const std::string& name, const Eigen::MatrixXd& mat)
{
    spdlog::info("{} = [", name); // Print matrix name separately

    for (int i = 0; i < mat.rows(); ++i)
    {
        std::string rowStr = " ["; // Row start
        for (int j = 0; j < mat.cols(); ++j)
        {
            rowStr += std::to_string(mat(i, j));
            if (j < mat.cols() - 1)
                rowStr += ", "; // Comma separator
        }
        rowStr += "]"; // Row end

        spdlog::info(rowStr); // Log each row separately
    }

    spdlog::info("]"); // Close the main bracket
}

namespace libstp::ahrs
{
    Vector3d computeColumnwiseMedian(const MatrixXd& samples)
    {
        Vector3d medians;
        for (int col = 0; col < samples.cols(); ++col)
        {
            std::vector<double> values(samples.col(col).data(), samples.col(col).data() + samples.rows());
            std::sort(values.begin(), values.end());

            size_t n = values.size();
            if (n % 2 == 0)
            {
                medians(col) = (values[n / 2 - 1] + values[n / 2]) / 2.0;
            }
            else
            {
                medians(col) = values[n / 2];
            }
        }
        return medians;
    }

    double computeVariance(const MatrixXd& samples)
    {
        const long n = samples.rows();
        const RowVector3d mean = samples.colwise().mean();
        MatrixXd centered = samples.rowwise() - mean;
        const Vector3d variance = (centered.array().square().colwise().sum() / (n - 1)).matrix();
        return variance.mean();
    }

    void AttitudeEstimatorTask::calibrateBiases()
    {
        SPDLOG_INFO("Calibrating IMU. Don't touch the robot!");

        MatrixXd gyroSamples(CALIBRATION_SAMPLES, 3);
        MatrixXd accelSamples(CALIBRATION_SAMPLES, 3);
        MatrixXd magSamples(CALIBRATION_SAMPLES, 3);

        for (int i = 0; i < CALIBRATION_SAMPLES; ++i)
        {
            gyroSamples.row(i) = Vector3d(gyro_x(), gyro_y(), gyro_z()) * DEG_TO_RAD;
            accelSamples.row(i) = Vector3d(accel_x(), accel_y(), accel_z());
            magSamples.row(i) = Vector3d(magneto_x(), magneto_y(), magneto_z());
            std::this_thread::sleep_for(milliseconds(1000 / ESTIMATION_FREQUENCY));
        }
        //
        // Matrix<double, CALIBRATION_SAMPLES, 3> gyroSamples;
        // gyroSamples << -0.003193, -0.102166, 0.010642,
        //     -0.004257, -0.103230, 0.010642,
        //     -0.004257, -0.102166, 0.012771,
        //     -0.002128, -0.102166, 0.013835,
        //     -0.002128, -0.101101, 0.013835;
        //
        // Matrix<double, CALIBRATION_SAMPLES, 3> accelSamples;
        // accelSamples << -0.083797, 0.014365, 9.938332,
        //     -0.083797, 0.014365, 9.938332,
        //     -0.107739, 0.047884, 9.931149,
        //     -0.074220, 0.000000, 9.993398,
        //     -0.071826, 0.043096, 9.897630;
        //
        // Matrix<double, CALIBRATION_SAMPLES, 3> magSamples;
        // magSamples << 34.050003, 11.550000, -136.950012,
        //     33.675003, 12.075001, -136.875000,
        //     33.225002, 11.925000, -136.725006,
        //     34.050003, 12.150001, -136.950012,
        //     33.825001, 11.925000, -136.725006;

        gyroSamples *= DEG_TO_RAD;
        magSamples *= UT2NT;

        Vector3d gyroOffset = computeColumnwiseMedian(gyroSamples);
        Vector3d accelOffset = computeColumnwiseMedian(accelSamples);
        accelOffset.z() -= GRAVITY;

        gyroSamples = gyroSamples.rowwise() - gyroOffset.transpose();
        accelSamples = accelSamples.rowwise() - accelOffset.transpose();
        magSamples = applyMagnetoCalibrationMatrix(magSamples);

        const double gyroVariance = computeVariance(gyroSamples);
        const double accelVariance = computeVariance(accelSamples);
        const double magVariance = computeVariance(magSamples);
        std::cout << "Noise Covariance: Gyro: " << gyroVariance << ", Accel: " << accelVariance << ", Mag: " <<
            magVariance
            << std::endl;
        ekf.setMeasurementNoiseCovariance(gyroVariance, accelVariance, magVariance);
        ekf.computeInitialAttitude(gyroSamples, accelSamples, magSamples);
    }

    void AttitudeEstimatorTask::estimationLoop()
    {
        auto lastTime = high_resolution_clock::now();
        SPDLOG_DEBUG("Attitude estimator started");

        while (running)
        {
            auto currentTime = high_resolution_clock::now();
            duration<double> cycleTime = currentTime - lastTime;
            lastTime = currentTime;

            Vector3d gyro, accel, mag;
            getSensorReadings(gyro, accel, mag);

            ekf.Q = ekf.update(ekf.Q, gyro, accel, mag, cycleTime.count());

            auto [roll, pitch, yaw] = math::quaternionToEuler(ekf.Q);
            SPDLOG_TRACE("Roll: {}, Pitch: {}, Yaw: {}", roll, pitch, yaw);

            auto leftWait = milliseconds(1000 / ESTIMATION_FREQUENCY) - duration_cast<milliseconds>(cycleTime);
            if (leftWait.count() > 0)
            {
                std::this_thread::sleep_for(leftWait);
            }
            else
            {
                SPDLOG_WARN("Estimation loop took too long: {} ms", -leftWait.count());
            }
        }
        SPDLOG_DEBUG("Attitude estimator stopped");
    }

    void AttitudeEstimatorTask::startEstimation()
    {
        running = true;
        calibrateBiases();
    }

    void AttitudeEstimatorTask::stopEstimation()
    {
        running = false;
    }

    Vector3d AttitudeEstimatorTask::applyMagnetoCalibration(const Vector3d& mag) const
    {
        return softIronMatrix.cast<double>().inverse() * (mag - hardIronOffset.cast<double>());
    }

    MatrixXd AttitudeEstimatorTask::applyMagnetoCalibrationMatrix(const MatrixXd& magMatrix) const
    {
        Eigen::Matrix3d softIronMatrix;
        softIronMatrix << 8.6835979917983174e-05, 1.8693188866621046e-06, 7.2021832934262233e-06,
            1.8693188866621044e-06, 6.7465869482354705e-05, -2.7647151757120113e-06,
            7.2021832934262275e-06, -2.7647151757120113e-06, 5.6744400048492919e-05;
        Eigen::Vector3d hardIronOffset;
        hardIronOffset << 35286.544748373242, 1681.6333912084119, -88325.461119161439;

        Eigen::Matrix3d invSoftIron = softIronMatrix.inverse();
        eigenToString2("magMatrix", magMatrix);
        Eigen::MatrixXd mag_shifted = magMatrix.rowwise() - hardIronOffset.transpose();
        eigenToString2("mag_shifted", mag_shifted);
        Eigen::MatrixXd mag_cal = mag_shifted * invSoftIron.transpose();
        eigenToString2("mag_cal", mag_cal);
        return mag_cal;
    }


    void AttitudeEstimatorTask::getSensorReadings(Vector3d& gyro, Vector3d& accel, Vector3d& mag) const
    {
        gyro = Vector3d(gyro_x(), gyro_y(), gyro_z()) * DEG_TO_RAD - gyroOffset;
        accel = Vector3d(accel_x(), accel_y(), accel_z()) - accelOffset;
        mag = applyMagnetoCalibration(Vector3d(magneto_x(), magneto_y(), magneto_z())) * UT2NT;
    }
}
