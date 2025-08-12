//
// Created by tobias on 2/9/25.
//
#include "libstp/ahrs/ekf.h"

#include <iostream>

#include "libstp/_config.h"
#include <sstream>

using namespace libstp::ahrs;

void eigenToString(const std::string& name, const Eigen::MatrixXd& mat)
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

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d S;
    S << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return S;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const double frequency)
    : frequency_(frequency), deltaTime(1.0 / frequency)
{
    varGyro = 0.0;
    varAccel = 0.0;
    varMagneto = 0.0;

    P = Eigen::Matrix4d::Identity();
    R = computeMeasurementNoiseCovariance();
}

void ExtendedKalmanFilter::computeInitialAttitude(
    const Eigen::MatrixXd& gyroSamples,
    const Eigen::MatrixXd& accelSamples,
    const Eigen::MatrixXd& magSamples)
{
    SPDLOG_INFO("Computing initial attitude using EKF.");
    if (gyroSamples.cols() != 3 || accelSamples.cols() != 3 || magSamples.cols() != 3)
    {
        SPDLOG_ERROR("Gyro samples: {}, Accel samples: {}, Mag samples: {}", gyroSamples.cols(), accelSamples.cols(),
                     magSamples.cols());
        SPDLOG_ERROR("All input matrices must have 3 columns.");
        return;
    }

    if (gyroSamples.rows() != accelSamples.rows() || gyroSamples.rows() != magSamples.rows())
    {
        throw std::invalid_argument("All input matrices must have the same number of rows.");
    }
    eigenToString("gyroSamples", gyroSamples);
    eigenToString("accelSamples", accelSamples);
    eigenToString("magSamples", magSamples);

    const auto numSamples = gyroSamples.rows();
    Eigen::MatrixXd Q(numSamples, 4);
    Eigen::Quaterniond result = ecompass(accelSamples.row(0), magSamples.row(0));
    Q.row(0) << result.w(), result.x(), result.y(), result.z();
    Q.row(0).normalize();
    SPDLOG_DEBUG("Q[{}]: ({}, {}, {}, {})", 0, Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3));

    for (int i = 1; i < numSamples; i++)
    {
        const Eigen::Vector3d gyr = gyroSamples.row(i);
        const Eigen::Vector3d acc = accelSamples.row(i);
        const Eigen::Vector3d mag = magSamples.row(i);

        Eigen::Quaterniond prevQ(Q(i - 1, 0), Q(i - 1, 1), Q(i - 1, 2), Q(i - 1, 3));
        SPDLOG_DEBUG("prevQ: ({}, {}, {}, {})", prevQ.w(), prevQ.x(), prevQ.y(), prevQ.z());
        Eigen::Quaterniond updatedQ = update(prevQ, gyr, acc, mag, deltaTime);
        Q.row(i) << updatedQ.w(), updatedQ.x(), updatedQ.y(), updatedQ.z();
        SPDLOG_DEBUG("Q[{}]: ({}, {}, {}, {})", i, Q(i, 0), Q(i, 1), Q(i, 2), Q(i, 3));
    }
}

Eigen::Matrix<double, 6, 6> ExtendedKalmanFilter::computeMeasurementNoiseCovariance() const
{
    Eigen::VectorXd covariance = Eigen::VectorXd::Constant(6, varAccel);
    covariance.tail(3).setConstant(varMagneto);
    return covariance.asDiagonal();
}

void ExtendedKalmanFilter::setMeasurementNoiseCovariance(double gyroVariance, double accelVariance, double magVariance)
{
    varGyro = gyroVariance;
    varAccel = accelVariance;
    varMagneto = magVariance;
    R = computeMeasurementNoiseCovariance();
}

Eigen::Quaterniond ExtendedKalmanFilter::update(const Eigen::Quaterniond& q,
                                                const Eigen::Vector3d& rawGyro,
                                                const Eigen::Vector3d& rawAccel,
                                                const Eigen::Vector3d& rawMagneto,
                                                double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("Time step must be positive.");
    }
    // Check norm of input quaternion
    double norm_q = q.norm();
    if (std::abs(norm_q - 1.0) > 1e-9)
    {
        throw std::runtime_error("A-priori quaternion must be unit length.");
    }

    // Prediction step
    Eigen::Quaterniond q_pred = f(q, rawGyro, dt);
    SPDLOG_DEBUG("q_pred: ({}, {}, {}, {})", q_pred.w(), q_pred.x(), q_pred.y(), q_pred.z());

    Eigen::Matrix4d F = dfdq(rawGyro, dt);

    const Eigen::Matrix4d Q_t = varGyro * (F * F.transpose());

    Eigen::Matrix4d P_pred = F * P * F.transpose() + Q_t;

    // Measurement step
    Eigen::VectorXd z_pred = h(q_pred, Eigen::VectorXd::Zero(6));

    Eigen::Matrix<double, 6, 4> H = dhdq(q_pred, Eigen::VectorXd::Zero(6));

    Eigen::Matrix<double, 6, 6> S = H * P_pred * H.transpose() + R;

    Eigen::Matrix<double, 4, 6> K = P_pred * H.transpose() * S.inverse();

    Eigen::Vector4d dq_vec = K * (Eigen::VectorXd::Zero(6) - z_pred);

    Eigen::Quaterniond dq(1.0, 0.5 * dq_vec(0), 0.5 * dq_vec(1), 0.5 * dq_vec(2));
    dq.normalize();

    Eigen::Quaterniond q_upd = q_pred * dq;
    q_upd.normalize();
    P = (Eigen::Matrix4d::Identity() - K * H) * P_pred;
    return q_upd;
}

void ExtendedKalmanFilter::setMagReference(const Eigen::Vector3d& magneticReference)
{
    mRef = magneticReference.normalized();
    aRef = Eigen::Vector3d::UnitZ();
}

Eigen::Matrix4d ExtendedKalmanFilter::Omega(const Eigen::Vector3d& x)
{
    Eigen::Matrix4d Omega;
    Omega << 0.0, -x(0), -x(1), -x(2),
        x(0), 0.0, x(2), -x(1),
        x(1), -x(2), 0.0, x(0),
        x(2), x(1), -x(0), 0.0;
    return Omega;
}

Eigen::Quaterniond ExtendedKalmanFilter::f(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega, double dt)
{
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d omega_half_dt = 0.5 * dt * omega;

    // Convert omega into a quaternion (pure quaternion: [0, ωx, ωy, ωz])
    Eigen::Quaterniond omega_quat(0, omega_half_dt.x(), omega_half_dt.y(), omega_half_dt.z());

    // Compute the new quaternion using quaternion multiplication: q_new = q * exp(0.5 * dt * Omega)
    delta_q.w() = 1.0;
    delta_q.vec() = omega_half_dt;
    delta_q.normalize();

    return q * delta_q; // Correct quaternion multiplication
}

Eigen::Matrix4d ExtendedKalmanFilter::dfdq(const Eigen::Vector3d& omega, const double dt)
{
    return Eigen::Matrix4d::Identity() + Omega(0.5 * dt * omega);
}

// Ensure h() returns an Eigen::VectorXd instead of Eigen::Quaterniond
Eigen::VectorXd ExtendedKalmanFilter::h(const Eigen::Quaterniond q, const Eigen::VectorXd& z) const
{
    const Eigen::Vector3d accel_est = q._transformVector(aRef);
    const Eigen::Vector3d mag_est = q._transformVector(mRef);

    Eigen::VectorXd z_est(6);
    z_est.head<3>() = accel_est;
    z_est.tail<3>() = mag_est;
    return z_est;
}

Eigen::MatrixXd ExtendedKalmanFilter::dhdq(const Eigen::Quaterniond& q, const Eigen::VectorXd& z) const
{
    // ToDo: Check if refactored mode is better
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    Eigen::MatrixXd H(3, 4);
    H << aRef(0) * qw + aRef(1) * qz - aRef(2) * qy, aRef(0) * qx + aRef(1) * qy + aRef(2) * qz, -aRef(0) * qy + aRef(1)
        * qx - aRef(2) * qw, -aRef(0) * qz + aRef(1) * qw + aRef(2) * qx,
        -aRef(0) * qz + aRef(1) * qw + aRef(2) * qx, aRef(0) * qy - aRef(1) * qx + aRef(2) * qw, aRef(0) * qx + aRef(1)
        * qy + aRef(2) * qz, -aRef(0) * qw - aRef(1) * qz + aRef(2) * qy,
        aRef(0) * qy - aRef(1) * qx + aRef(2) * qw, aRef(0) * qz - aRef(1) * qw - aRef(2) * qx, aRef(0) * qw + aRef(1) *
        qz - aRef(2) * qy, aRef(0) * qx + aRef(1) * qy + aRef(2) * qz;

    if (z.size() == 6)
    {
        Eigen::MatrixXd H_2(3, 4);
        H_2 << mRef(0) * qw + mRef(1) * qz - mRef(2) * qy, mRef(0) * qx + mRef(1) * qy + mRef(2) * qz, -mRef(0) * qy +
            mRef(1) * qx - mRef(2) * qw, -mRef(0) * qz + mRef(1) * qw + mRef(2) * qx,
            -mRef(0) * qz + mRef(1) * qw + mRef(2) * qx, mRef(0) * qy - mRef(1) * qx + mRef(2) * qw, mRef(0) * qx +
            mRef(1) * qy + mRef(2) * qz, -mRef(0) * qw - mRef(1) * qz + mRef(2) * qy,
            mRef(0) * qy - mRef(1) * qx + mRef(2) * qw, mRef(0) * qz - mRef(1) * qw - mRef(2) * qx, mRef(0) * qw +
            mRef(1) * qz - mRef(2) * qy, mRef(0) * qx + mRef(1) * qy + mRef(2) * qz;

        Eigen::MatrixXd H_full(6, 4);
        H_full << H, H_2;
        return 2.0 * H_full;
    }

    return 2.0 * H;
}

Eigen::Quaterniond ExtendedKalmanFilter::ecompass(const Eigen::Vector3d& acc, const Eigen::Vector3d& mag)
{
    // Normalize the accelerometer. This is our "down" direction in the NED frame.
    const double a_norm = acc.norm();
    if (a_norm < 1e-12)
    {
        throw std::runtime_error("Accelerometer vector has near-zero magnitude, cannot normalize.");
    }
    Eigen::Vector3d Rz = acc / a_norm; // 'Down' in NED

    // Normalize the magnetometer reading.
    const double m_norm = mag.norm();
    if (m_norm < 1e-12)
    {
        throw std::runtime_error("Magnetometer vector has near-zero magnitude, cannot normalize.");
    }
    const Eigen::Vector3d m_normed = mag / m_norm;

    // In the NED frame:
    // Rz = Down
    // Ry = cross(Down, Magnetic_field)
    // Rx = cross(Ry, Rz)
    Eigen::Vector3d Ry = Rz.cross(m_normed);
    if (const double Ry_norm = Ry.norm(); Ry_norm < 1e-12)
    {
        throw std::runtime_error("Cross product for Y is near zero. Accelerometer or magnetometer may be parallel.");
    }
    Ry.normalize();

    Eigen::Vector3d Rx = Ry.cross(Rz);
    if (const double Rx_norm = Rx.norm(); Rx_norm < 1e-12)
    {
        throw std::runtime_error("Cross product for X is near zero. Geometry is degenerate.");
    }
    Rx.normalize();

    // Construct the rotation matrix. Following the Python code, the final 3x3 has:
    // row(0) = Rx, row(1) = Ry, row(2) = Rz
    // This is equivalent to np.c_[Rx, Ry, Rz].T
    Eigen::Matrix3d R;
    R << Rx.x(), Rx.y(), Rx.z(),
        Ry.x(), Ry.y(), Ry.z(),
        Rz.x(), Rz.y(), Rz.z();

    // Convert rotation matrix to quaternion.
    Eigen::Quaterniond q(R);

    // Return the orientation as a quaternion.
    return q;
}
