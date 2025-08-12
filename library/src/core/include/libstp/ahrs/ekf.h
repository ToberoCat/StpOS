//
// Created by tobias on 2/9/25.
//
#pragma once

/**
 * @brief Extended Kalman Filter (EKF) for attitude estimation (quaternion-based).
 *
 * The EKF estimates a quaternion representing orientation with measurements
 * from gyroscopes, accelerometers, and magnetometers. This implementation
 * closely follows the Python version described in the provided documentation.
 *
 * The main steps are:
 *   1) Prediction:
 *      q_hat(t) = f(q(t-1), omega(t))    // discrete-time approximation
 *      P_hat(t) = F P(t-1) F^T + Q
 *
 *   2) Correction:
 *      v(t)  = z(t) - h(q_hat(t))
 *      S(t)  = H P_hat(t) H^T + R
 *      K(t)  = P_hat(t) H^T S(t)^(-1)
 *      q(t)  = q_hat(t) + K(t)*v(t)
 *      P(t)  = [I - K(t)*H] P_hat(t)
 *
 * @note All rotations are handled as quaternions. The user must ensure the
 *       input data are consistent with the chosen reference frame (NED or ENU).
 */

#include <Eigen/Dense>

namespace libstp::ahrs
{
    /**
     * @class ExtendedKalmanFilter
     * @brief An Extended Kalman Filter for orientation (quaternion) estimation.
     */
    class ExtendedKalmanFilter
    {
    public:
        Eigen::Quaterniond Q; ///< Last corrected quaternion state

        /**
         * @brief Constructor for the EKF
         * @param[in] frequency    Sampling frequency in Hz (default = 100.0)
         */
        explicit ExtendedKalmanFilter(double frequency = 100.0);

        /**
        * @brief Default destructor
        */
        ~ExtendedKalmanFilter() = default;

        /**
         * Computes the initial attitude (orientation) of a system using sensor data from 
         * a gyroscope, accelerometer, and magnetometer, all provided in the NED (North-East-Down) reference frame.
         *
         * @param gyroSamples A matrix of size (N × 3), where each row represents a gyroscope 
         *                    measurement in radians per second (rad/s). The columns correspond 
         *                    to angular velocity components along the (x, y, z) axes in the NED frame.
         * @param accelSamples A 3D vector representing the measured acceleration in meters per 
         *                     second squared (m/s²) along the (x, y, z) axes in the NED frame. 
         *                     This is typically used to determine the pitch and roll angles.
         * @param magSamples A 3D vector representing the measured magnetic field strength in 
         *                   nanoteslas (nT) along the (x, y, z) axes in the NED frame. 
         *                   This is primarily used to determine the heading (yaw angle).
         */
        void computeInitialAttitude(
            const Eigen::MatrixXd& gyroSamples,
            const Eigen::MatrixXd& accelSamples,
            const Eigen::MatrixXd& magSamples
        );

        [[nodiscard]] Eigen::Matrix<double, 6, 6> computeMeasurementNoiseCovariance() const;

        /*
         * Gyro Var in rad / s
         * Accel Var in m / s^2
         * Mag Var in nT
         */
        void setMeasurementNoiseCovariance(double gyroVariance, double accelVariance, double magVariance);


        /**
         * @brief Update method for the EKF using one sample of data.
         *
         * @param[in] q_prev  A-priori quaternion estimate (4D, normalized).
         * @param[in] gyr     Angular velocity in rad/s (3D).
         * @param[in] acc     Accelerometer measurement in m/s^2 (3D).
         * @param[in] mag     Magnetometer measurement (3D) - optional.
         * @param[in] dt      Time step in seconds. If negative, it uses internal dt_.
         *
         * @return The a-posteriori (corrected) quaternion estimate (4D, normalized).
         */
        Eigen::Quaterniond update(const Eigen::Quaterniond& q_prev,
                               const Eigen::Vector3d& gyr,
                               const Eigen::Vector3d& acc,
                               const Eigen::Vector3d& mag = Eigen::Vector3d::Zero(),
                               double dt = -1.0);

        /**
         * @brief Set the reference magnetometer vector by explicit 3D vector.
         *
         * If you have a known 3D reference, pass it here.
         * This snippet shows just the 3D vector setter.
         * The reference can be found at: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
         *
         * @param[in] magneticReference  3D reference of Earth magnetic field in chosen frame.
         */
        void setMagReference(const Eigen::Vector3d& magneticReference);

    private:
        /**
         * @brief Internal method: Omega operator for a 3D vector x.
         * @param[in] x A 3D vector.
         * @return A 4x4 "Omega matrix".
         */
        [[nodiscard]] static Eigen::Matrix4d Omega(const Eigen::Vector3d& x);

        /**
         * @brief The discrete-time process model for quaternion propagation.
         *        f(q, omega, dt) ~= [I + (dt/2)Omega(omega)] * q
         *
         * @param[in] q     The previous quaternion state (4D).
         * @param[in] omega Angular velocity vector (3D).
         * @param[in] dt    Time step in seconds.
         * @return          Predicted quaternion (not yet normalized).
         */
        [[nodiscard]] static Eigen::Quaterniond f(const Eigen::Quaterniond& q,
                                               const Eigen::Vector3d& omega,
                                               double dt);

        /**
         * @brief Jacobian of f w.r.t q, i.e. dfdq.
         * @param[in] omega Angular velocity vector (3D).
         * @param[in] dt    Time step in seconds.
         * @return          4x4 matrix F = (I + (dt/2)*Omega(omega)).
         */
        [[nodiscard]] static Eigen::Matrix4d dfdq(const Eigen::Vector3d& omega,
                                                  double dt);

        /**
         * @brief Measurement model: h(q) => either 3D or 6D, depending on whether using acc or acc+mag.
         * @param[in] q     Predicted quaternion state (4D).
         * @param[in] z     Current measurement vector: either 3D (acc) or 6D (acc+mag).
         * @return          Expected measurement from the predicted state.
         */
        [[nodiscard]] Eigen::VectorXd h(Eigen::Quaterniond q,
                                        const Eigen::VectorXd& z) const;

        /**
         * @brief Jacobian of h w.r.t q, i.e. dh/dq. 
         * @param[in] q Predicted quaternion state (4D).
         * @param[in] z Current measurement vector: either 3D or 6D.
         * @return      Jacobian matrix (3x4 or 6x4).
         */
        [[nodiscard]] Eigen::MatrixXd dhdq(const Eigen::Quaterniond& q,
                                           const Eigen::VectorXd& z) const;

        /**
         * @brief (Optional) Helper function to compute orientation from one sample of accelerometer + magnetometer,
         *        often called ecompass or triad method.
         *
         * @param[in] acc  3D accelerometer data.
         * @param[in] mag  3D magnetometer data.
         * @return         A quaternion representing the orientation.
         *
         * @note Not implemented here. Provide your own version if needed.
         */
        static Eigen::Quaterniond ecompass(const Eigen::Vector3d& acc,
                                           const Eigen::Vector3d& mag);

        double frequency_; ///< Sampling frequency in Hz
        double deltaTime; ///< Time step in seconds

        double varGyro; ///< Gyroscope noise variance
        double varAccel; ///< Accelerometer noise variance
        double varMagneto; ///< Magnetometer noise variance

        Eigen::Matrix4d P; ///< 4x4 State covariance
        Eigen::Matrix<double, 6, 6> R; ///< 6x6 Measurement noise covariance (for acc+mag)

        Eigen::Vector3d aRef; ///< Gravity reference in chosen frame
        Eigen::Vector3d mRef; ///< Magnetic reference in chosen frame
    };
}
