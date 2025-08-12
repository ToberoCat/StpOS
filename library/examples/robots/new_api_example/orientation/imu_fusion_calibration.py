import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

from orientation.fusion.ekf import EkfFusion


def unwrap_angles(angles):
    """
    Unwrap angles given in degrees.
    Converts to radians, unwraps, then converts back to degrees.
    """
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))


def load_sensor_data(file_path="sensor_data.csv"):
    """
    Loads sensor data from a CSV file.
    Expected columns:
      0-2: Gyroscope, 3-6: Accelerometer, 6-8: Magnetometer, 9: Ground-truth yaw angle.
    """
    data = np.genfromtxt(file_path, delimiter=",", skip_header=1)
    sample_time = 60  # total recording time in seconds
    calibration_samples = 50  # first 50 samples used for calibration
    dt = sample_time / len(data)

    gyroscope = data[:, 0:3]
    accelerometer = data[:, 3:6]
    magnetometer = data[:, 6:9]
    angle = data[:, 9]

    # Unwrap the true angle to remove clipping between -180 and 180.
    angle = unwrap_angles(angle)

    print("Data shapes:", gyroscope.shape, accelerometer.shape, magnetometer.shape, angle.shape)
    return gyroscope, accelerometer, magnetometer, angle, dt, calibration_samples


def calibrate_sensors(gyroscope, accelerometer, magnetometer, calibration_samples):
    """
    Calibrates the gyroscope and accelerometer using the first calibration_samples.
    Also computes baseline noise estimates from the calibration data.
    """
    # For a stationary sensor the gyro should be zero
    gyro_bias = np.mean(gyroscope[:calibration_samples], axis=0)
    # Assume expected acceleration is [0, 0, 9.81]. Here the calibration subtracts [0, 0, -9.81]
    acc_bias = np.mean(accelerometer[:calibration_samples], axis=0) - np.array([0, 0, -9.81])

    gyro_cal = gyroscope - gyro_bias
    acc_cal = accelerometer - acc_bias
    # Magnetometer is assumed to be already calibrated
    mag_cal = magnetometer.copy()

    print("Bias estimates (gyro, acc):", gyro_bias, acc_bias)

    baseline_gyro = np.mean(np.var(gyro_cal[:calibration_samples], axis=0))
    baseline_acc = np.mean(np.var(acc_cal[:calibration_samples], axis=0))
    baseline_mag = np.mean(np.var(mag_cal[:calibration_samples], axis=0))
    print("Baseline noise estimates (gyro, acc, mag):", baseline_gyro, baseline_acc, baseline_mag)

    return gyro_cal, acc_cal, mag_cal, gyro_bias, acc_bias, baseline_gyro, baseline_acc, baseline_mag


def simulate_ekf(fusion, gyro_data, acc_data, mag_data, dt, correction):
    """
    Runs the EKF filter sequentially over the dataset.
    Applies a constant correction to the yaw estimate.
    Returns an array of unwrapped yaw estimates (in degrees).
    """
    yaw_estimates = []
    for i in range(len(gyro_data)):
        fusion.update(gyro_data[i], acc_data[i], mag_data[i], dt)
        _, _, yaw = fusion.get_euler()
        # Apply correction; note that (-yaw) + correction is used to adjust the sign/offset.
        yaw_estimates.append((-yaw) + correction)

    # Unwrap the EKF yaw estimates (convert to radians, unwrap, then back to degrees)
    yaw_estimates = np.rad2deg(np.unwrap(np.deg2rad(yaw_estimates)))
    return np.array(yaw_estimates)


def objective(params, gyro_data, acc_data, mag_data, ground_truth, dt, calib_samples,
              baseline_gyro, baseline_acc, baseline_mag):
    """
    Objective function for optimization.
    The parameter vector params = [scale_gyro, scale_acc, scale_mag] scales the noise variances.
    The function returns the mean squared error over the validation set.
    Note: because both ground truth and predictions are unwrapped, the simple difference
    represents the true angular distance.
    """
    scale_gyro, scale_acc, scale_mag = params
    fusion = EkfFusion(gyro_data, acc_data, mag_data,
                       gyro_variance=baseline_gyro * scale_gyro,
                       accel_variance=baseline_acc * scale_acc,
                       magneto_variance=baseline_mag * scale_mag,
                       delta_t=dt)
    # Determine correction based on the calibration period
    initial_yaw = -fusion.get_euler()[2]
    true_yaw = np.mean(ground_truth[:calib_samples])
    correction = true_yaw - initial_yaw

    yaw_est = simulate_ekf(fusion, gyro_data, acc_data, mag_data, dt, correction)
    pred = yaw_est[calib_samples:]
    truth = ground_truth[calib_samples:]
    mse = np.mean((pred - truth) ** 2)
    return mse


def main():
    # === Load and prepare data ===
    gyroscope, accelerometer, magnetometer, angle, dt, calibration_samples = load_sensor_data("sensor_data.csv")

    # === Sensor Calibration ===
    gyro_cal, acc_cal, mag_cal, gyro_bias, acc_bias, baseline_gyro, baseline_acc, baseline_mag = calibrate_sensors(
        gyroscope, accelerometer, magnetometer, calibration_samples)

    # === Simulation with Default Noise Variances ===
    default_fusion = EkfFusion(gyro_cal, acc_cal, mag_cal,
                               gyro_variance=baseline_gyro,
                               accel_variance=baseline_acc,
                               magneto_variance=baseline_mag,
                               delta_t=dt)
    initial_yaw = -default_fusion.get_euler()[2]
    true_yaw = np.mean(angle[:calibration_samples])
    correction = true_yaw - initial_yaw
    print("Initial yaw (default):", initial_yaw)
    print("True yaw:", true_yaw)
    print("Correction:", correction)

    default_yaw = simulate_ekf(default_fusion, gyro_cal, acc_cal, mag_cal, dt, correction)
    initial_mse = np.mean((default_yaw[:calibration_samples] - angle[:calibration_samples]) ** 2)
    print("Initial MSE over calibration set (default):", initial_mse)

    # === Optimization of Noise Scaling Parameters ===
    p0 = [1.0, 1.0, 1.0]  # initial guess: unit scaling factors
    res = minimize(objective, p0, args=(gyro_cal, acc_cal, mag_cal, angle, dt, calibration_samples,
                                        baseline_gyro, baseline_acc, baseline_mag),
                   method='Nelder-Mead')
    print("Optimization result:", res)

    best_scale_gyro, best_scale_acc, best_scale_mag = res.x
    print("Best parameters:")
    print("  Gyro scale:", best_scale_gyro)
    print("  Acc scale:", best_scale_acc)
    print("  Magneto scale:", best_scale_mag)

    # === Simulation with Optimized Parameters ===
    final_fusion = EkfFusion(gyro_cal, acc_cal, mag_cal,
                             gyro_variance=baseline_gyro * best_scale_gyro,
                             accel_variance=baseline_acc * best_scale_acc,
                             magneto_variance=baseline_mag * best_scale_mag,
                             delta_t=dt)

    initial_yaw = -final_fusion.get_euler()[2]
    true_yaw = np.mean(angle[:calibration_samples])
    final_correction = true_yaw - initial_yaw

    final_yaw = simulate_ekf(final_fusion, gyro_cal, acc_cal, mag_cal, dt, final_correction)
    final_mse = np.mean((final_yaw[calibration_samples:] - angle[calibration_samples:]) ** 2)
    print("Final MSE over validation set:", final_mse)
    default_mse = np.mean((default_yaw[calibration_samples:] - angle[calibration_samples:]) ** 2)
    print("Default MSE over validation set:", default_mse)

    # === Visualization ===
    sample_time = 60  # seconds (total recording time)
    time_axis = np.linspace(0, sample_time, len(angle))

    plt.figure(figsize=(12, 6))
    plt.plot(time_axis, angle, label='True Angle (Unwrapped)', linewidth=2)
    plt.plot(time_axis, default_yaw, label='EKF Default Noise', linestyle='--')
    plt.plot(time_axis, final_yaw, label='EKF Optimized Noise', linestyle='-.')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Angle (degrees)')
    plt.title('True Angle vs. EKF Estimated Yaw Angle')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
