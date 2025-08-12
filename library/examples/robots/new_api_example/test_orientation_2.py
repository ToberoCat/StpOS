import time
from math import degrees

import numpy as np
from ahrs.common.orientation import q2euler
from ahrs.filters import EKF
from libstp.motor import Motor
from libstp.sensor import (
    GyroXSensor, GyroYSensor, GyroZSensor,
    AccelXSensor, AccelYSensor, AccelZSensor,
    MagnetoXSensor, MagnetoYSensor, MagnetoZSensor
)

DEG2RAD = np.pi / 180.0  # Degrees to radians
UT2NT = 1000.0  # Microtesla to nanotesla

CALIBRATION_SAMPLES = 100
READ_FREQUENCY = 100.0  # Hz

HARD_IRON_BIAS = np.array([35.286545, 1.681633, -88.325461])
SOFT_IRON_MATRIX = np.array(
    [[0.086836, 0.001869, 0.007202], [0.001869, 0.067466, -0.002765], [0.007202, -0.002765, 0.056744]])

# Noise variances used by the EKF (gyr^2, acc^2, mag^2)
NOISES_VAR = [
    0.0006,  # Gyroscope noise variance
    0.0131,  # Accelerometer noise variance
    0.33 ** 2  # Magnetometer noise variance
]

gyro_x_sensor = GyroXSensor()
gyro_y_sensor = GyroYSensor()
gyro_z_sensor = GyroZSensor()

accel_x_sensor = AccelXSensor()
accel_y_sensor = AccelYSensor()
accel_z_sensor = AccelZSensor()

mag_x_sensor = MagnetoXSensor()
mag_y_sensor = MagnetoYSensor()
mag_z_sensor = MagnetoZSensor()

motor1 = Motor(0)
motor2 = Motor(1)


def calibrate_sensors(num_samples: int = CALIBRATION_SAMPLES):
    print("Calibrating sensors... Keep the device still!")
    time.sleep(2)

    ax_vals, ay_vals, az_vals = [], [], []
    gx_vals, gy_vals, gz_vals = [], [], []
    mx_vals, my_vals, mz_vals = [], [], []

    for _ in range(num_samples):
        ax_vals.append(accel_x_sensor.get_value())
        ay_vals.append(accel_y_sensor.get_value())
        az_vals.append(accel_z_sensor.get_value())

        gx_vals.append(gyro_x_sensor.get_value())
        gy_vals.append(gyro_y_sensor.get_value())
        gz_vals.append(gyro_z_sensor.get_value())

        mx_vals.append(mag_x_sensor.get_value())
        my_vals.append(mag_y_sensor.get_value())
        mz_vals.append(mag_z_sensor.get_value())

        time.sleep(1 / READ_FREQUENCY)

    # Estimate biases for accelerometer (including gravity on Z) and gyroscope
    acc_bias = np.array([
        np.median(ax_vals),
        np.median(ay_vals),
        np.median(az_vals) - 9.81
    ])
    gyro_bias = np.array([
        np.median(gx_vals),
        np.median(gy_vals),
        np.median(gz_vals)
    ])

    gyro_samples, accel_samples, magneto_samples = np.array([gx_vals, gy_vals, gz_vals]).T, np.array(
        [ax_vals, ay_vals, az_vals]).T, np.array([mx_vals, my_vals, mz_vals]).T

    # Estimate the variance of the gyroscope in rad/s (gyro measures in deg/s)
    gyro_variance: float = (np.var(ax_vals) + np.var(ay_vals) + np.var(az_vals)) / 3
    accel_variance: float = (np.var(gx_vals) + np.var(gy_vals) + np.var(gz_vals)) / 3

    mag_shifted = magneto_samples - HARD_IRON_BIAS
    mag_cal = np.dot(mag_shifted, SOFT_IRON_MATRIX.T) * UT2NT

    magneto_variance: float = np.var(mag_cal) / 3
    print(f"Estimated gyro variance: {gyro_variance:.4f} rad/s")
    print(f"Estimated accel variance: {accel_variance:.4f} m/s²")
    print(f"Estimated magneto variance: {magneto_variance:.4f} nT²")
    NOISES_VAR[0] = gyro_variance
    NOISES_VAR[1] = accel_variance
    NOISES_VAR[2] = magneto_variance

    print("Calibration complete.")
    print(f"Accelerometer bias: {acc_bias}")
    print(f"Gyroscope bias: {gyro_bias}")
    print("Sample data with bias:")
    print(f"Accel: {accel_samples[0] - acc_bias}")
    print(f"Gyro: {gyro_samples[0] - gyro_bias}")
    # Return biases and raw sensor arrays
    return (acc_bias, gyro_bias), (gyro_samples, accel_samples, magneto_samples)


def initialize_ekf(gyro_samples, accel_samples, magneto_samples) -> EKF:
    """
    Initialize the EKF filter with the sample arrays. The magnetometer samples
    are calibrated using both hard-iron and soft-iron compensation, and then
    scaled to nanotesla.
    """
    # Gyro in rad/s
    gyro_cal = gyro_samples * DEG2RAD

    # Accel with bias removed
    accel_cal = accel_samples

    # Magnetometer with bias removed, soft-iron applied, then scaled
    #    1) (samples - HARD_IRON_BIAS)
    #    2) Apply soft-iron matrix
    #    3) Scale by UT2NT
    mag_shifted = magneto_samples - HARD_IRON_BIAS
    mag_cal = np.dot(mag_shifted, SOFT_IRON_MATRIX.T) * UT2NT

    ekf_instance = EKF(
        frequency=READ_FREQUENCY,
        frame='NED',
        noises=NOISES_VAR,
        gyr=gyro_cal,
        acc=accel_cal,
        mag=mag_cal
    )
    return ekf_instance


def get_sensor_data(acc_bias: np.ndarray, gyro_bias: np.ndarray) -> tuple:
    """
    Read current sensor data and apply:
      - Accelerometer bias removal
      - Gyro bias removal + degrees-to-radians conversion
      - Magnetometer: hard-iron bias removal, soft-iron correction, optional scaling
    """
    # Raw accelerometer
    acc_raw = np.array([
        accel_x_sensor.get_value(),
        accel_y_sensor.get_value(),
        accel_z_sensor.get_value()
    ])
    acc = acc_raw - acc_bias

    # Raw gyroscope
    gyro_raw = np.array([
        gyro_x_sensor.get_value(),
        gyro_y_sensor.get_value(),
        gyro_z_sensor.get_value()
    ])
    gyro = (gyro_raw - gyro_bias) * DEG2RAD

    # Raw magnetometer
    mag_raw = np.array([
        mag_x_sensor.get_value(),
        mag_y_sensor.get_value(),
        mag_z_sensor.get_value()
    ])

    # Apply hard-iron bias -> soft-iron matrix -> scale
    mag_shifted = (mag_raw - HARD_IRON_BIAS)
    mag_cal = SOFT_IRON_MATRIX.dot(mag_shifted) * UT2NT

    return acc, gyro, mag_cal


def main():
    # 1. Calibrate sensors
    biases, samples = calibrate_sensors()
    acc_bias, gyro_bias = biases

    gyro_samples, accel_samples, mag_samples = samples

    # Remove biases from calibration arrays, convert gyro to rad/s
    gyro_samples = (gyro_samples - gyro_bias)
    accel_samples = (accel_samples - acc_bias)

    # 2. Initialize EKF
    ekf = initialize_ekf(gyro_samples, accel_samples, mag_samples)

    print("\nStarting real-time orientation estimation...")
    time.sleep(1)

    initial_yaw = None
    initial_time = time.time()
    last_time = time.time()

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # 3. Get real-time sensor readings with biases removed + magnetometer soft-iron calibration
            acc, gyro, mag = get_sensor_data(acc_bias, gyro_bias)

            # 4. Update the EKF
            Q_new = ekf.update(q=ekf.q, gyr=gyro, acc=acc, mag=mag, dt=dt)
            ekf.q = Q_new.copy()

            roll, pitch, yaw = q2euler(Q_new)
            roll_deg = degrees(roll)
            pitch_deg = degrees(pitch)
            yaw_deg = degrees(yaw)

            # Print every 0.25 seconds (approx.)
            if current_time % 0.25 < dt:
                print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}, dt: {dt:.4f}s")

            # Let the system stabilize for first 5 seconds
            if current_time < initial_time + 5.0:
                continue

            if initial_yaw is None:
                initial_yaw = yaw_deg

            target_heading_deg = 0
            current_heading_deg = yaw_deg - initial_yaw
            sign = 1 if (time.time() - initial_time) < 20 else  -1
            heading_error = sign * int((target_heading_deg - current_heading_deg) * 100)

            # Example motor control (commented out):
            #motor1.set_velocity(sign * (1000 + heading_error))
            #motor2.set_velocity(sign * (1000 - heading_error))
            #print(f"Headings: Target={target_heading_deg}, Current={current_heading_deg}, Error={heading_error}")
            time.sleep(max(0.0, 1.0 / READ_FREQUENCY - (time.time() - current_time)))

    except KeyboardInterrupt:
        print("\nExiting loop due to KeyboardInterrupt.")

    finally:
        motor1.set_velocity(0)
        motor2.set_velocity(0)
        print("Motors stopped. Cleanup complete.")


if __name__ == "__main__":
    main()
