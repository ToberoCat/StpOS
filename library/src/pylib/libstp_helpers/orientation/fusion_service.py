import math

from libstp_helpers.orientation.fusion.ekf import EkfFusion
# from orientation.fusion.madgwick import MadgwickFusion
from libstp_helpers.orientation.imu import IMU


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (w, x, y, z).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions q1 * q2.
    q = (w, x, y, z)
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (w, x, y, z)


def quaternion_conjugate(q):
    """
    Conjugate of a quaternion (inverse if quaternion is normalized).
    """
    w, x, y, z = q
    return (w, -x, -y, -z)


import threading
import time
from collections import deque


class ImuFusionService:
    def __init__(self,
                 frequency=100,
                 use_madgwick=False,
                 device=None,
                 moving_average_window=5,  # Number of samples for smoothing
                 calibration_samples=500,
                 g_var_scale=0.5,
                 a_var_scale=1,
                 m_var_scale=1
                 ):
        """
        frequency: Data read frequency from IMU in Hz
        use_madgwick: Whether to use MadgwickFusion or EkfFusion
        device: Some device/robot class that expects set_quaternion(w, x, y, z)
        moving_average_window: Number of recent yaw samples to average for smoothing
        """
        self.frequency = frequency
        self.delta_t = 1.0 / frequency
        self.stop_event = threading.Event()
        self.device = device
        self.moving_average_window = moving_average_window

        # Moving average buffer for yaw smoothing
        self.yaw_history = deque(maxlen=self.moving_average_window)

        # IMU initialization and calibration
        self.imu = IMU()
        self.samples_gyro, self.samples_acc, self.samples_mag = self.imu.calibrate()
        self.gyro_var = self.imu.gyro.get_variance() * g_var_scale
        self.accel_var = self.imu.accel.get_variance() * a_var_scale
        self.mag_var = self.imu.magneto.get_variance() * m_var_scale

        print(f"Gyro Variance: {self.gyro_var}")
        print(f"Accel Variance: {self.accel_var}")
        print(f"Mag Variance: {self.mag_var}")

        if use_madgwick:
            from orientation.fusion.madgwick import MadgwickFusion
            self.fusion = MadgwickFusion(
                samples_gyro=self.samples_gyro,
                samples_accel=self.samples_acc,
                samples_magneto=self.samples_mag,
                gyro_variance=self.gyro_var,
                accel_variance=self.accel_var,
                magneto_variance=self.mag_var,
                delta_t=self.delta_t
            )
        else:
            self.fusion = EkfFusion(
                samples_gyro=self.samples_gyro,
                samples_accel=self.samples_acc,
                samples_magneto=self.samples_mag,
                gyro_variance=self.gyro_var,
                accel_variance=self.accel_var,
                magneto_variance=self.mag_var,
                delta_t=self.delta_t
            )

        self.reference_quat = None
        self.offset_angle = None
        self.thread = None

    def reset_state(self, angle=0.0):
        roll, pitch, current_yaw = self.fusion.get_euler()
        self.offset_angle = (current_yaw - (self.offset_angle or current_yaw)) - angle
        print(f"[reset_state] offset_angle set to: {self.offset_angle:.2f} rad")

    def set_quaternion(self, quaternion, raw_yaw):
        """
        Send the smoothed quaternion to your device.
        """
        # Update moving average buffer
        self.yaw_history.append(raw_yaw - (self.offset_angle or 0.0))

        # Compute moving average of yaw
        smoothed_yaw = sum(self.yaw_history) / len(self.yaw_history)

        if self.device:
            self.device.set_quaternion(0, 0, 0, smoothed_yaw)
        else:
            print(f"[set_quaternion] Smoothed Yaw={smoothed_yaw:.3f} rad")

    def start_daemon(self):
        if self.thread and self.thread.is_alive():
            print("[start_daemon] Thread is already running.")
            return

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print("[start_daemon] Daemon thread started.")

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()
        print("[stop] Daemon thread stopped.")

    def _run_loop(self):
        start_time, counter, print_interval = time.perf_counter(), 0.0, 0.5

        time.sleep(0.5)
        while not self.stop_event.is_set():
            loop_start = time.perf_counter()
            dt = loop_start - start_time
            start_time = loop_start
            counter += dt

            gyro_data, acc_data, mag_data = self.imu.get_reading()
            self.fusion.update(gyro_data, acc_data, mag_data, dt)

            roll, pitch, yaw = self.fusion.get_euler()
            raw_quat = euler_to_quaternion(roll, pitch, yaw)

            if self.reference_quat is None:
                self.reference_quat = raw_quat
            if self.offset_angle is None:
                self.offset_angle = yaw

            ref_conjugate = quaternion_conjugate(self.reference_quat)
            relative_quat = quaternion_multiply(ref_conjugate, raw_quat)

            self.set_quaternion(relative_quat, yaw)

            if counter >= print_interval:
                smoothed_yaw = sum(self.yaw_history) / len(self.yaw_history) if self.yaw_history else yaw
                print(f"[ImuFusionService] RPY raw=({roll:.2f}, {pitch:.2f}, {yaw:.2f}), "
                      f"smoothed_yaw={smoothed_yaw:.2f}")
                counter -= print_interval

            # time.sleep(max(0.0, self.delta_t - (time.perf_counter() - loop_start)))


if __name__ == "__main__":
    service = ImuFusionService(frequency=100, use_madgwick=False)
    service.start_daemon()

    time.sleep(5)
    # Now reset. This will capture the *current* orientation as reference.
    service.reset_state()
    time.sleep(5)

    service.stop()
