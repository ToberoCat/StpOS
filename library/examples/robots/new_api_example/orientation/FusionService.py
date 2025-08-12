import math
import threading
import time

from orientation.fusion.ekf import EkfFusion
# from orientation.fusion.madgwick import MadgwickFusion
from orientation.imu import IMU


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (w, x, y, z).
    This is a standard math conversion in the aerospace convention.
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


class ImuFusionService:
    def __init__(self, frequency=100, use_madgwick=False):
        """
        frequency: Data read frequency from IMU in Hz
        use_madgwick: Whether to use MadgwickFusion or EkfFusion
        """
        self.frequency = frequency
        self.delta_t = 1.0 / frequency
        self.stop_event = threading.Event()

        # IMU initialization and calibration
        self.imu = IMU()
        samples_gyro, samples_acc, samples_mag = self.imu.calibrate(100)
        gyro_var = self.imu.gyro.get_variance()
        accel_var = self.imu.accel.get_variance()
        mag_var = self.imu.magneto.get_variance()

        # Print the variances for debugging
        print(f"Gyro Variance: {gyro_var}")
        print(f"Accel Variance: {accel_var}")
        print(f"Mag Variance: {mag_var}")

        # Choose your fusion filter
        if use_madgwick:
            from orientation.fusion.madgwick import MadgwickFusion
            self.fusion = MadgwickFusion(
                samples_gyro=samples_gyro,
                samples_accel=samples_acc,
                samples_magneto=samples_mag,
                gyro_variance=gyro_var,
                accel_variance=accel_var,
                magneto_variance=mag_var,
                delta_t=self.delta_t
            )
        else:
            self.fusion = EkfFusion(
                samples_gyro=samples_gyro,
                samples_accel=samples_acc,
                samples_magneto=samples_mag,
                gyro_variance=gyro_var,
                accel_variance=accel_var,
                magneto_variance=mag_var,
                delta_t=self.delta_t
            )

        # For heading offset so that the initial heading is treated as 0 deg
        self.offset_angle = None

        # Thread reference
        self.thread = None

    def reset_state(self, angle=0.0):
        """
        Reset the heading to a desired angle (by default 0.0).
        This can also be used to reinitialize certain parts of the fusion if needed.

        angle: the angle (in radians) you want the system to have right now
        """
        # If you want to re-run calibrations or re-initialize your filter you can do so here.
        # Minimal example: Adjust the offset so current yaw becomes 'angle'.

        # Get current orientation from the fusion
        roll, pitch, current_yaw = self.fusion.get_euler()

        # If offset_angle hasn't been set yet, treat the first call as the initial heading
        if self.offset_angle is None:
            self.offset_angle = current_yaw
        # We want the new heading to be `angle`, so offset will shift it accordingly
        self.offset_angle = (current_yaw - self.offset_angle) - angle

        print(f"[reset_state] offset_angle set to: {self.offset_angle:.2f} rad "
              f"so that the new yaw is {angle} rad")

    def set_quaternion(self, quaternion):
        """
        Stub method to pass quaternion data to a closed loop differential drive
        or any other consumer. Override or replace this with your own implementation.

        quaternion: (w, x, y, z)
        """
        # In a real system, you might do something like:
        # self.drive_controller.update_orientation(quaternion)
        pass

    def start_daemon(self):
        """
        Start the background daemon thread.
        """
        if self.thread is not None and self.thread.is_alive():
            print("[start_daemon] Thread is already running.")
            return

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print("[start_daemon] Daemon thread started.")

    def stop(self):
        """
        Signal the daemon thread to stop and wait for it to exit.
        """
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        print("[stop] Daemon thread stopped.")

    def _run_loop(self):
        """
        Main loop that runs in a thread. Reads IMU data, updates the fusion filter,
        and calls set_quaternion at a fixed rate.
        """
        start_time = time.perf_counter()
        counter = 0.0
        # We'll do something like 0.5s update for printing or other debug usage
        print_interval = 0.5

        while not self.stop_event.is_set():
            loop_start = time.perf_counter()
            dt = loop_start - start_time
            start_time = loop_start
            counter += dt

            # Get IMU data
            gyro_data, acc_data, mag_data = self.imu.get_reading()

            # Update the fusion filter
            self.fusion.update(gyro_data, acc_data, mag_data, dt)

            # Get Euler angles
            roll, pitch, yaw = self.fusion.get_euler()

            # Initialize offset_angle if not set
            if self.offset_angle is None:
                self.offset_angle = yaw

            # Apply offset so that initial heading is zero
            yaw_relative = yaw - self.offset_angle

            # Convert to quaternion
            quaternion = euler_to_quaternion(roll, pitch, yaw_relative)

            # Pass quaternion to external consumer (like your diff drive controller)
            self.set_quaternion(quaternion)

            # Print debug info every 0.5 seconds
            if counter >= print_interval:
                print(f"[ImuFusionService] RPY raw=({roll:.2f}, {pitch:.2f}, {yaw:.2f}), "
                      f"offset={self.offset_angle:.2f}, yaw_relative={yaw_relative:.2f}")
                counter -= print_interval

            # Sleep to keep the loop at roughly 'frequency' Hz
            # sleep_time = max(0.0, self.delta_t - (time.perf_counter() - loop_start))
            # time.sleep(sleep_time)


if __name__ == "__main__":
    service = ImuFusionService(frequency=100, use_madgwick=False)
    service.start_daemon()

    time.sleep(5)

    service.reset_state(angle=0.0)

    time.sleep(5)

    service.stop()
