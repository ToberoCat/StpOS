import asyncio
import time
from collections import deque

import numpy as np
from libstp.datatypes import Axis, Speed, for_cw_rotation, while_false, Direction
from libstp.device.omni_wheeled import OmniWheeledDevice
from libstp.motor import Motor
from libstp.sensor import is_button_clicked
from scipy.signal import butter, filtfilt

from libstp_helpers.orientation.imu import IMU


class CollisionDetector3D:
    """
    Iterative collision detector for a mobile robot with 3-axis accelerometer data.
    It removes gravity using a low-pass filter and detects collisions using threshold-based horizontal acceleration.

    Additionally, it computes the correction angle needed to align the robot with the wall.
    """

    def __init__(self,
                 imu: IMU,
                 sample_rate=100.0,
                 cutoff_freq=5.0,
                 threshold=3.0,
                 min_consecutive_samples=3,
                 buffer_size=100,
                 gravity_vector=(0, 9.81, 0)):
        self.imu = imu
        self.sample_rate = sample_rate
        self.cutoff_freq = cutoff_freq
        self.threshold = threshold
        self.min_consecutive_samples = min_consecutive_samples
        self.gravity_vector = np.array(gravity_vector)

        nyq = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyq
        self.b, self.a = butter(4, normal_cutoff, btype='low', analog=False)  # Noise reduction

        self.buffer_x = deque(maxlen=buffer_size)
        self.buffer_y = deque(maxlen=buffer_size)
        self.buffer_z = deque(maxlen=buffer_size)

        self.consecutive_count = 0

    def update(self):
        ax, ay, az = self.imu.accel.get_reading()
        self.buffer_x.append(ax)
        self.buffer_y.append(ay)
        self.buffer_z.append(az)

        # Only filter if we have enough samples
        if len(self.buffer_x) <= 15:
            return False, None
        raw_x = np.array(self.buffer_x)
        raw_y = np.array(self.buffer_y)
        raw_z = np.array(self.buffer_z)

        dominant_axis = np.argmax(np.abs(self.gravity_vector))  # Find index of largest gravity component

        # 2. Gravity Compensation (Subtract estimated gravity)
        comp_x = raw_x - self.gravity_vector[0]
        comp_y = raw_y - self.gravity_vector[1]
        comp_z = raw_z - self.gravity_vector[2]

        # 3. Apply Noise Reduction Filter to Compensated Signal
        fx = filtfilt(self.b, self.a, comp_x)
        fy = filtfilt(self.b, self.a, comp_y)
        fz = filtfilt(self.b, self.a, comp_z)

        # Dynamically determine which two axes to use for horizontal magnitude
        if dominant_axis == 0:  # Gravity is along X, use Y and Z
            horizontal_mag = np.sqrt(fy[-1] ** 2 + fz[-1] ** 2)
            correction_angle = np.arctan2(fy[-1], fz[-1])
        elif dominant_axis == 1:  # Gravity is along Y, use X and Z
            horizontal_mag = np.sqrt(fx[-1] ** 2 + fz[-1] ** 2)
            correction_angle = np.arctan2(fx[-1], fz[-1])
        else:  # Gravity is along Z, use X and Y
            horizontal_mag = np.sqrt(fx[-1] ** 2 + fy[-1] ** 2)
            correction_angle = np.arctan2(fx[-1], fy[-1])

        # Collision detection logic
        if horizontal_mag > self.threshold:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 0

        # If we have enough consecutive samples above threshold, declare collision
        if self.consecutive_count >= self.min_consecutive_samples:
            self.consecutive_count = 0
            return True, correction_angle

        return False, None  # No collision detected

    def plot_data(self):
        import matplotlib.pyplot as plt
        import numpy as np

        # Extracting the data from the collision detector class
        timestamps = np.array(self.timestamps)
        raw_x = np.array(self.buffer_x)
        raw_y = np.array(self.buffer_y)
        raw_z = np.array(self.buffer_z)
        filt_x = np.array(self.filtered_x)
        filt_y = np.array(self.filtered_y)
        filt_z = np.array(self.filtered_z)
        grav_x = np.array(self.gravity_x)
        grav_y = np.array(self.gravity_y)
        grav_z = np.array(self.gravity_z)
        timestamps = timestamps[:len(filt_x)]
        raw_x = raw_x[:len(filt_x)]
        raw_y = raw_y[:len(filt_x)]
        raw_z = raw_z[:len(filt_x)]

        fig, axs = plt.subplots(6, 1, figsize=(12, 24), sharex=True)

        # 1. Raw acceleration data
        axs[0].plot(timestamps, raw_x, label="Raw X", alpha=0.6)
        axs[0].plot(timestamps, raw_y, label="Raw Y", alpha=0.6)
        axs[0].plot(timestamps, raw_z, label="Raw Z", alpha=0.6)
        axs[0].set_ylabel("Raw Acceleration (m/s²)")
        axs[0].set_title("Raw Acceleration Data")
        axs[0].legend()

        # 2. Gravity estimation
        axs[1].plot(timestamps, grav_x, label="Estimated Gravity X", linestyle="dashed")
        axs[1].plot(timestamps, grav_y, label="Estimated Gravity Y", linestyle="dashed")
        axs[1].plot(timestamps, grav_z, label="Estimated Gravity Z", linestyle="dashed")
        axs[1].set_ylabel("Estimated Gravity (m/s²)")
        axs[1].set_title("Gravity Estimation")
        axs[1].legend()

        # 3. Filtered acceleration (gravity compensated)
        axs[2].plot(timestamps, filt_x, label="Filtered X", linewidth=2)
        axs[2].plot(timestamps, filt_y, label="Filtered Y", linewidth=2)
        axs[2].plot(timestamps, filt_z, label="Filtered Z", linewidth=2)
        axs[2].set_ylabel("Filtered Acceleration (m/s²)")
        axs[2].set_title("Filtered Acceleration (Gravity Compensated)")
        axs[2].legend()

        # 4. Horizontal magnitude and threshold
        axs[3].plot(timestamps, self.horizontal_mag, label="Horizontal Magnitude", linewidth=2, color="purple")
        axs[3].axhline(y=self.threshold, color='r', linestyle='--', label="Threshold")
        axs[3].set_ylabel("Acceleration Magnitude (m/s²)")
        axs[3].set_title("Horizontal Acceleration Magnitude & Collision Threshold")
        axs[3].legend()

        # 5. Collision detection over time
        axs[4].plot(timestamps, self.collisions_detection_candidates, label="Collision Detection Candidates",
                    color="red", drawstyle="steps-post")
        axs[4].plot(timestamps, self.collisions_detected, label="Collision Detected", color="magenta",
                    drawstyle="steps-post")
        axs[4].set_ylabel("Collision (1=True, 0=False)")
        axs[4].set_title("Collision Detection Events Over Time")
        axs[4].legend()

        # 6. Correction angle evolution
        axs[5].plot(timestamps, self.correction_angle, label="Correction Angle (rad)", color="blue")
        axs[5].set_ylabel("Correction Angle (radians)")
        axs[5].set_xlabel("Time (s)")
        axs[5].set_title("Correction Angle Evolution Over Time")
        axs[5].legend()

        plt.tight_layout()
        plt.show()


async def gyro_align(device, imu, rotation_sign, tolerance=0.3, stable_required=0.3):
    """
    Rotate the robot in small increments until the gyroscope indicates that angular motion
    has stabilized (i.e. the robot is no longer turning).

    Parameters:
        device: The robot device (with a rotate method).
        imu: The IMU instance providing gyro data.
        rotation_sign: +1 for a clockwise rotation, -1 for counterclockwise.
        small_angle: The rotation increment in degrees.
        tolerance: Angular velocity (rad/s) below which rotation is considered stable.
        stable_required: The cumulative time (in seconds) for which the averaged angular velocity
                         must remain below the tolerance before ending the alignment.
    """
    stable_time = 0.0
    # We'll use a short delay between samples to build a moving average.
    sample_interval = 0.01  # seconds

    # Continue issuing small rotation commands until stability is reached.
    while True:
        # Rotate by a small increment in the desired direction.
        device.__apply_kinematics_model__(-0.2, 0.0, rotation_sign)
        await asyncio.sleep(0.1)

        # Collect several gyro samples over 0.2 s.
        samples = []
        sample_start = time.time()
        while time.time() - sample_start < 0.2:
            gx, gy, gz = imu.gyro.get_reading()
            samples.append(gy)  # Hardcoded make it adjustable
            await asyncio.sleep(sample_interval)
        avg_angular_velocity = np.mean(np.abs(samples))

        # If the average angular velocity is below the tolerance, count that time as stable.
        if avg_angular_velocity < tolerance:
            stable_time += 0.2
        else:
            stable_time = 0.0

        # Log for debugging.
        print(
            f"Avg gyro ω: {avg_angular_velocity:.3f} deg/s, stable time: {stable_time:.2f} s, Rotation sign: {rotation_sign}")

        # If stability has been maintained for long enough, break out.
        if stable_time >= stable_required:
            break


def determine_rotation_sign(device, imu):
    start_time = time.time()
    heading = 0.0
    device.__apply_kinematics_model__(-0.2, 0.0, 0.0)
    while time.time() - start_time < 1.0:
        gx, gy, gz = imu.gyro.get_reading() # gy is the axis needed in this case, make it choosable
        heading += -gy * 0.01
        time.sleep(0.01)
    print("Initial heading:", heading)
    return 1 if heading > 0 else -1


async def main():
    import logging

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    effective_sample_rate = 100.0  # Hz
    imu = IMU()

    detector = CollisionDetector3D(imu)

    t = 0.0
    dt = 1.0 / effective_sample_rate
    imu.calibrate(100)

    start_time = time.time()
    next_loop_time = start_time
    executions = 0

    front_right_motor = Motor(0)
    front_left_motor = Motor(1)
    rear_right_motor = Motor(2)
    rear_left_motor = Motor(3)
    with OmniWheeledDevice(
            Axis.Y,
            Direction.Normal,
            front_left_motor,
            front_right_motor,
            rear_left_motor,
            rear_right_motor
    ) as device:  # type: OmniWheeledDevice
        device.wheel_distance_from_center = 0.12
        device.wheel_radius = 0.075 / 2
        device.ticks_per_revolution = 1571
        device.set_w_pid(0.5, 0.0, 0.0)
        device.set_vx_pid(1.0, 0.0, 0.0)
        device.set_vy_pid(1.0, 0.0, 0.0)
        device.set_heading_pid(5.0, 0.0, 0.0)
        # Begin driving (for example, drive straight with a small negative x velocity).
        device.__apply_kinematics_model__(1.0, 0.0, 0.0)

        # Run the sensor loop for a maximum of 5 seconds or until a collision is detected.
        while (time.time() - start_time) < 5:
            current_time = time.time()
            collision, correction_angle = detector.update()

            if collision:
                logging.info(
                    f"Collision detected at t={executions * dt:.2f}s with correction angle {correction_angle:.2f} rad")
                break

            executions += 1
            next_loop_time += dt
            sleep_time = next_loop_time - time.time()
            if sleep_time > 0:
                time.sleep(0.01)
            if time.time() > next_loop_time + dt:
                logging.warning("Loop falling behind, resynchronizing timing!")
                next_loop_time = time.time()

        total_time = time.time() - start_time
        actual_frequency = executions / total_time if total_time > 0 else 0
        logging.info(f"Actual execution frequency: {actual_frequency:.2f} Hz")
        #
        # # Decide rotation direction based on the collision correction angle.
        # # If correction_angle is positive, we choose clockwise (+1); otherwise, counterclockwise (-1).
        # rotation_sign = determine_rotation_sign(device, imu)
        #
        # logging.info("Initiating gyro-based alignment...")
        # await gyro_align(device, imu, rotation_sign, stable_required=0.3)
        # logging.info("Gyro-based alignment complete.")

    #detector.plot_data()


if __name__ == "__main__":
    asyncio.run(main())
