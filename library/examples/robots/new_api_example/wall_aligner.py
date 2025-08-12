import asyncio
import time
from collections import deque

import numpy as np
from scipy.signal import butter, filtfilt


class WallAligner:
    """
    A class for detecting collisions using an IMU and aligning the robot with a wall upon impact.

    Features:
    - Collision detection using filtered accelerometer data.
    - Gravity compensation using a low-pass filter.
    - Automatic alignment after collision using gyroscope feedback.
    """

    def __init__(self, device, imu, sample_rate=100.0, cutoff_freq=5.0, collision_threshold=3.0,
                 min_consecutive_samples=3, buffer_size=100,
                 tolerance=0.3, stable_required=0.3, sample_interval=0.01, initial_duration=1.0,
                 window_duration=0.2):
        """
        Initialize the WallAligner with collision detection and gyro alignment.

        Parameters:
            device: The robot device (e.g., OmniWheeledDevice).
            imu: The IMU instance providing accelerometer and gyro data.
            sample_rate (float): Sensor sampling rate in Hz.
            cutoff_freq (float): Cutoff frequency for accelerometer noise reduction.
            collision_threshold (float): Acceleration magnitude threshold for collision detection.
            min_consecutive_samples (int): Number of consecutive samples above the threshold to confirm a collision.
            buffer_size (int): Size of sensor data buffer.
            tolerance (float): Angular velocity threshold for stable alignment.
            stable_required (float): Duration for which stability must be maintained before alignment completes.
            sample_interval (float): Delay between gyro samples.
            initial_duration (float): Time spent determining the rotation direction.
            window_duration (float): Time window for stability checking.
        """
        self.device = device
        self.imu = imu
        self.sample_rate = sample_rate
        self.cutoff_freq = cutoff_freq
        self.collision_threshold = collision_threshold
        self.min_consecutive_samples = min_consecutive_samples
        self.buffer_size = buffer_size

        # Motion and alignment parameters
        self.tolerance = tolerance
        self.stable_required = stable_required
        self.sample_interval = sample_interval
        self.initial_duration = initial_duration
        self.window_duration = window_duration

        # Butterworth filter setup
        nyq = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyq
        self.b, self.a = butter(4, normal_cutoff, btype='low', analog=False)  # Noise reduction
        self.bg, self.ag = butter(2, 1.0 / nyq, btype='low', analog=False)  # Gravity estimation

        # Sensor data buffers
        self.buffer_x = deque(maxlen=buffer_size)
        self.buffer_y = deque(maxlen=buffer_size)
        self.buffer_z = deque(maxlen=buffer_size)

        self.filtered_x = deque(maxlen=buffer_size)
        self.filtered_y = deque(maxlen=buffer_size)

        # Collision state tracking
        self.consecutive_count = 0

    def detect_collision(self):
        """
        Checks for a collision using accelerometer data with gravity compensation.

        Returns:
            collision (bool): True if a collision is detected.
            correction_angle (float): Angle (radians) for alignment correction.
        """
        # Read raw accelerometer data
        ax, ay, az = self.imu.accel.get_reading()

        # Append raw values to buffers
        self.buffer_x.append(ax)
        self.buffer_y.append(ay)
        self.buffer_z.append(az)

        # Only process if we have enough samples
        if len(self.buffer_x) <= 15:
            return False, None

        raw_x = np.array(self.buffer_x)
        raw_y = np.array(self.buffer_y)
        raw_z = np.array(self.buffer_z)

        # Assume gravity along Y-axis
        gx, gy, gz = 0, 9.81, 0  # Static assumption

        # Gravity Compensation
        comp_x = raw_x - gx
        comp_y = raw_y - gy

        # Noise reduction filtering
        fx = filtfilt(self.b, self.a, comp_x)
        fy = filtfilt(self.b, self.a, comp_y)

        # Compute horizontal acceleration magnitude
        horizontal_mag = np.sqrt(fx[-1] ** 2 + fy[-1] ** 2)
        correction_angle = np.arctan2(fx[-1], fy[-1])

        # Collision detection logic
        if horizontal_mag > self.collision_threshold:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 0

        # Confirm collision if threshold is exceeded for consecutive samples
        if self.consecutive_count >= self.min_consecutive_samples:
            self.consecutive_count = 0
            return True, correction_angle

        return False, None

    async def align(self, forward_speed, strafe_speed):
        """
        Align the robot with the wall after a collision.

        Returns:
            int: Rotation direction (+1 for clockwise, -1 for counterclockwise).
        """
        # --- Phase 1: Determine Rotation Direction ---
        integrated_heading = 0.0
        start_time = time.time()
        self.device.__apply_kinematics_model__(forward_speed, strafe_speed, 0.0)
        while time.time() - start_time < self.initial_duration:
            _, gy, _ = self.imu.gyro.get_reading()
            integrated_heading += -gy * self.sample_interval
            await asyncio.sleep(self.sample_interval)
        rotation_sign = 1 if integrated_heading > 0 else -1

        # --- Phase 2: Gyro-based Alignment ---
        stable_time = 0.0
        self.device.__apply_kinematics_model__(forward_speed, strafe_speed, rotation_sign)
        while True:
            sample_window = []
            window_start = time.time()
            while time.time() - window_start < self.window_duration:
                _, gy, _ = self.imu.gyro.get_reading()
                sample_window.append(abs(gy))
                await asyncio.sleep(self.sample_interval)
            avg_angular_velocity = np.mean(sample_window)

            print(f"Avg gyro ω: {avg_angular_velocity:.3f}, stable time: {stable_time:.2f}s, Rotation: {rotation_sign}")

            if avg_angular_velocity < self.tolerance:
                stable_time += self.window_duration
            else:
                stable_time = 0.0

            if stable_time >= self.stable_required:
                break

        self.device.__apply_kinematics_model__(forward_speed, strafe_speed, 0)
        await asyncio.sleep(0.5)
        self.device.__apply_kinematics_model__(0, 0, 0)
        return rotation_sign

    async def _run(self, forward_speed, strafe_speed):
        """
        Continuously monitors for collisions and aligns the robot upon detection.
        Ensures that the sample rate remains consistent.
        """
        dt = 1.0 / self.sample_rate
        next_loop_time = time.time()

        self.device.__apply_kinematics_model__(forward_speed, strafe_speed, 0)
        while True:
            collision, correction_angle = self.detect_collision()

            if collision:
                print(f"Collision detected! Correcting with angle: {correction_angle:.2f} rad")
                break

            next_loop_time += dt
            sleep_time = next_loop_time - time.time()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                print("Warning: Loop falling behind, resynchronizing...")
                next_loop_time = time.time()

        await self.align(forward_speed, strafe_speed)

    async def backward(self):
        await self._run(-0.2, 0)

    async def strafe(self, direction):
        d_sign = 1 if direction > 0 else -1
        await self._run(0, d_sign * 0.2)

