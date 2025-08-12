import asyncio
import time

import numpy as np

from libstp_helpers.collision_detection import CollisionDetector


class WallAligner:
    """
    A class for detecting collisions using an IMU and aligning the robot with a wall upon impact.

    Features:
    - Collision detection using filtered accelerometer data.
    - Gravity compensation using a low-pass filter.
    - Automatic alignment after collision using gyroscope feedback.
    """

    def __init__(self, device, detector: CollisionDetector, 
                 tolerance=0.3, 
                 stable_required=0.3, 
                 sample_interval=0.01, 
                 initial_duration=1.0,
                 window_duration=0.2):
        """
        Initialize the WallAligner with collision detection and gyro alignment.

        Parameters:
            device: The robot device (e.g., OmniWheeledDevice).
            detector: The CollisionDetector object.
            tolerance (float): Angular velocity threshold for stable alignment.
            stable_required (float): Duration for which stability must be maintained before alignment completes.
            sample_interval (float): Delay between gyro samples.
            initial_duration (float): Time spent determining the rotation direction.
            window_duration (float): Time window for stability checking.
        """
        self.device = device
        self.tolerance = tolerance
        self.stable_required = stable_required
        self.sample_interval = sample_interval
        self.initial_duration = initial_duration
        self.window_duration = window_duration
        self.imu = detector.imu
        self.detector = detector

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
            _, gy, _ = self.imu.gyro.get_value()
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
                _, gy, _ = self.imu.gyro.get_value()
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
        dt = 1.0 / self.detector.sample_rate
        next_loop_time = time.time()

        self.device.__apply_kinematics_model__(forward_speed, strafe_speed, 0)
        while True:
            collision, correction_angle = self.detector.update()

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

    async def forward(self):
        await self._run(0.2, 0)

    async def strafe(self, direction):
        d_sign = 1 if direction > 0 else -1
        await self._run(0, d_sign * 0.2)