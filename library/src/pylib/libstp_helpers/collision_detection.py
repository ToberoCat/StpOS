from collections import deque
from typing import Optional

import numpy as np
from libstp.datatypes import while_false
from scipy.signal import butter, filtfilt


class CollisionDetector:
    def __init__(self,
                 device,
                 sample_rate=100.0,
                 cutoff_freq=5.0,
                 threshold=2.5,
                 min_consecutive_samples=3,
                 buffer_size=100,
                 gravity_vector=(0, 9.81, 0)):
        self.imu = device.imu
        self.sample_rate = sample_rate
        self.cutoff_freq = cutoff_freq
        self.threshold = threshold
        self.min_consecutive_samples = min_consecutive_samples
        self.gravity_vector = np.array(gravity_vector)

        nyq = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyq
        self.b, self.a = butter(4, normal_cutoff, btype='low', analog=False)

        self.buffer_x = deque(maxlen=buffer_size)
        self.buffer_y = deque(maxlen=buffer_size)
        self.buffer_z = deque(maxlen=buffer_size)

        self.consecutive_count = 0

    def update(self):
        ax, ay, az = self.imu.accel.get_value()
        self.buffer_x.append(ax)
        self.buffer_y.append(ay)
        self.buffer_z.append(az)

        if len(self.buffer_x) <= 15:
            return False, None
        raw_x = np.array(self.buffer_x)
        raw_y = np.array(self.buffer_y)
        raw_z = np.array(self.buffer_z)

        dominant_axis = np.argmax(np.abs(self.gravity_vector))

        comp_x = raw_x - self.gravity_vector[0]
        comp_y = raw_y - self.gravity_vector[1]
        comp_z = raw_z - self.gravity_vector[2]

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

        if horizontal_mag > self.threshold:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 0

        if self.consecutive_count >= self.min_consecutive_samples:
            self.consecutive_count = 0
            return True, correction_angle

        return False, None


def until_collision(device = None, detector: Optional[CollisionDetector] = None):
    if device is None and detector is None:
        raise ValueError("Either Device or CollisionDetector must be provided.")
    if detector is None:
        detector = CollisionDetector(device)

    def check_condition():
        collision, _ = detector.update()
        return collision

    return while_false(check_condition)
