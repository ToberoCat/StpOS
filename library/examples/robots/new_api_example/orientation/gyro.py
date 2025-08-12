import numpy as np
from libstp.sensor import GyroXSensor, GyroZSensor, GyroYSensor


class Gyro:
    def __init__(self):
        self.gyro_x, self.gyro_y, self.gyro_z = GyroXSensor(), GyroYSensor(), GyroZSensor()
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.gyro_variance = np.array([0.0, 0.0, 0.0])

    def calibrate(self, samples):
        self.gyro_offset = np.median(samples, axis=0)

        applied = [self.apply_calibration(sample) for sample in samples]
        self.gyro_variance = np.var(applied, axis=0)
        return applied

    def apply_calibration(self, sample):
        return sample - self.gyro_offset

    def get_variance(self):
        return self.gyro_variance.mean()

    def get_reading(self):
        # To radiant
        return self.apply_calibration(np.array([
            self.gyro_x.get_value(),
            self.gyro_y.get_value(),
            self.gyro_z.get_value()
        ]) * np.pi / 180)
