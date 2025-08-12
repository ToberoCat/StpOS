import numpy as np
from libstp.sensor import AccelXSensor, AccelYSensor, AccelZSensor


class Accel:
    def __init__(self):
        self.accel_x, self.accel_y, self.accel_z = AccelXSensor(), AccelYSensor(), AccelZSensor()
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.accel_variance = np.array([0.0, 0.0, 0.0])

    def calibrate(self, samples):
        self.accel_offset = np.median(samples, axis=0)

        gravity_axis = np.argmax(np.abs(self.accel_offset))
        gravity_sign = np.sign(self.accel_offset[gravity_axis])

        self.accel_offset[gravity_axis] -= 9.81 * gravity_sign

        axis_names = ['X', 'Y', 'Z']
        print(f"Detected gravity axis: {axis_names[gravity_axis]} ({'+' if gravity_sign > 0 else '-'})")

        applied = [self.apply_calibration(sample) for sample in samples]

        self.accel_variance = np.var(applied, axis=0)
        return applied

    def apply_calibration(self, sample):
        return sample - self.accel_offset

    def get_variance(self):
        return self.accel_variance.mean()

    def get_reading(self):
        return self.apply_calibration(np.array([
            self.accel_x.get_value(),
            self.accel_y.get_value(),
            self.accel_z.get_value()
        ]))
