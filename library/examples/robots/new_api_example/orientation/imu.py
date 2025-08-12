import time

import numpy as np

from orientation.accel import Accel
from orientation.aligner import Alignment
from orientation.gyro import Gyro
from orientation.magneto import Magneto


class IMU:
    def __init__(self):
        self.gyro = Gyro()
        self.accel = Accel()
        self.magneto = Magneto()
        self.aligner = Alignment(self.accel, self.magneto)
        self.read_freq = 100

    def calibrate(self, sample_count):
        samples_gyro = []
        samples_accel = []
        samples_mag = []
        print("Calibrating IMU... Please keep the device still.")
        for _ in range(sample_count):
            samples_gyro.append(self.gyro.get_reading())
            samples_accel.append(self.accel.get_reading())
            samples_mag.append(self.magneto.get_reading())
            time.sleep(1 / self.read_freq)

        samples_gyro = np.array(self.gyro.calibrate(samples_gyro))
        samples_accel = np.array(self.accel.calibrate(samples_accel))
        samples_mag = np.array(self.magneto.calibrate(samples_mag))
        print("Calibration complete.")

        R = self.aligner.calibrate()

        for i in range(len(samples_gyro)):
            samples_gyro[i] = self.aligner.align(samples_gyro[i])
            samples_accel[i] = self.aligner.align(samples_accel[i])
            samples_mag[i] = self.aligner.align(samples_mag[i])

        return samples_gyro, samples_accel, samples_mag

    def get_reading(self):
        return (self.aligner.align(self.gyro.get_reading()),
                self.aligner.align(self.accel.get_reading()),
                self.aligner.align(self.magneto.get_reading()))
