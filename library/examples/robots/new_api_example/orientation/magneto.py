import numpy as np
from libstp.sensor import MagnetoXSensor, MagnetoYSensor, MagnetoZSensor


class Magneto:
    def __init__(self):
        self.mag_x, self.mag_y, self.mag_z = MagnetoXSensor(), MagnetoYSensor(), MagnetoZSensor()
        self.magneto_variance = np.array([0.0, 0.0, 0.0])

    def apply_calibration(self, raw):
        hardIronOffset = np.array([6.795551921588304, -2.8382780053268664, -16.81266611037978])
        softIronMatrix = np.array([[1.1566975080977819, 0.025051206354146319, -0.12861993897443216], [0.025051206354146319, 1.1019141970915729, 0.16988047417722063], [-0.12861993897443222, 0.1698804741772206, 0.88704151404327969]])

        m_corrected = raw - hardIronOffset
        m_calibrated = (softIronMatrix @ m_corrected.T).T
        return m_calibrated

    def calibrate(self, samples):
        self.magneto_variance = np.var(samples, axis=0)
        return samples

    def get_variance(self):
        return self.magneto_variance.mean()

    def get_reading(self):
        raw = np.array([
            self.mag_z.get_value(),
            self.mag_y.get_value(),
            self.mag_x.get_value(),
        ])
        return self.apply_calibration(raw)
