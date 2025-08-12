from math import degrees

import numpy as np
from ahrs.common.orientation import q2euler
from ahrs.filters import EKF
from ahrs.utils import WMM

from libstp_helpers.orientation.fusion import SensorFusion


class EkfFusion(SensorFusion):
    def __init__(self,
                 samples_gyro,
                 samples_accel,
                 samples_magneto,
                 gyro_variance,
                 accel_variance,
                 magneto_variance,
                 delta_t):
        super().__init__(samples_gyro,
                         samples_accel,
                         samples_magneto,
                         gyro_variance,
                         accel_variance,
                         magneto_variance,
                         delta_t)
        samples_gyro, samples_accel, samples_magneto = self.__transform_data(samples_gyro, samples_accel,
                                                                             samples_magneto)
        self.noise_variances = np.array([
            gyro_variance, # 1.5581432792616294,
            accel_variance, # * 0.00016425855613536413,
            magneto_variance # * 1.2622828299387887
        ])
        print(self.noise_variances)
        latitude, longitude, height = 48.207563, 15.616942, 269.0
        wmm = WMM(latitude=latitude, longitude=longitude, height=height)
        m_ref = np.array([wmm.X, wmm.Y, wmm.Z])
        self.ekf_instance = EKF(
            frequency=1 / delta_t,
            frame='NED',
            noises=self.noise_variances,
            gyr=samples_gyro,
            acc=samples_accel,
            mag=samples_magneto,
            magnetic_ref=m_ref
        )

    def __transform_data(self, gyro_data, acc_data, mag_data):
        return gyro_data, acc_data, mag_data

    def update(self, gyro_data, acc_data, mag_data, dt):
        gyro_data, acc_data, mag_data = self.__transform_data(gyro_data, acc_data, mag_data)
        q_new = self.ekf_instance.update(q=self.ekf_instance.q, gyr=gyro_data, acc=acc_data, mag=mag_data, dt=dt)
        self.ekf_instance.q = q_new.copy()


    def get_euler(self):
        roll, pitch, yaw = q2euler(self.ekf_instance.q)
        #roll, pitch, yaw = degrees(roll), degrees(pitch), degrees(yaw)
        return roll, pitch, yaw
