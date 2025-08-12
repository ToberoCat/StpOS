class SensorFusion:
    def __init__(self,
                 samples_gyro,
                 samples_accel,
                 samples_magneto,
                 gyro_variance,
                 accel_variance,
                 magneto_variance,
                    delta_t
                 ):
        self.samples_gyro = samples_gyro
        self.samples_accel = samples_accel
        self.samples_magneto = samples_magneto
        self.gyro_variance = gyro_variance
        self.accel_variance = accel_variance
        self.magneto_variance = magneto_variance
        self.delta_t = delta_t

    def update(self, gyro_data, acc_data, mag_data, dt):
        pass

    def get_euler(self):
        pass