from math import degrees

import imufusion
import numpy as np

from orientation.fusion import SensorFusion


class MadgwickFusion(SensorFusion):
    def __init__(self,
                 samples_gyro,
                 samples_accel,
                 samples_magneto,
                 gyro_variance,
                 accel_variance,
                 magneto_variance,
                 delta_t,
                 auto_tune=False,
                 fixed_gain=None,
                 fixed_acc_rej=None,
                 fixed_mag_rej=None):
        """
        Initialize the imufusion Madgwick filter.

        Parameters:
          samples_gyro, samples_accel, samples_magneto : calibration data arrays
          gyro_variance, accel_variance, magneto_variance : sensor noise estimates
          delta_t : sample period (in seconds)
          auto_tune : if True, run autotuning on the calibration data (default: True)
          fixed_gain, fixed_acc_rej, fixed_mag_rej : if auto_tune is False, you may supply fixed parameters
        """
        super().__init__(samples_gyro,
                         samples_accel,
                         samples_magneto,
                         gyro_variance,
                         accel_variance,
                         magneto_variance,
                         delta_t)
        self.delta_t = delta_t
        self.sample_rate = int(1 / delta_t)

        # Transform calibration data (here we scale magnetometer readings)
        self.samples_gyro, self.samples_accel, self.samples_magneto = self.__transform_data(
            samples_gyro, samples_accel, samples_magneto)

        if auto_tune:
            self.gain, self.acc_rejection, self.mag_rejection = self._autotune_parameters(
                self.samples_gyro, self.samples_accel, self.samples_magneto, delta_t)
        else:
            self.gain = fixed_gain if fixed_gain is not None else 0.5
            self.acc_rejection = fixed_acc_rej if fixed_acc_rej is not None else 10.0
            self.mag_rejection = fixed_mag_rej if fixed_mag_rej is not None else 10.0

        print(f"Using settings: gain = {self.gain:.4f}, "
              f"acc_rejection = {self.acc_rejection:.4f}, "
              f"mag_rejection = {self.mag_rejection:.4f}")

        # Create the imufusion Madgwick filter instance.
        # Note: imufusion uses its Ahrs class with a Settings structure.
        self.offset = imufusion.Offset(self.sample_rate)
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NED,  # coordinate convention
            self.gain,  # filter gain
            2000,  # gyroscope range (fixed)
            self.acc_rejection,  # acceleration rejection threshold
            self.mag_rejection,  # magnetic rejection threshold
            5 * self.sample_rate  # recovery trigger period (5 seconds)
        )

    def __transform_data(self, gyro_data, acc_data, mag_data):
        return gyro_data, acc_data, mag_data

    def _simulate_filter(self, gain, acc_rej, mag_rej, gyro_data, acc_data, mag_data, dt):
        """
        Simulate the imufusion filter on the calibration data with the given parameters.
        Since the device is static during calibration, the estimated Euler angles should be constant.
        We measure the sum of variances of the Euler angles as the error metric.
        """
        temp_ahrs = imufusion.Ahrs()
        temp_ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NED,
            gain,
            2000,
            acc_rej,
            mag_rej,
            5 * (1 / dt)
        )
        eulers = []
        # Feed the calibration data sample-by-sample.
        for i in range(len(gyro_data)):
            temp_ahrs.update(gyro_data[i], acc_data[i], mag_data[i], dt)
            # Obtain Euler angles in radians
            euler = np.array(temp_ahrs.quaternion.to_euler())
            eulers.append(euler)
        eulers = np.array(eulers)
        # The error is the sum of variances of roll, pitch, and yaw.
        error = np.sum(np.var(eulers, axis=0))
        return error

    def _autotune_parameters(self, gyro_data, acc_data, mag_data, dt):
        """
        Autotune the filter parameters using Differential Evolution.
        The objective is to minimize the variance in the estimated Euler angles over the calibration period.
        """
        from scipy.optimize import differential_evolution

        def objective(params):
            gain_candidate, acc_rej_candidate, mag_rej_candidate = params
            if gain_candidate <= 0 or acc_rej_candidate <= 0 or mag_rej_candidate <= 0:
                return 1e6  # penalize non-physical values
            return self._simulate_filter(gain_candidate, acc_rej_candidate, mag_rej_candidate,
                                         gyro_data, acc_data, mag_data, dt)

        # Define search bounds for gain and rejection thresholds.
        bounds = [
            (0.1, 1.0),  # gain bounds
            (1.0, 20.0),  # acceleration rejection bounds
            (1.0, 20.0)  # magnetic rejection bounds
        ]
        result = differential_evolution(
            objective,
            bounds,
            strategy='best1bin',
            maxiter=50,
            popsize=15,
            tol=1e-6,
            mutation=(0.5, 1),
            recombination=0.7,
            polish=True
        )
        gain_opt, acc_rej_opt, mag_rej_opt = result.x
        print("Autotuning complete:", result.message)
        print(f"Optimal parameters: gain = {gain_opt:.4f}, "
              f"acc_rejection = {acc_rej_opt:.4f}, "
              f"mag_rejection = {mag_rej_opt:.4f}")
        return gain_opt, acc_rej_opt, mag_rej_opt

    def update(self, gyro_data, acc_data, mag_data, dt):
        """
        Update the imufusion filter with new sensor readings.
        Incoming data is transformed before updating.
        """
        gyro_data, acc_data, mag_data = self.__transform_data(gyro_data, acc_data, mag_data)
        gyro_data = self.offset.update(gyro_data)
        self.ahrs.update(gyro_data, acc_data, mag_data, dt)

    def get_euler(self):
        """
        Retrieve the current Euler angles (in degrees) from the filter.
        """
        euler = self.ahrs.quaternion.to_euler()
        return tuple(degrees(angle) for angle in euler)
