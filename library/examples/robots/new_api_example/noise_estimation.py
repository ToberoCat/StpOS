import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import ace_tools_open as tools
from libstp.sensor import *


# Initialize sensors
gyro_x = GyroXSensor()
gyro_y = GyroYSensor()
gyro_z = GyroZSensor()
accel_x = AccelXSensor()
accel_y = AccelYSensor()
accel_z = AccelZSensor()
mag_x = MagnetoXSensor()
mag_y = MagnetoYSensor()
mag_z = MagnetoZSensor()

# Sampling setup
sampling_rate = 1000  # Hz
duration = 5  # seconds
num_samples = sampling_rate * duration

# Initialize storage for sensor data
gyro_x_samples, gyro_y_samples, gyro_z_samples = [], [], []
accel_x_samples, accel_y_samples, accel_z_samples = [], [], []
mag_x_samples, mag_y_samples, mag_z_samples = [], [], []

# Collect sensor data
for _ in range(num_samples):
    gyro_x_samples.append(gyro_x.get_value())
    gyro_y_samples.append(gyro_y.get_value())
    gyro_z_samples.append(gyro_z.get_value())

    accel_x_samples.append(accel_x.get_value())
    accel_y_samples.append(accel_y.get_value())
    accel_z_samples.append(accel_z.get_value())

    mag_x_samples.append(mag_x.get_value())
    mag_y_samples.append(mag_y.get_value())
    mag_z_samples.append(mag_z.get_value())

# Convert to NumPy arrays
gyro_x_samples = np.array(gyro_x_samples)
gyro_y_samples = np.array(gyro_y_samples)
gyro_z_samples = np.array(gyro_z_samples)

accel_x_samples = np.array(accel_x_samples)
accel_y_samples = np.array(accel_y_samples)
accel_z_samples = np.array(accel_z_samples)

mag_x_samples = np.array(mag_x_samples)
mag_y_samples = np.array(mag_y_samples)
mag_z_samples = np.array(mag_z_samples)

# Compute bias (mean offset)
gyro_bias = [np.mean(gyro_x_samples), np.mean(gyro_y_samples), np.mean(gyro_z_samples)]
accel_bias = [np.mean(accel_x_samples), np.mean(accel_y_samples), np.mean(accel_z_samples)]
mag_bias = [np.mean(mag_x_samples), np.mean(mag_y_samples), np.mean(mag_z_samples)]  # ✅ Fixed missing mag_bias

# Compute noise variance
gyro_variance = [np.var(gyro_x_samples), np.var(gyro_y_samples), np.var(gyro_z_samples)]
accel_variance = [np.var(accel_x_samples), np.var(accel_y_samples), np.var(accel_z_samples)]
mag_variance = [np.var(mag_x_samples), np.var(mag_y_samples), np.var(mag_z_samples)]

# Convert gyro to rad/s and accel to m/s²
gyro_variance_rad = [var * (np.pi / 180) ** 2 for var in gyro_variance]
accel_variance_m = [var * (9.81) ** 2 for var in accel_variance]

# Magnetometer calibration
hard_iron_bias = np.array([-3.857525, -4.382995, 14.866970])  # Hard iron correction
soft_iron_matrix = np.array([  # Soft iron correction matrix
    [0.067137, -0.004223, -0.003411],
    [-0.004223, 0.062717, -0.005649],
    [-0.003411, -0.005649, 0.044935]
])

# Convert raw magnetometer data to 3×N array
raw_mag = np.vstack([mag_x_samples, mag_y_samples, mag_z_samples])  # Shape (3, N)

# ✅ Step 1: Apply Hard Iron Correction
mag_vector_hard_iron = raw_mag - hard_iron_bias.reshape(3,1)

# ✅ Step 2: Apply Soft Iron Correction (Matrix multiplication)
mag_vector_calibrated = soft_iron_matrix @ mag_vector_hard_iron

# ✅ Extract corrected values
mag_x_hard, mag_y_hard, mag_z_hard = mag_vector_hard_iron
mag_x_calib, mag_y_calib, mag_z_calib = mag_vector_calibrated

# ✅ Display statistics in table
df_stats = pd.DataFrame({
    "Sensor": [
        "Gyro X (rad/s)", "Gyro Y (rad/s)", "Gyro Z (rad/s)",
        "Accel X (m/s²)", "Accel Y (m/s²)", "Accel Z (m/s²)",
        "Magneto X (µT)", "Magneto Y (µT)", "Magneto Z (µT)"
    ],
    "Bias": np.round(gyro_bias + accel_bias + mag_bias, decimals=8),
    "Variance": np.round(gyro_variance_rad + accel_variance_m + mag_variance, decimals=8)
})
tools.display_dataframe_to_user(name="Sensor Bias & Noise Variance", dataframe=df_stats,)

# ✅ Plot Magnetometer Calibration
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.scatter(raw_mag[0, :], raw_mag[1, :], alpha=0.5, label="Raw Magnetometer Data")
plt.scatter(mag_x_hard, mag_y_hard, alpha=0.5, label="Hard Iron Corrected")
plt.xlabel("Mag X (µT)")
plt.ylabel("Mag Y (µT)")
plt.title("Magnetometer Hard Iron Correction")
plt.legend()
plt.grid()

plt.subplot(1, 2, 2)
plt.scatter(mag_x_hard, mag_y_hard, alpha=0.5, label="Hard Iron Corrected")
plt.scatter(mag_x_calib, mag_y_calib, alpha=0.5, label="Hard + Soft Iron Corrected")
plt.xlabel("Mag X (µT)")
plt.ylabel("Mag Y (µT)")
plt.title("Magnetometer Soft Iron Correction")
plt.legend()
plt.grid()

plt.show()

# ✅ Time-Domain Noise Data
plt.figure(figsize=(12, 6))
plt.plot(gyro_x_samples[:1000], label="Gyro X Noise (°/s)")
plt.plot(gyro_y_samples[:1000], label="Gyro Y Noise (°/s)")
plt.plot(gyro_z_samples[:1000], label="Gyro Z Noise (°/s)")
plt.xlabel("Samples")
plt.ylabel("°/s")
plt.title("Gyroscope Noise")
plt.legend()
plt.grid()
plt.show()

# ✅ Power Spectral Density (PSD) Analysis
freqs = np.fft.fftfreq(len(gyro_x_samples), d=1 / sampling_rate)
gyro_x_psd = np.abs(np.fft.fft(gyro_x_samples))**2
gyro_y_psd = np.abs(np.fft.fft(gyro_y_samples))**2
gyro_z_psd = np.abs(np.fft.fft(gyro_z_samples))**2

plt.figure(figsize=(8, 6))
plt.loglog(freqs[:num_samples // 2], gyro_x_psd[:num_samples // 2], label="Gyro X PSD")
plt.loglog(freqs[:num_samples // 2], gyro_y_psd[:num_samples // 2], label="Gyro Y PSD")
plt.loglog(freqs[:num_samples // 2], gyro_z_psd[:num_samples // 2], label="Gyro Z PSD")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Power Spectral Density")
plt.title("Gyroscope PSD")
plt.legend()
plt.grid()
plt.show()

# ✅ Allan Variance Approximation
tau_values = np.logspace(0.1, 3, num=50)
allan_var_gyro_x = [np.var(np.mean(gyro_x_samples[:int(tau_i)])) for tau_i in tau_values]

plt.figure(figsize=(8, 6))
plt.loglog(tau_values, allan_var_gyro_x, label="Gyro X Allan Variance")
plt.xlabel("Averaging Time (s)")
plt.ylabel("Allan Variance")
plt.title("Allan Variance Analysis - Gyroscope")
plt.legend()
plt.grid()
plt.show()