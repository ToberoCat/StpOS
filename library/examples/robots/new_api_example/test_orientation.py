import asyncio
import numpy as np
from math import atan2, degrees, cos, sin, sqrt

from libstp.sensor import GyroZSensor, AccelXSensor, AccelYSensor, AccelZSensor, MagnetoXSensor, MagnetoYSensor, \
    MagnetoZSensor, is_button_clicked, GyroXSensor, GyroYSensor

# Define your calibration values (replace with actual values)
hard_iron_bias = np.array([-3.857525, -4.382995, 14.866970])  # Hard iron correction (bias values)
soft_iron_matrix = np.array([  # Soft iron correction matrix
    [0.067137, -0.004223, -0.003411],
    [-0.004223, 0.062717, -0.005649],
    [-0.003411, -0.005649, 0.044935]
])

async def main():
    gyro_x = GyroXSensor()
    gyro_y = GyroYSensor()
    gyro_z = GyroZSensor()
    accel_x = AccelXSensor()
    accel_y = AccelYSensor()
    accel_z = AccelZSensor()
    mag_x = MagnetoXSensor()
    mag_y = MagnetoYSensor()
    mag_z = MagnetoZSensor()

    # Bias correction, keeping gravity on Z-axis
    samples = 10
    ax_values, ay_values, az_values = [], [], []

    print("Calibrating sensors... Keep device still!")
    await asyncio.sleep(2)

    for _ in range(samples):
        ax_values.append(accel_x.get_value())
        ay_values.append(accel_y.get_value())
        az_values.append(accel_z.get_value())

        await asyncio.sleep(0.01)

    ax_bias = sum(ax_values) / samples
    ay_bias = sum(ay_values) / samples
    az_bias = (sum(az_values) / samples) + 9.81  # Keep gravity

    print(f"Bias: ax={ax_bias:.3f}, ay={ay_bias:.3f}, az={az_bias:.3f}")

    alpha = 0.9  # Low-pass filter
    ax_filtered = ay_filtered = az_filtered = 0
    mx_filtered = my_filtered = mz_filtered = 0

    while not is_button_clicked():
        ax = accel_x.get_value() - ax_bias
        ay = accel_y.get_value() - ay_bias
        az = accel_z.get_value() - az_bias

        # ✅ Step 1: Compute pitch & roll from accelerometer
        pitch = atan2(-ax, sqrt(ay**2 + az**2))  # Rotation around X-axis
        roll = atan2(ay, az)                     # Rotation around Y-axis

        # Read raw magnetometer values
        mx = mag_x.get_value()
        my = mag_y.get_value()
        mz = mag_z.get_value()

        # ✅ Step 2: Apply Hard Iron Correction (Subtract bias)
        mag_vector = np.array([mx, my, mz]) - hard_iron_bias

        # ✅ Step 3: Apply Soft Iron Correction (Matrix multiplication)
        corrected_mag_vector = np.dot(soft_iron_matrix, mag_vector)

        mx_corrected, my_corrected, mz_corrected = corrected_mag_vector

        # ✅ Step 4: Rotate Magnetometer Readings into the Tilt-Free Reference Frame
        sin_pitch = sin(pitch)
        cos_pitch = cos(pitch)
        sin_roll = sin(roll)
        cos_roll = cos(roll)

        # **Apply full rotation matrix**
        mx_tilted = mx_corrected * cos_pitch + mz_corrected * sin_pitch
        my_tilted = mx_corrected * sin_roll * sin_pitch + my_corrected * cos_roll - mz_corrected * sin_roll * cos_pitch

        # ✅ Step 5: Compute tilt-compensated yaw
        yaw = atan2(my_tilted, mx_tilted)

        # Apply low-pass filter for smoothing
        mx_filtered = alpha * mx_filtered + (1 - alpha) * mx_tilted
        my_filtered = alpha * my_filtered + (1 - alpha) * my_tilted
        mz_filtered = alpha * mz_filtered + (1 - alpha) * mz_corrected

        print(f"Tilt-Compensated Yaw: {degrees(yaw):.2f}°")

        await asyncio.sleep(0.1)

asyncio.run(main())
