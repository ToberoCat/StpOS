import time

from libstp.sensor import is_button_clicked
import pandas as pd

from orientation.imu import IMU

imu = IMU()

# Header: time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z
data = []
start_time = time.perf_counter()
print("Press the button to stop data collection.")
while not is_button_clicked():
    gyro_data, acc_data, mag_data = imu.get_reading()
    data.append([time.perf_counter() - start_time, *gyro_data, *acc_data, *mag_data])
    if len(data) % 100 == 0:
        print(f"Collected {len(data)} samples.")

dataframe = pd.DataFrame(data, columns=["time", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z", "mag_x", "mag_y", "mag_z"])
dataframe.to_csv("data.csv", index=False)
