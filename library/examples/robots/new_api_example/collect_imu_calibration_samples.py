import csv
import time

from external_tracker_client import Configuration, ApiClient, DefaultApi
from orientation.imu import IMU


class PositionFetcher:
    def __init__(self):
        self.configuration = Configuration(
            host="http://192.168.31.172:8000",
        )
        self.api_client = ApiClient(configuration=self.configuration)
        self.client = DefaultApi(api_client=self.api_client)

    def fetch_angle(self):
        markers = self.client.get_markers_markers_get()
        if not markers.robot_found:
            print("Robot not found.")
            return 401
        return markers.robot_angle


fetcher = PositionFetcher()
imu = IMU()
samples = []

print("Press Ctrl+C to stop.")
start = time.time()
while time.time() - start < 60:
    angle = fetcher.fetch_angle()
    if angle is None:
        continue
    gyro_data, acc_data, mag_data = imu.get_reading()
    sample = list(gyro_data) + list(acc_data) + list(mag_data) + [angle]
    samples.append(sample)
    time.sleep(0.001)

with open("samples.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz", "theta"])
    writer.writerows(samples)
    print("Samples saved to samples.csv - Samples count:", len(samples))
