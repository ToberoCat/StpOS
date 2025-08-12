import csv
import os
import threading
import time
from typing import Callable, Optional

import matplotlib.pyplot as plt


class DataPlotter:
    def __init__(self, data_func: Callable[[], float], interval: float = 1.0, duration: Optional[float] = None,
                 save_csv: Optional[str] = None, load_csv: Optional[str] = None, show_plot: bool = True):
        """
        Initializes the DataPlotter.
        :param data_func: Function that gets called to collect data, should return a float.
        :param interval: Time interval (in seconds) between data collections.
        :param duration: Total duration for data collection (None for infinite loop until manually stopped).
        :param save_csv: Filename to save collected data (optional).
        :param load_csv: Filename to load data from instead of recording (optional).
        :param show_plot: Whether to display the plot after recording (default: True).
        """
        self.data_func = data_func
        self.interval = interval
        self.duration = duration
        self.save_csv = save_csv
        self.load_csv = load_csv
        self.show_plot = show_plot
        self.data = []
        self.running = False
        self.thread = None

    def _collect_data(self):
        start_time = time.time()
        while self.running:
            elapsed_time = time.time() - start_time
            if self.duration and elapsed_time > self.duration:
                break

            value = self.data_func()
            self.data.append((elapsed_time, value))
            time.sleep(self.interval)

        if self.save_csv:
            self._save_to_csv()

        if self.show_plot:
            self.plot()

    def start_recording(self):
        """Starts data collection in a separate thread."""
        if self.load_csv and os.path.exists(self.load_csv):
            self._load_from_csv()
            self.plot()
            return

        self.running = True
        self.thread = threading.Thread(target=self._collect_data)
        self.thread.daemon = True
        self.thread.start()

    def stop_recording(self):
        """Stops data collection."""
        self.running = False
        if self.thread:
            self.thread.join()

    def _save_to_csv(self):
        """Saves collected data to a CSV file."""
        with open(self.save_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Value"])
            writer.writerows(self.data)

    def _load_from_csv(self):
        """Loads data from a CSV file."""
        self.data = []
        with open(self.load_csv, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            self.data = [(float(row[0]), float(row[1])) for row in reader]

    def plot(self):
        """Plots the recorded data."""
        if not self.data:
            print("No data available to plot.")
            return

        times, values = zip(*self.data)
        plt.figure(figsize=(10, 5))
        plt.plot(times, values, marker='o', linestyle='-')
        plt.xlabel("Time (s)")
        plt.ylabel("Value")
        plt.title("Collected Data Over Time")
        plt.grid()
        plt.show()


# Example usage:
if __name__ == "__main__":
    import random


    def get_random_value():
        return random.uniform(0, 10)


    plotter = DataPlotter(data_func=get_random_value, interval=0.5, duration=5, save_csv="data.csv", show_plot=True)
    plotter.start_recording()
    time.sleep(6)  # Ensure recording completes
    plotter.stop_recording()
