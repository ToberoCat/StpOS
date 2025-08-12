import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_data(csv_path):
    """
    Load the CSV data into a pandas DataFrame.
    """
    df = pd.read_csv(csv_path)
    df['yaw'] = df['yaw'].apply(np.degrees)
    ground_truth_baseline = df['ground_truth_theta'].iloc[0]
    sensor_fusion_baseline = df['yaw'].iloc[0]
    df['yaw'] -= (sensor_fusion_baseline - ground_truth_baseline)
    df['timestamp'] -= df['timestamp'].iloc[0]
    return df


def compute_yaw_error(df):
    """
    Compute the yaw error between the sensor fusion yaw and the ground truth.
    """
    df['yaw_error'] = df['yaw'] - df['ground_truth_theta']
    df['yaw_error'] = ((df['yaw_error'] + 180) % 360) - 180  # Wrap error into [-180, 180]
    df["roll_error"] = df["roll"]  # should be 0
    df["pitch_error"] = df["pitch"]  # should be 0
    return df


def moving_average(series, window_size=10):
    """
    Compute the moving average of a pandas Series.
    """
    return series.rolling(window=window_size, center=True, min_periods=1).mean()


def print_error_statistics(df):
    """
    Compute and print key error statistics.
    """
    mean_error = df['yaw_error'].mean()
    std_error = df['yaw_error'].std()
    rmse_error = np.sqrt(np.mean(df['yaw_error'] ** 2))
    mae_error = np.mean(np.abs(df['yaw_error']))

    print("Yaw Error Statistics:")
    print(f"  Mean Error: {mean_error:.2f} deg")
    print(f"  Standard Deviation: {std_error:.2f} deg")
    print(f"  RMSE: {rmse_error:.2f} deg")
    print(f"  MAE: {mae_error:.2f} deg")


def plot_time_series(df, window_size=10):
    """
    Plot the sensor fusion yaw and ground truth yaw over time with moving averages.
    """
    plt.figure(figsize=(12, 6))

    # Plot raw values with transparency
    plt.plot(df['timestamp'], df['yaw'], label='Sensor Fusion Yaw (Raw)', alpha=0.3)
    plt.plot(df['timestamp'], df['ground_truth_theta'], label='Ground Truth Yaw (Raw)', alpha=0.3)

    # Plot moving averages
    plt.plot(df['timestamp'], moving_average(df['yaw'], window_size), label='Sensor Fusion Yaw (Moving Avg)',
             linewidth=2)
    plt.plot(df['timestamp'], moving_average(df['ground_truth_theta'], window_size),
             label='Ground Truth Yaw (Moving Avg)', linewidth=2)

    plt.xlabel("Timestamp")
    plt.ylabel("Yaw Angle (deg)")
    plt.title("Sensor Fusion Yaw vs. Ground Truth Yaw (with Moving Average)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


def plot_error_time_series(df, window_size=10):
    """
    Plot yaw, roll, and pitch errors over time with moving averages in separate subplots.
    """
    error_components = {
        'yaw_error': 'Yaw Error',
        'roll_error': 'Roll Error',
        'pitch_error': 'Pitch Error'
    }

    # Determine how many plots are needed based on available columns
    available_errors = [error for error in error_components if error in df.columns]
    num_plots = len(available_errors)

    if num_plots == 0:
        print("No error columns (yaw_error, roll_error, pitch_error) found in the DataFrame.")
        return

    fig, axes = plt.subplots(num_plots, 1, figsize=(14, 5 * num_plots), sharex=True)

    # Ensure axes is iterable even if there's only one subplot
    if num_plots == 1:
        axes = [axes]

    for ax, error in zip(axes, available_errors):
        ax.plot(df['timestamp'], df[error], label=f'{error_components[error]} (Raw)', alpha=0.3)
        ax.plot(df['timestamp'], moving_average(df[error], window_size),
                label=f'{error_components[error]} (Moving Avg)', linewidth=2)

        ax.set_ylabel("Error (deg)")
        ax.set_title(f"{error_components[error]} Over Time")
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Timestamp")
    plt.tight_layout()
    plt.show()


def plot_error_histogram(df):
    """
    Plot a histogram of the yaw error distribution.
    """
    plt.figure(figsize=(10, 6))
    plt.hist(df['yaw_error'], bins=30, color='orange', edgecolor='black', alpha=0.7)
    plt.xlabel("Yaw Error (deg)")
    plt.ylabel("Frequency")
    plt.title("Histogram of Yaw Error")
    plt.grid(True, alpha=0.3)
    plt.show()


def main(csv_file, window_size=10):
    # Load the CSV data
    df = load_data(csv_file)
    # Compute the error between sensor fusion yaw and the ground truth
    df = compute_yaw_error(df)
    # Print error statistics
    print_error_statistics(df)
    # Plot sensor fusion vs. ground truth over time with moving averages
    plot_time_series(df, window_size)
    # Plot the yaw error over time with moving averages
    plot_error_time_series(df, window_size)
    # Plot a histogram of the yaw error
    plot_error_histogram(df)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze sensor fusion yaw error against ground truth")
    parser.add_argument("csv_file", help="Path to CSV file containing sensor data")
    parser.add_argument("--window_size", type=int, default=20, help="Window size for moving average (default: 10)")
    args = parser.parse_args()
    main(args.csv_file, args.window_size)
