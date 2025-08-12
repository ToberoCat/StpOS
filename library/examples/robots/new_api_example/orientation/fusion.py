import imufusion
import matplotlib.pyplot as plt
import numpy as np

# Import sensor data
data = np.genfromtxt("sensor_data.csv", delimiter=",", skip_header=1)

sample_rate = 100  # 100 Hz

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]
magnetometer = data[:, 7:10]

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,  # convention
    0.5,  # gain
    2000,  # gyroscope range
    10,  # acceleration rejection
    10,  # magnetic rejection
    5 * sample_rate,  # recovery trigger period = 5 seconds
)

# Process sensor data
delta_time = np.diff(timestamp, prepend=timestamp[0])

euler = np.empty((len(timestamp), 3))
internal_states = np.empty((len(timestamp), 6))
flags = np.empty((len(timestamp), 4))

for index in range(len(timestamp)):
    gyroscope[index] = offset.update(gyroscope[index])
    ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time[index])
    euler[index] = ahrs.quaternion.to_euler()

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = np.array(
        [
            ahrs_internal_states.acceleration_error,
            ahrs_internal_states.accelerometer_ignored,
            ahrs_internal_states.acceleration_recovery_trigger,
            ahrs_internal_states.magnetic_error,
            ahrs_internal_states.magnetometer_ignored,
            ahrs_internal_states.magnetic_recovery_trigger,
        ]
    )

    ahrs_flags = ahrs.flags
    flags[index] = np.array(
        [
            ahrs_flags.initialising,
            ahrs_flags.angular_rate_recovery,
            ahrs_flags.acceleration_recovery,
            ahrs_flags.magnetic_recovery,
        ]
    )


def plot_bool(x, y, label):
    plt.figure()
    plt.plot(x, y, "tab:cyan", label=label)
    plt.yticks([0, 1], ["False", "True"])
    plt.grid()
    plt.legend()
    plt.title(label)
    plt.xlabel("Time (s)")
    plt.show()


# Plot Euler angles
plt.figure()
plt.plot(timestamp, euler[:, 0], "tab:red", label="Roll")
plt.plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
plt.plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
plt.ylabel("Degrees")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()
plt.title("Euler Angles")
plt.show()

# Plot initialising flag
plot_bool(timestamp, flags[:, 0], "Initialising")

# Plot angular rate recovery flag
plot_bool(timestamp, flags[:, 1], "Angular Rate Recovery")

# Plot acceleration rejection internal states and flag
plt.figure()
plt.plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration Error")
plt.ylabel("Degrees")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()
plt.title("Acceleration Error")
plt.show()

plot_bool(timestamp, internal_states[:, 1], "Accelerometer Ignored")

plt.figure()
plt.plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration Recovery Trigger")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()
plt.title("Acceleration Recovery Trigger")
plt.show()

plot_bool(timestamp, flags[:, 2], "Acceleration Recovery")

# Plot magnetic rejection internal states and flag
plt.figure()
plt.plot(timestamp, internal_states[:, 3], "tab:olive", label="Magnetic Error")
plt.ylabel("Degrees")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()
plt.title("Magnetic Error")
plt.show()

plot_bool(timestamp, internal_states[:, 4], "Magnetometer Ignored")

plt.figure()
plt.plot(timestamp, internal_states[:, 5], "tab:orange", label="Magnetic Recovery Trigger")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()
plt.title("Magnetic Recovery Trigger")
plt.show()

plot_bool(timestamp, flags[:, 3], "Magnetic Recovery")
