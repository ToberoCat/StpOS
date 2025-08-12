import time

import numpy as np
from libstp.motor import Motor
from libstp.sensor import is_button_clicked

front_right_motor = Motor(0)
front_left_motor = Motor(1)
rear_right_motor = Motor(2)
rear_left_motor = Motor(3)


def calculate_wheel_speeds(Vx, Vy, omega, R, L):
    """
    Calculates the wheel speeds for a 4-wheeled omni-directional robot.

    Parameters:
    - Vx: Linear velocity in the x direction (m/s)
    - Vy: Linear velocity in the y direction (m/s)
    - omega: Rotational velocity (rad/s)
    - R: Radius of the wheels (m)
    - L: Distance from the center of the robot to each wheel (m)

    Returns:
    - wheel_speeds: A list containing the angular speeds of the wheels [w1, w2, w3, w4] (rad/s)
    """
    # Inverse kinematics matrix
    inv_kinematics_matrix = np.array([
        [1, 1, L],  # Front-right wheel (w1)
        [1, -1, -L],  # Front-left wheel (w2)
        [1, 1, -L],  # Back-left wheel (w3)
        [1, -1, L]  # Back-right wheel (w4)
    ])

    # Desired velocities (Vx, Vy, omega) as a vector
    velocities = np.array([Vx, Vy, omega])

    # Calculate the angular speeds of the wheels
    wheel_speeds = (1 / R) * np.dot(inv_kinematics_matrix, velocities)

    return wheel_speeds


def wheel_speed_to_ticks(wheel_speeds, ticks_per_revolution):
    """
    Converts wheel speeds (rad/s) to motor encoder ticks per second.

    Parameters:
    - wheel_speeds: List of wheel speeds in rad/s [w1, w2, w3, w4]
    - ticks_per_revolution: Number of encoder ticks per wheel revolution

    Returns:
    - ticks_per_second: List of motor speeds in ticks per second
    """
    # Conversion factor: radians to revolutions
    rad_to_revolutions = 1 / (2 * np.pi)

    # Convert each wheel speed
    ticks_per_second = [
        speed * rad_to_revolutions * ticks_per_revolution for speed in wheel_speeds
    ]

    return ticks_per_second

def set_ticks_per_seconds(tps):
    """
    Sets the motor speeds in ticks per second.

    Parameters:
    - tps: List of motor speeds in ticks per second
    """
    front_right_motor.set_velocity(int(tps[0]))
    front_left_motor.set_velocity(int(tps[1]))
    rear_left_motor.set_velocity(int(tps[2]))
    rear_right_motor.set_velocity(int(tps[3]))

# Example usage:
if __name__ == "__main__":
    # Example parameters
    Vx = 0.0  # m/s (forward velocity)
    Vy = 0.0  # m/s (sideways velocity)
    omega = 0.5  # rad/s (rotation speed)
    R = 0.075 / 2  # Wheel radius (m)
    L = 0.12  # Distance from robot center to wheel (m)
    ticks_per_revolution = 1578  # Encoder resolution

    # Calculate wheel speeds
    wheel_speeds = calculate_wheel_speeds(Vx, Vy, omega, R, L)
    print(f"Wheel Speeds (rad/s): {wheel_speeds}")

    ticks_per_second = wheel_speed_to_ticks(wheel_speeds, ticks_per_revolution)
    print(f"Ticks Per Second: {ticks_per_second}")

    set_ticks_per_seconds(ticks_per_second)
    while not is_button_clicked():
        time.sleep(0.1)
