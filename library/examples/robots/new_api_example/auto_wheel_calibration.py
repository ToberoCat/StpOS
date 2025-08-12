import asyncio
import itertools

import numpy as np
from libstp.datatypes import Axis, Direction, for_seconds, Speed
from libstp.device.omni_wheeled import OmniWheeledDevice
from libstp.motor import Motor
from libstp.sensor import GyroYSensor
from libstp_helpers.utility import to_task


def calibrate_motors(measured_speeds, wheel_radius, wheel_distance_from_center, target_omega=0.0):
    """
    Calibrates motor configuration by testing all permutations and sign combinations.

    measured_speeds: list of 4 measured speeds (from encoders) in rad/s.
    wheel_radius: radius of the wheels (in meters).
    wheel_distance_from_center: distance from the robot's center to each wheel (in meters).
    target_omega: the desired angular velocity (0 for driving straight).

    Returns a dictionary with the best permutation and sign corrections.
    """
    best_error = float('inf')
    best_mapping = None

    # Loop over all 4! permutations of encoder indices (i.e., which encoder reading maps to which motor)
    for perm in itertools.permutations(range(4)):
        # Loop over all 2^4 sign combinations (+1 or -1 for each motor)
        for signs in itertools.product([1, -1], repeat=4):
            # Apply permutation and sign corrections
            corrected = [signs[i] * measured_speeds[perm[i]] for i in range(4)]

            # Compute the estimated angular velocity (omega) using a simplified kinematics model.
            computed_omega = (wheel_radius / (4.0 * wheel_distance_from_center)) * (
                    -corrected[0] + corrected[1] + corrected[2] - corrected[3]
            )

            error = abs(computed_omega - target_omega)
            if error < best_error:
                best_error = error
                best_mapping = {
                    "permutation": perm,  # mapping: encoder index -> motor position order [FR, FL, RL, RR]
                    "signs": signs,  # sign correction for each motor
                    "computed_omega": computed_omega,
                    "error": best_error
                }
    return best_mapping


async def main():
    front_right_motor = Motor(0)
    front_left_motor = Motor(1)
    rear_right_motor = Motor(2)
    rear_left_motor = Motor(3)

    with OmniWheeledDevice(
            Axis.Y,
            Direction.Normal,
            front_left_motor,
            front_right_motor,
            rear_left_motor,
            rear_right_motor
    ) as device:  # type: OmniWheeledDevice
        device.wheel_distance_from_center = 0.12
        device.wheel_radius = 0.075 / 2
        device.ticks_per_revolution = 1571

        # Set PID values
        device.set_w_pid(0.5, 0.0, 0.0)
        device.set_vx_pid(1.0, 0.0, 0.0)
        device.set_vy_pid(1.0, 0.0, 0.0)
        device.set_heading_pid(5.0, 0.0, 0.0)

        # Reset motor position estimates
        for motor in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
            motor.reset_position_estimate()

        # Get initial encoder values
        fl_ticks = front_left_motor.get_current_position_estimate()
        fr_ticks = front_right_motor.get_current_position_estimate()
        rl_ticks = rear_left_motor.get_current_position_estimate()
        rr_ticks = rear_right_motor.get_current_position_estimate()

        # Start driving task
        device.__apply_kinematics_model__(0.2, 0.0, 0.0)

        # Gyro measurement loop
        start_time = asyncio.get_running_loop().time()
        heading = 0
        gyro = GyroYSensor()

        while (asyncio.get_running_loop().time() - start_time) < 5:
            v = gyro.get_value()
            now = asyncio.get_running_loop().time()
            dt = now - start_time
            start_time = now
            heading += v * dt
            await asyncio.sleep(0.01)

        # Get encoder deltas
        fl_ticks_delta = front_left_motor.get_current_position_estimate() - fl_ticks
        fr_ticks_delta = front_right_motor.get_current_position_estimate() - fr_ticks
        rl_ticks_delta = rear_left_motor.get_current_position_estimate() - rl_ticks
        rr_ticks_delta = rear_right_motor.get_current_position_estimate() - rr_ticks

        # Compute measured speeds
        delta_time = asyncio.get_running_loop().time() - start_time
        measured_speeds = [
            (2 * np.pi * fl_ticks_delta) / (device.ticks_per_revolution * delta_time),
            (2 * np.pi * fr_ticks_delta) / (device.ticks_per_revolution * delta_time),
            (2 * np.pi * rl_ticks_delta) / (device.ticks_per_revolution * delta_time),
            (2 * np.pi * rr_ticks_delta) / (device.ticks_per_revolution * delta_time)
        ]

        # Calibrate motors
        best_mapping = calibrate_motors(measured_speeds, device.wheel_radius, device.wheel_distance_from_center, 0)

        print("Best mapping found:")
        print("  Permutation (encoder index -> motor position):", best_mapping["permutation"])
        print("  Sign corrections:", best_mapping["signs"])
        print("  Computed omega:", best_mapping["computed_omega"])
        print("  Error:", best_mapping["error"])


if __name__ == "__main__":
    asyncio.run(main())
