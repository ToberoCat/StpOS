from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps.custom_step import custom_step
from libstp_helpers.api.steps.sequential import Sequential, seq

from src.steps.setup import full_speed_pid, calibrate_light_sensors, calibrate_distance_sensor


class SetupMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            full_speed_pid(),
            calibrate_light_sensors(),
            calibrate_distance_sensor(),
            # calibrate_slider(),
            # arm_servo_oben_setup(),
            # arm_mini_servo_setup(),
            # flaschen_servo_real_setup(),
            custom_step(lambda device, _: device.set_max_speeds(9999, 9999, 9999))
        ])
