# The functional approach might be more suitable to use when having a single file
# Best practice would be to use a mix of oop and functional
# Author: Tobias Madlberger

from libstp.backend import WombatRobotBackend
from libstp.sensor import LightSensor, calibrate_light_sensors, calibrate_magnetometer
from libstp_helpers import run_as_module
from libstp_helpers.defintions import define_definition

robot = WombatRobotBackend()

# Definitions - Could be in a separate file
left_light_sensor: LightSensor = robot.create_light_sensor(0)
right_light_sensor: LightSensor = robot.create_light_sensor(1)
wait_for_light_sensor: LightSensor = robot.create_light_sensor(5)


# Setup - Could be in a separate file
def setup():
    run_as_module("calibrate-magneto", calibrate_magnetometer)
    run_as_module("calibrate-light", calibrate_light_sensors, [left_light_sensor, right_light_sensor])

    # Setup your robot here


def main():
    run_as_module("wait-for-light", wait_for_light_sensor.wait_for_light)


if __name__ == "__main__":
    run_as_module("setup", setup)
    run_as_module("main", main)
