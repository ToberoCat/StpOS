# The oop approach might be more suitable to use when distributing the code across multiple files
# Best practice would be to use a mix of oop and functional (Like this example does)
# Author: Tobias Madlberger

from libstp.backend import RobotBackend, WombatRobotBackend
from libstp.sensor import LightSensor, calibrate_light_sensors, calibrate_magnetometer
from libstp_helpers import run_as_module, Module

robot = WombatRobotBackend()

# Definitions - Could be in a separate file
left_light_sensor: LightSensor = robot.create_light_sensor(0)
right_light_sensor: LightSensor = robot.create_light_sensor(1)
wait_for_light_sensor: LightSensor = robot.create_light_sensor(5)


# Setup - Could be in a separate file
class SetupModule(Module):
    def __init__(self, robot: RobotBackend):
        super().__init__("setup")
        self.robot = robot

    def run(self):
        run_as_module("calibrate-magneto", calibrate_magnetometer)
        run_as_module("calibrate-light", calibrate_light_sensors, [left_light_sensor, right_light_sensor])
        # Setup your robot here


class MainModule(Module):
    def __init__(self, robot: RobotBackend):
        super().__init__("main")
        self.robot = robot

    def run(self):
        run_as_module("wait-for-light", wait_for_light_sensor.wait_for_light)


if __name__ == "__main__":
    SetupModule(robot)
    MainModule(robot)
