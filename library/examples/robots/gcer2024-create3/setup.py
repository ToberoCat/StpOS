import time
from typing import Callable

from libstp.backend import Create3Backend
from libstp.datatypes import Speed
from libstp.functions import constant, for_distance, for_cw_rotation, for_ccw_rotation

from fullexamplecreate.definitions import Definitions
from libstp_helpers import run_as_module
from libstp_helpers.create3 import Create3SetupModule
from libstp_helpers.sensors import calibrate_light_sensors_with_properties, delete_calibrate_light_sensors_properties
from libstp_helpers.utility.timings import print_timestamp


class SetupModule(Create3SetupModule):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        self.definitions = definitions
        robot.await_commands()
        super().__init__(robot, definitions.wait_for_light_sensor)

    def register_parts(self):
        self.register_part(self.__test_and_setup__)
        self.register_part(self.__calibrate_light__)

    def __test_and_setup__(self, set_status: Callable[[int], None] = None):
        self.__setup_arm__()
        self.__calibrate_flipper__()
        self.__setup_grabber__()
        self.__test_robot_backend__()

    def __test_robot_backend__(self):
        self.robot.drive_straight(for_distance(0.1), constant(Speed.Medium))
        self.robot.rotate(for_cw_rotation(45), constant(Speed.VeryFast))

    def __setup_grabber__(self):
        print_timestamp()
        self.definitions.pommes_macci_hilfsHAKler.setup()
        self.definitions.pommes_gatsch.close()

    def __calibrate_flipper__(self):
        print_timestamp()
        self.definitions.light_switch_flipper_motor.calibrate()

    def __setup_arm__(self):
        print_timestamp()
        self.definitions.folding_servo.fold()
        self.definitions.pool_noodle_stopper_servo.close()

    def __calibrate_light__(self, set_status: Callable[[int], None] = None):
        run_as_module("calibrate", delete_calibrate_light_sensors_properties)
        calibrate_light_sensors_with_properties([
            self.definitions.left_outer_light_sensor,
            self.definitions.right_outer_light_sensor,
            self.definitions.left_inner_light_sensor,
            self.definitions.right_inner_light_sensor
        ])
