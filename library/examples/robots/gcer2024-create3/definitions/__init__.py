import time
from typing import List

from libstp.backend import RobotBackend
from libstp.logging import info, warn
from libstp.motor import Motor
from libstp.sensor import Create3BumpSensor, Create3LightSensor, DistanceSensor
from libstp.sensor import WombatLightSensor, DigitalSensor, wait_for_any_button_click

from fullexamplecreate.definitions.motors import LightSwitchFlipperMotor
from libstp_helpers import arguments
from libstp_helpers.utility import run_async

from fullexamplecreate.definitions.servos import *


class Definitions:
    def __init__(self, robot: RobotBackend):
        self.robot: RobotBackend = robot
        self.left_outer_light_sensor = Create3LightSensor(0, calibration_factor=0.8)
        self.right_outer_light_sensor = Create3LightSensor(3, calibration_factor=0.8)

        self.left_inner_light_sensor = Create3LightSensor(1, calibration_factor=0.9)
        self.right_inner_light_sensor = Create3LightSensor(2, calibration_factor=0.8)

        self.bumper_0 = Create3BumpSensor(0)
        self.bumper_1 = Create3BumpSensor(1)
        self.bumper_2 = Create3BumpSensor(2)
        self.bumper_3 = Create3BumpSensor(3)
        self.bumper_4 = Create3BumpSensor(4)

        table_length = arguments.get("table-length")
        if table_length is None:
            warn("Enter the length of the game table (cm): ")
            table_length = input("")

        self.folding_servo = FoldingServo(1, table_length=float(table_length))
        self.pool_noodle_stopper_servo = PoolNoodleStopperServo(2)
        self.pommes_macci_hilfsHAKler = PommesLifter(0)
        self.pommes_gatsch = PommesGatsch(3)
        self.light_switch_flipper_sensor = DigitalSensor(0)
        self.light_switch_flipper_calibration_up_sensor = DigitalSensor(1)
        self.light_switch_flipper_calibration_down_sensor = DigitalSensor(2)
        self.light_switch_flipper_motor = LightSwitchFlipperMotor(
            port=0,
            up_click_sensor=self.light_switch_flipper_calibration_up_sensor,
            down_click_sensor=self.light_switch_flipper_calibration_down_sensor
        )

        self.wait_for_light_sensor = WombatLightSensor(0)
        self.front_bumpers: List[DigitalSensor] = [
            self.bumper_1,
            self.bumper_2,
            self.bumper_3,
        ]

    def any_bumper_pressed(self):
        return any(bumper.is_clicked() for bumper in self.front_bumpers)
