import time
from abc import ABC
from datetime import timedelta
from typing import Callable

from libstp.backend import RobotBackend
from libstp.datatypes import SpeedType, Speed
from libstp.functions import while_false, for_time, for_distance, constant, lerp

from fullexamplecreate.definitions import Definitions
from libstp_helpers import Module


class Create3Module(ABC, Module):
    def __init__(self, name: str, robot: RobotBackend, definitions: Definitions):
        self.robot = robot
        self.definitions = definitions
        super().__init__(name)

    def forward_line_up(self):
        def speed():
            return SpeedType.wheels(
                -0.1 if self.definitions.left_outer_light_sensor.is_on_black() else 0.1,
                -0.1 if self.definitions.right_outer_light_sensor.is_on_black() else 0.1
            )

        self.robot.set_speed_while(while_false(
            lambda: self.definitions.left_outer_light_sensor.is_on_black() or self.definitions.right_outer_light_sensor.is_on_black()),
            constant(SpeedType(0.2, 0)))
        self.robot.set_speed_while(while_false(
            lambda: self.definitions.left_outer_light_sensor.is_on_black() and self.definitions.right_outer_light_sensor.is_on_black()),
            speed)

    def backward_line_up(self, speedigus=0.06, backspeed=0.12):
        def speed():
            return SpeedType.wheels(
                speedigus if self.definitions.left_outer_light_sensor.is_on_black() else -speedigus,
                speedigus if self.definitions.right_outer_light_sensor.is_on_black() else -speedigus
            )

        self.robot.set_speed_while(while_false(
            lambda: self.definitions.left_outer_light_sensor.is_on_black() or self.definitions.right_outer_light_sensor.is_on_black()),
                                   constant(SpeedType(-backspeed, 0)))
        self.robot.set_speed_while(while_false(
            lambda: self.definitions.left_outer_light_sensor.is_on_black() and self.definitions.right_outer_light_sensor.is_on_black()),
                                   speed)

    def backward_align(self, backward_time=timedelta(seconds=0.5), forward_time=timedelta(seconds=0.3)):
        self.robot.drive_straight(for_time(backward_time), lerp(
            Speed.Medium.backward(),
            Speed.VeryFast.backward(),
            backward_time
        ))
        time.sleep(0.5)
        self.robot.drive_straight(for_time(forward_time), constant(SpeedType(0.03, 0)))

    def drive_till_bumper(self, speed: Callable[[], SpeedType]):
        self.robot.drive_straight(while_false(self.definitions.any_bumper_pressed), speed)
