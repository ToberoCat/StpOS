from datetime import timedelta
import time

from libstp.backend import Create3Backend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import for_distance, constant, while_false, for_cw_rotation, for_ccw_rotation, for_time
from libstp.motion import follow_black_line_while
from libstp.motion.squareup import forward_line_up, backward_line_up
from libstp_helpers.utility import KillTimer

from fullexamplecreate.modules import Create3Module


class DriveToPlaceAstronautPosition(Create3Module):
    def __init__(self, robot: Create3Backend, definitions):
        super().__init__("drive_to_grab_position", robot, definitions)

    def run(self):
        self.definitions.folding_servo.fold()
        self.robot.drive_straight(for_distance(10), constant(Speed.Fast))
        self.robot.rotate(for_cw_rotation(50), constant(Speed.Fast))
        self.robot.drive_straight(for_distance(-47), constant(Speed.Fast))
        self.robot.rotate(for_cw_rotation(80), constant(Speed.Fast))
        self.robot.drive_straight(for_distance(-30), constant(Speed.Fast))

        if self.definitions.right_outer_light_sensor.is_on_black() or self.definitions.left_outer_light_sensor.is_on_black() or self.definitions.right_inner_light_sensor.is_on_black() or self.definitions.left_inner_light_sensor.is_on_black():
            while self.definitions.right_outer_light_sensor.is_on_black() or self.definitions.left_outer_light_sensor.is_on_black() or self.definitions.right_inner_light_sensor.is_on_black() or self.definitions.left_inner_light_sensor.is_on_black():
                self.robot.drive_straight(for_distance(0.1), constant(Speed.Medium))
        self.backward_line_up(backspeed=0.12)
        self.robot.drive_straight(for_distance(5), constant(Speed.Medium))
        self.backward_line_up(speedigus=0.08, backspeed=0.12)

        self.robot.drive_straight(for_distance(4), constant(Speed.Medium))
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.Medium))
