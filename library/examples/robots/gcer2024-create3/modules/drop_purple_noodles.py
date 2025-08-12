import time
from datetime import timedelta

from libstp.backend import Create3Backend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import for_ccw_rotation, for_distance, constant, for_cw_rotation, while_false, for_time
from libstp.motion import follow_black_line_while
from libstp_helpers.utility import run_async

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module


class DropPurpleNoodles(Create3Module):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        super().__init__("drop_purple_noodles", robot, definitions)

    def run(self):
        pass

    def get_to_corner_of_nooodles(self):
        # self.robot.rotate(for_ccw_rotation(20), constant(Speed.VeryFast))
        job = run_async(self.definitions.light_switch_flipper_motor.set_position(-220))
        self.definitions.folding_servo.fold()  # ToDo: Thread this once fixed
        self.robot.rotate(for_cw_rotation(33), constant(Speed.VeryFast))
        self.robot.set_speed_while(for_time(timedelta(seconds=0.8)), constant(SpeedType(0.18, -42)))
        self.robot.set_speed_while(for_time(timedelta(seconds=1.6)), constant(SpeedType(0.18, 90)))
        # self.robot.drive_straight(for_distance(20), constant(Speed.VeryFast))
        # self.robot.rotate(for_ccw_rotation(110), constant(Speed.VeryFast))
        # self.robot.drive_straight(for_distance(10), constant(Speed.VeryFast))
        follow_black_line_while(self.robot,
                                for_time(timedelta(seconds=1.8)),
                                self.definitions.left_inner_light_sensor,
                                self.definitions.right_inner_light_sensor,
                                constant(SpeedType(0.15, 0))
                                )
        follow_black_line_while(self.robot,
                                while_false(self.definitions.any_bumper_pressed),
                                self.definitions.left_inner_light_sensor,
                                self.definitions.right_inner_light_sensor,
                                constant(SpeedType(0.5, 0))
                                )
        self.robot.drive_straight(for_distance(-0.5), constant(Speed.Medium))
        time.sleep(1)
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.VeryFast))

        self.robot.drive_straight(for_time(timedelta(seconds=0.65)), constant(Speed.VeryFast.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=1)), constant(Speed.Medium.backward()))
        self.robot.set_speed_while(for_time(timedelta(seconds=0.2)), constant(SpeedType(0, -50)))
        self.robot.drive_straight(for_time(timedelta(seconds=0.4)), constant(Speed.VeryFast.backward()))
        time.sleep(0.2)
        self.robot.drive_straight(for_time(timedelta(seconds=0.25)), constant(Speed.Slow))
        time.sleep(0.2)
        job.join(timeout=0.1)
