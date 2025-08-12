import time
from datetime import timedelta
from typing import Callable

from libstp.backend import Create3Backend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import constant, for_distance, for_cw_rotation, for_ccw_rotation, for_time
from libstp.logging import debug
from libstp.motion.squareup import backward_line_up
from libstp_helpers.utility.timings import print_timestamp

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module


class Pommes(Create3Module):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        super().__init__("pommes", robot, definitions)

    def run(self):
        pass

    def pommes_lets_chat(self):  # grab se last 4 pommsis
        print_timestamp()
        # time.sleep(0.5)
        # self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.Medium.backward()))
        self.robot.rotate(for_cw_rotation(2), constant(Speed.Medium))
        self.pommes_medium()

        self.robot.rotate(for_cw_rotation(13), constant(Speed.Medium))
        self.drive_till_bumper(constant(Speed.VerySlow))

        self.pommes_small()

        self.definitions.pommes_macci_hilfsHAKler.setup()

    def get_to_lets_chat(self):  # get to the second pommsis
        print_timestamp()

        # self.robot.drive_straight(for_distance(1), constant(Speed.VeryFast))
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.VeryFast))
        self.backward_align(backward_time=timedelta(seconds=0.3), forward_time=timedelta(seconds=0.5))
        time.sleep(0.5)
        self.robot.drive_straight(for_distance(28.6), constant(Speed.VeryFast))
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.VeryFast))
        self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.VeryFast.backward()))
        self.robot.set_speed_while(for_time(timedelta(seconds=1.239)), constant(SpeedType(-0.2178, 50.82)))
        time.sleep(0.5)
        self.robot.set_speed_while(for_time(timedelta(seconds=1.322)), constant(SpeedType(0.2178, -50.82)))
        time.sleep(0.3)
        self.robot.drive_straight(for_time(timedelta(seconds=0.65)), constant(Speed.VeryFast.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=1)), constant(Speed.Medium.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=0.4)), constant(Speed.VeryFast.backward()))
        self.robot.rotate(for_time(timedelta(seconds=0.2)), constant(SpeedType(0,60)))
        self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.VeryFast.backward()))
        time.sleep(0.2)
        self.robot.drive_straight(for_time(timedelta(seconds=0.25)), constant(Speed.Slow))
        time.sleep(0.2)
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.Medium))
        self.drive_till_bumper(constant(Speed.VerySlow))
        self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.Slow.backward()))

    def pommes_small(self):
        print_timestamp()
        self.definitions.pommes_gatsch.semi_open()
        self.robot.drive_straight(for_time(timedelta(seconds=0.07)), constant(Speed.VerySlow.backward()))
        self.definitions.pommes_macci_hilfsHAKler.up_half_small()
        self.robot.drive_straight(for_time(timedelta(seconds=0.3)), constant(Speed.Medium))
        self.pommes(self.definitions.pommes_macci_hilfsHAKler.up_small)

    def pommes_medium(self):
        print_timestamp()

        self.pommes(self.definitions.pommes_macci_hilfsHAKler.up_medium)

    def pommes(self, full_way: Callable[[], None]):
        print_timestamp()
        self.definitions.pommes_gatsch.semi_open()
        self.definitions.pommes_macci_hilfsHAKler.down_hovering_over_table()
        self.definitions.pommes_gatsch.open_faaast()
        self.definitions.pommes_macci_hilfsHAKler.down_touching_table()
        self.definitions.pommes_gatsch.close()

        self.robot.drive_straight(for_time(timedelta(seconds=0.005)), constant(Speed.VerySlow.backward()))
        full_way()

        # self.robot.drive_straight(for_time(timedelta(seconds=0.4)), constant(Speed.Medium.backward()))
        self.robot.drive_straight(for_distance(-0.4), constant(Speed.Medium))
        self.definitions.pommes_gatsch.open_faaast()