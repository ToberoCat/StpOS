import time
from datetime import timedelta

from libstp.backend import Create3Backend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import constant, for_distance, for_time, while_false
from libstp_helpers.functions import while_true_timeout
from libstp_helpers.utility import run_async
from libstp_helpers.utility.timings import print_timestamp

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module


class PlaceAstronaut(Create3Module):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        self.robot = robot
        self.definitions = definitions
        super().__init__("place_astronaut", robot, definitions)

    def run(self):
        print_timestamp()
        # self.definitions.light_switch_flipper_motor.put_up()
        # self.robot.drive_straight(for_time(timedelta(seconds=0.05)), constant(Speed.Fast.backward()))
        # time.sleep(0.2)
        # self.robot.set_speed_while(for_time(timedelta(seconds=1.5)), constant(SpeedType(-0.16, 40)))
        # self.robot.set_speed_while(for_time(timedelta(seconds=1.4)), constant(SpeedType(0.16, -40)))

        self.backward_align(backward_time=timedelta(seconds=0.5), forward_time=timedelta(seconds=0.7))
        self.robot.drive_straight(for_time(0.2), constant(Speed.Medium))
        self.definitions.light_switch_flipper_motor.set_velocity(-500)
        time.sleep(0.1)
        self.definitions.light_switch_flipper_motor.set_velocity(-500)
        time.sleep(0.3)
        self.definitions.light_switch_flipper_motor.set_velocity(-500)
        time.sleep(0.6)
        self.definitions.light_switch_flipper_motor.set_velocity(0)

        self.robot.rotate(for_time(0.5), constant(Speed.VeryFast))
        time.sleep(0.2)

        self.definitions.light_switch_flipper_motor.set_velocity(550)
        time.sleep(0.1)
        self.definitions.light_switch_flipper_motor.set_velocity(350)
        time.sleep(0.3)
        self.definitions.light_switch_flipper_motor.set_velocity(250)
        time.sleep(0.6)
        self.definitions.light_switch_flipper_motor.set_velocity(0)

        self.robot.rotate(for_time(0.5), constant(Speed.VeryFast.backward()))
        time.sleep(0.1)
        self.robot.drive_straight(for_distance(-2), constant(Speed.VeryFast))
        # self.robot.rotate(for_ccw_rotation(90), constant(Speed.VeryFast))
        # self.robot.drive_straight(for_time(timedelta(seconds=1)), constant(Speed.Medium.backward()))

        self.definitions.light_switch_flipper_motor.set_velocity(-800)
        time.sleep(0.5)

        def keinen_sinn():
            start = time.time()
            while time.time() - start < 0.7:
                self.definitions.light_switch_flipper_motor.set_velocity(-400)
            self.definitions.light_switch_flipper_motor.stop()

        job = run_async(keinen_sinn)
        self.robot.drive_straight(for_time(timedelta(seconds=0.4)), constant(Speed.Medium))
        # time.sleep(0.3)
        # self.definitions.light_switch_flipper_motor.set_velocity(-400)

        def task():
            # time.sleep(0.6)
            # self.definitions.light_switch_flipper_motor.stop()
            self.definitions.light_switch_flipper_motor.put_down()

        run_async(task)
        self.robot.drive_straight(
            while_true_timeout(lambda: not self.definitions.light_switch_flipper_motor.is_down(), timedelta(seconds=4)),
            constant(SpeedType(0.12, 0)))
        self.robot.drive_straight(for_time(timedelta(seconds=0.23)), constant(Speed.Medium))
        job.join(timeout=0.1)
        self.definitions.light_switch_flipper_motor.stop()
        self.definitions.light_switch_flipper_motor.stop()
