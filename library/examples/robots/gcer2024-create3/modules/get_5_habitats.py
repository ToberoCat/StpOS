import time
from datetime import timedelta

from libstp.backend import RobotBackend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import for_ccw_rotation, constant, for_distance, for_time, for_cw_rotation, while_false
from libstp_helpers.utility import run_async, KillTimer

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module


class Get5Habitats(Create3Module):
    def __init__(self, robot: RobotBackend, definitions: Definitions):
        super().__init__("get_5_habitats", robot, definitions)

    def run(self):
        self.robot.drive_straight(for_time(timedelta(seconds=0.3)), constant(Speed.VerySlow))
        self.robot.rotate(for_cw_rotation(45), constant(Speed.VeryFast))

        def lool():
            self.definitions.pool_noodle_stopper_servo.open()
            self.definitions.light_switch_flipper_motor.set_position(-220)
            self.definitions.folding_servo.mini_fold()
        run_async(lool)
        self.backward_align()
        self.robot.drive_straight(for_distance(50), constant(SpeedType(0.13, 0)))
        self.robot.rotate(for_cw_rotation(90), constant(Speed.Fast))
        self.drive_till_bumper(constant(Speed.VerySlow))
        # self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.Medium.backward()))

        self.definitions.light_switch_flipper_motor.put_down()
        self.robot.rotate(for_ccw_rotation(45), constant(Speed.Fast))
        self.robot.drive_straight(for_time(0.1), constant(Speed.Medium.backward()))

        def shitttty_thing():
            self.definitions.light_switch_flipper_motor.set_velocity(1500)
            time.sleep(0.3)
            self.definitions.light_switch_flipper_motor.set_velocity(1200)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(1000)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(1000)
            time.sleep(0.3)
            self.definitions.light_switch_flipper_motor.set_velocity(600)
            time.sleep(0.6)
            self.definitions.light_switch_flipper_motor.set_velocity(400)
            time.sleep(0.2)
            self.definitions.light_switch_flipper_motor.stop()

        self.definitions.light_switch_flipper_motor.set_velocity(800)
        run_async(shitttty_thing)
        time.sleep(0.6)
        self.robot.drive_straight(for_time(timedelta(seconds=0.4)), constant(Speed.VerySlow.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=0.8)), constant(Speed.Slow.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.Medium.backward()))
        self.definitions.light_switch_flipper_motor.set_velocity(500)
        time.sleep(0.2)
        self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.Medium))
        self.definitions.light_switch_flipper_motor.stop()
        self.definitions.light_switch_flipper_motor.enable()

        def asdf():
            self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.Medium.backward()))
            self.definitions.light_switch_flipper_motor.set_velocity(-1000)
            time.sleep(0.4)
            self.definitions.light_switch_flipper_motor.set_velocity(1000)
            time.sleep(0.8)
            self.definitions.light_switch_flipper_motor.set_velocity(0)
            self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.Medium))
            self.definitions.light_switch_flipper_motor.stop()

        KillTimer(2, self.definitions.light_switch_flipper_motor.put_up, asdf)

        # time.sleep(1.7)
