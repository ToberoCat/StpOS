import time
from datetime import timedelta

from libstp.backend import Create3Backend
from libstp.datatypes import Speed, SpeedType
from libstp.functions import constant, for_distance, for_ccw_rotation, for_cw_rotation, while_false, for_time
from libstp.logging import debug
from libstp_helpers.functions import while_true_timeout
from libstp_helpers.utility import run_async

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module


class GrabPurpleNoodles(Create3Module):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        super().__init__("grab_purple_noodles", robot, definitions)

    def align(self):
        # self.robot.drive_straight(for_distance(-3), constant(Speed.Fast))
        # self.robot.drive_straight(for_distance(-0.5), constant(Speed.Medium))
        self.robot.rotate(for_ccw_rotation(90), constant(Speed.VeryFast))

        self.backward_line_up(speedigus=0.06, backspeed=0.12)
        # self.robot.drive_straight(for_distance(-10), constant(Speed.VeryFast))
        # forward_line_up(self.robot, self.definitions.left_outer_light_sensor, self.definitions.right_outer_light_sensor)
        self.robot.drive_straight(for_distance(6), constant(Speed.VerySlow))
        self.robot.rotate(for_cw_rotation(90), constant(Speed.Medium))

    def run(self):
        self.definitions.pool_noodle_stopper_servo.close()
        self.align()
        self.robot.drive_straight(
            while_true_timeout(lambda: not self.definitions.light_switch_flipper_sensor.is_clicked(),
                               timedelta(seconds=2)),
            constant(Speed.VerySlow.backward()))
        self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.VerySlow))

        start_time = time.time()
        while not self.definitions.light_switch_flipper_sensor.is_clicked() and time.time() - start_time < 0.5:
            self.robot.drive_straight(for_distance(-0.1), constant(Speed.VerySlow))
        time.sleep(0.1)
        self.robot.drive_straight(for_distance(0.6), constant(Speed.VerySlow))
        self.robot.rotate(for_ccw_rotation(12), constant(Speed.Fast))

        self.definitions.folding_servo.unfold()

        self.robot.rotate(for_cw_rotation(9), constant(Speed.Medium))
        self.robot.drive_straight(for_distance(-0.8), constant(Speed.Medium))
        self.robot.rotate(for_cw_rotation(3), constant(Speed.Medium))

        def lolul():
            self.definitions.folding_servo.unfold_more()

        def loool():
            self.definitions.light_switch_flipper_motor.set_velocity(1500)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(1500)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(1500)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(1500)
            time.sleep(0.1)
            self.definitions.light_switch_flipper_motor.set_velocity(800)
            time.sleep(1)
            self.definitions.light_switch_flipper_motor.stop()

        job2 = run_async(lolul)
        self.definitions.light_switch_flipper_motor.set_velocity(800)
        job1 = run_async(loool)
        time.sleep(0.3)
        self.robot.drive_straight(for_distance(1.2), constant(Speed.Medium))
        time.sleep(0.7)
        self.robot.drive_straight(for_distance(-1), constant(Speed.Medium))
        job1.join(timeout=0.1)
        self.definitions.light_switch_flipper_motor.set_velocity(800)
        job3 = run_async(loool)
        self.robot.drive_straight(for_time(0.15), constant(Speed.Slow))

        job5 = run_async(self.definitions.folding_servo.shake_pool_noodles)
        for i in range(1, 3):
            self.robot.set_speed_while(for_time(timedelta(seconds=0.15)), constant(SpeedType(0, 10)))
            self.robot.set_speed_while(for_time(timedelta(seconds=0.20)), constant(SpeedType(0, -90)))
            self.robot.set_speed_while(for_time(timedelta(seconds=0.15)), constant(SpeedType(0, 90)))
        job3.join(timeout=0.1)
        job5.join(timeout=0.1)
        time.sleep(0.5)

        def ahaha():
            time.sleep(0.8)
            self.definitions.light_switch_flipper_motor.set_position(-220)

        run_async(ahaha)

        # self.robot.rotate(for_cw_rotation(2), constant(Speed.VerySlow))
        self.robot.drive_straight(for_distance(-3), constant(Speed.VerySlow))
        self.robot.rotate(for_ccw_rotation(5), constant(Speed.VerySlow))
        self.robot.drive_straight(for_distance(-10), constant(Speed.Medium))
        self.definitions.light_switch_flipper_motor.set_position(-220)
        job2.join(timeout=0.1)
        # self.robot.rotate(for_ccw_rotation(4), constant(Speed.Medium))
