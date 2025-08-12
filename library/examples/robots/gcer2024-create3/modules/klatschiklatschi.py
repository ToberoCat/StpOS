import time
from datetime import timedelta

from libstp.backend import Create3Backend
from libstp.datatypes import Speed
from libstp.functions import for_ccw_rotation, for_distance, constant, for_cw_rotation, for_time
from libstp_helpers.utility import run_async

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules import Create3Module
from fullexamplecreate.modules.Pommes import Pommes


class klatsch_aufeinmal(Create3Module):
    def __init__(self, robot: Create3Backend, definitions: Definitions):
        self.pommes = Pommes(robot, definitions)
        super().__init__("drop_purple_noodles", robot, definitions)

    def run(self):
        pass

    def fettige_pommsis(self):
        # self.robot.drive_straight(for_distance(-0.3), constant(Speed.VerySlow))
        job = run_async(self.definitions.light_switch_flipper_motor.set_position(-220))
        self.robot.rotate(for_cw_rotation(102), constant(Speed.Medium))
        self.drive_till_bumper(constant(Speed.Medium))
        self.robot.drive_straight(for_time(timedelta(seconds=0.1)), constant(Speed.Slow.backward()))
        # self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.VerySlow))

        # self.drive_till_bumper(constant(Speed.VerySlow))
        self.definitions.pommes_gatsch.semi_open()
        # self.robot.drive_straight(for_time(timedelta(seconds=0.2)), constant(Speed.VerySlow.backward()))
        self.pommes.pommes_medium()

        self.drive_till_bumper(constant(Speed.VerySlow))
        self.robot.drive_straight(for_distance(-0.1), constant(Speed.Medium))

        ######### Drop noodles
        self.definitions.folding_servo.unfold_drop_one()
        self.definitions.pool_noodle_stopper_servo.open()
        job = run_async(self.definitions.folding_servo.shake_pool_noodles)

        for i in range(2):
            self.robot.rotate(for_ccw_rotation(3), constant(Speed.Fast))
            self.robot.rotate(for_cw_rotation(3), constant(Speed.Fast))

        job.join(timeout=0.1)

        self.definitions.folding_servo.unfold_drop_before_fast()
        self.definitions.pool_noodle_stopper_servo.slowly_set_position(1250, timedelta(seconds=0.5))

        self.definitions.folding_servo.enable()
        self.definitions.folding_servo.slowly_set_position(1350, timedelta(seconds=1))
        self.robot.rotate(for_ccw_rotation(13), constant(Speed.VerySlow))

        self.robot.drive_straight(for_distance(-2), constant(Speed.Medium))
        self.definitions.folding_servo.unfold_drop_before()
        self.definitions.pool_noodle_stopper_servo.open()
        self.drive_till_bumper(constant(Speed.Medium))
        job = run_async(self.definitions.folding_servo.shake_pool_noodles)

        for i in range(2):
            self.robot.rotate(for_ccw_rotation(3), constant(Speed.Fast))
            self.robot.rotate(for_cw_rotation(3), constant(Speed.Fast))

        job.join(timeout=0.1)

    #
        # self.robot.rotate(for_ccw_rotation(13), constant(Speed.Medium))
        # job = run_async(self.definitions.folding_servo.shake_pool_noodles)
        # for i in range(2):
        #     self.robot.rotate(for_ccw_rotation(6), constant(Speed.Slow))
        #     self.robot.rotate(for_cw_rotation(6), constant(Speed.Fast))
        # job.join()

        time.sleep(0.3)
        self.definitions.folding_servo.fold()
        job.join(timeout=0.1)
        # self.robot.rotate(for_ccw_rotation(2), constant(Speed.VerySlow))
        self.robot.drive_straight(for_distance(-0.5), constant(Speed.VerySlow))
        self.robot.rotate(for_cw_rotation(3), constant(Speed.Medium))
        self.pommes.pommes_small()
        self.definitions.pommes_macci_hilfsHAKler.setup()
