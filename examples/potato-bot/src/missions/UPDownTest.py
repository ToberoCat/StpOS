from typing import Any

from libstp.device import NativeDevice
from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import turn_cw, turn_ccw, drive_forward, follow_line_single, drive_backward, \
    drive_until_black, parallel, wait, Step, forward_lineup_on_white
from libstp_helpers.api.steps.custom_step import custom_step
from libstp_helpers.api.steps.motor import motor

from libstp_helpers.api.steps.sequential import Sequential, seq
from libstp_helpers.api.steps.servo import shake_servo
from libstp_helpers.api.steps.wait_for_button import wait_for_button

from src.hardware.defs import Defs
from src.steps.bottle_sensor_step import drive_until_bottle_detected
from src.steps.crazy_align import align_on_pipe
from src.steps.motor_step import shake_until_fry_dropped, shake_motor_to_loosen_fries, prep_for_drop
from src.steps.servo_steps import flaschen_servo_grab, flaschen_servo_Setup, flaschen_servo_fahrt, flaschen_servo_down
from src.steps.tray_timer_step import timeout_tray


class ShakeBot(Step):
    def __init__(self, direction: int):
        super().__init__()
        self.direction = direction

    async def run_step(self, device: NativeDevice, definitions: Defs) -> None:
        definitions.left_front_motor.set_velocity(self.direction * 1500)
        definitions.right_front_motor.set_velocity(-self.direction * 1500)


class Shake(Mission):
    def sequence(self) -> Sequential:
        return seq([
            motor("box_Motor", -10, 0.3, ),
            wait(2),
            motor("box_Motor", 12, 0.35, ),  # 100 0.2 seconds
            motor("box_Motor", -70, 0.35, ),
            # motor("box_Motor", 70, 0.35,),
            # motor("box_Motor", -70, 0.35,),
            motor("box_Motor", 30, 0.5, ),
        ])
