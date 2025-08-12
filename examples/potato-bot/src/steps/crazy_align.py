from typing import Callable, Union

from libstp.datatypes import ConditionalResult, Speed, while_false
from libstp_helpers.api.steps.drive import Drive

from src.hardware.defs import Defs


class AlignWithPipe(Drive):

    def __init__(self, push_me_wheel_speed: float, dont_drive_wheel_speed: float, do_correction: bool = False):
        def check():
            if Defs.l_sensor.is_on_white() and Defs.r_sensor.is_on_white():
                return True
            return False

        def get_speed(_) -> ConditionalResult:
            if Defs.l_sensor.is_on_white():
                return Speed.wheels(dont_drive_wheel_speed, push_me_wheel_speed)
            if Defs.r_sensor.is_on_white():
                return Speed.wheels(push_me_wheel_speed, dont_drive_wheel_speed)
            return Speed(0, 0)

        super().__init__(while_false(check), get_speed, do_correction)


def align_on_pipe():
    return AlignWithPipe(
        push_me_wheel_speed=-1.0,
        dont_drive_wheel_speed=0.9,
    )