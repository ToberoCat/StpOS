import time
from typing import Any

from libstp.device import NativeDevice
from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import wait, Step
from libstp_helpers.api.steps.sequential import Sequential, seq

from src.hardware.defs import Defs
from src.steps.servo_steps import flaschen_servo_down

class ManualShutdown(Step):

    async def run_step(self, device: NativeDevice, definitions: Defs) -> None:
        definitions.flaschen_servo.enable()
        definitions.flaschen_servo.set_position(0)
        time.sleep(0.5)
        definitions.flaschen_servo.disable()


class ShutdownMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            ManualShutdown(),
        ])
