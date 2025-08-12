from typing import Any, Union

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step
from libstp_helpers.api.steps.timeout import Timeout

from src.hardware.tray_timer import TrayTimer


class TimeoutTrayTimer(Timeout):

    def __init__(self,
                 step: Step,
                 tray_timer: Union[str, TrayTimer],
                 max_timer: float) -> None:
        super().__init__(step, 1)
        self.tray_timer = tray_timer
        self.max_timer = max_timer

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        tray_timer = self.get_property_from_definitions(self.tray_timer, definitions, TrayTimer)
        timer_progress = tray_timer.get_timer_progress()

        self.timeout_seconds = self.max_timer - timer_progress
        if self.timeout_seconds <= 0:
            return

        tray_timer.resume_timer()
        await super().run_step(device, definitions)
        tray_timer.pause_timer()


def timeout_tray(
        step: Step,
        tray_timer: Union[str, TrayTimer],
        max_timer: float) -> TimeoutTrayTimer:
    return TimeoutTrayTimer(step, tray_timer, max_timer)
