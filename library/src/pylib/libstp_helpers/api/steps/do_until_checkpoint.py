from typing import Any

from libstp.device import NativeDevice

from libstp_helpers.api.steps import Step
from libstp_helpers.synchronizer import Synchroniser


class DoUntilCheckpoint(Step):

    def __init__(self, checkpoint: float, step) -> None:
        super().__init__()
        self.checkpoint = checkpoint
        self.step = step

    async def _job_while_wait(self, device: NativeDevice, definitions: Any):
        await self.step.run_step(device, definitions)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        await super().run_step(device, definitions)

        synchronizer = self.get_property_from_definitions("synchronizer", definitions, Synchroniser)
        await synchronizer.do_until_checkpoint(self.checkpoint, self._job_while_wait, device, definitions)


def do_until_checkpoint(checkpoint: float, step) -> DoUntilCheckpoint:
    """Run a function until a checkpoint is reached"""
    return DoUntilCheckpoint(checkpoint=checkpoint, step=step)