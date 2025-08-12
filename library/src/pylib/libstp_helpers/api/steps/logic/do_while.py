import asyncio
from asyncio import CancelledError
from typing import Any

from libstp.device import NativeDevice

from libstp_helpers.api.steps import Step


class DoWhileActive(Step):

    def __init__(self, reference_step: Step, task: Step) -> None:
        super().__init__()
        self.reference_step = reference_step
        self.task = task

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        reference_step = asyncio.create_task(self.reference_step.run_step(device, definitions))
        task = asyncio.create_task(self.task.run_step(device, definitions))

        await reference_step
        task.cancel()
        try:
            await task
        except CancelledError:
            pass


def do_while_active(reference_step: Step, task: Step):
    return DoWhileActive(reference_step=reference_step, task=task)