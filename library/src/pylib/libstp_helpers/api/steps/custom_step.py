import asyncio
from typing import Any, Callable, Union

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step
from libstp_helpers.utility.logging import log

class CustomStep(Step):
    """
    A step that executes a provided function (async or non-async).
    """

    def __init__(self, func: Callable[..., Any]) -> None:
        """
        Initialize a CustomStep.

        Args:
            func: The function to execute. Can be async or non-async.
        """
        super().__init__()
        if not callable(func):
            raise TypeError(f"Expected a callable, got {type(func)}")
        self.func = func

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute the provided function, awaiting if it's async.

        Args:
            device: The device to run on.
            definitions: Additional definitions needed for execution.
        """
        await super().run_step(device, definitions)

        try:
            if asyncio.iscoroutinefunction(self.func):
                await self.func(device, definitions)
            else:
                self.func(device, definitions)
        except Exception as e:
            self.error(f"Error while executing custom step: {e}")
            raise

@log
def custom_step(func: Callable[..., Any]) -> CustomStep:
    """Create a CustomStep from a function."""
    return CustomStep(func=func)
