import asyncio
from typing import Any, Union

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step
from libstp_helpers.utility.logging import log

class WaitForSeconds(Step):
    """
    A step that waits for a specified duration before completing.
    """

    def __init__(self, seconds: Union[float, int]) -> None:
        """
        Initialize a WaitForSeconds step.
        
        Args:
            seconds: The number of seconds to wait.
            
        Raises:
            ValueError: If seconds is negative.
        """
        super().__init__()

        if seconds < 0:
            raise ValueError(f"Wait duration cannot be negative: {seconds}")

        self.seconds = float(seconds)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Wait for the specified duration.
        
        Args:
            device: The device to run on (not used in this step)
            definitions: Additional definitions needed for execution
        """
        await super().run_step(device, definitions)

        # Simply wait for the specified duration
        await asyncio.sleep(self.seconds)


@log
def wait(seconds: float) -> WaitForSeconds:
    """Wait for specified seconds"""
    return WaitForSeconds(seconds=seconds)
