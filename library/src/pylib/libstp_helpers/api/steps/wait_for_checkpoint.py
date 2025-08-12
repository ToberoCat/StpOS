from typing import Any, Union

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step

from libstp_helpers.synchronizer import Synchroniser
from libstp_helpers.utility.logging import log

class WaitForCheckpoint(Step):
    def __init__(self, checkpoint_seconds: Union[float, int]) -> None:
        """
        Initialize a Synchronizer step.
        
        Args:
            checkpoint_seconds: The number of seconds to wait before synchronizing.
            
        Raises:
            ValueError: If checkpoint_seconds is negative.
        """
        super().__init__()

        if checkpoint_seconds < 0:
            raise ValueError(f"Checkpoint duration cannot be negative: {checkpoint_seconds}")

        self.checkpoint_seconds = float(checkpoint_seconds)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Wait for the specified duration before synchronizing.
        
        Args:
            device: The device to run on (not used in this step)
            definitions: Additional definitions needed for execution
        """
        await super().run_step(device, definitions)

        synchronizer = self.get_property_from_definitions("synchronizer", definitions, Synchroniser)

        await synchronizer.wait_until_checkpoint(self.checkpoint_seconds)

@log
def wait_for_checkpoint(checkpoint_seconds: float) -> WaitForCheckpoint:
    """Synchronize for specified seconds"""
    return WaitForCheckpoint(checkpoint_seconds=checkpoint_seconds)