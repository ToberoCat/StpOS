import asyncio
from typing import Any, Union

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step, StepProtocol
from libstp_helpers.utility.logging import log

class Timeout(Step):
    """
    A step that executes another step with a timeout limit.
    Provides handlers for different outcomes (completion, timeout, error).
    """

    def __init__(self,
                 step: Step,
                 timeout_seconds: Union[float, int]) -> None:
        """
        Initialize a Timeout step.
        
        Args:
            step: The step to execute with a timeout.
            timeout_seconds: Maximum execution time in seconds.
            
        Raises:
            TypeError: If step is not a Step instance.
            ValueError: If timeout_seconds is negative.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        if timeout_seconds <= 0:
            raise ValueError(f"Timeout duration must be positive: {timeout_seconds}")

        self.step = step
        self.timeout_seconds = float(timeout_seconds)
        self.result = None

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute the wrapped step with a timeout.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Returns:
            TimeoutResult: The result of the execution
        """
        await super().run_step(device, definitions)

        try:
            # Try to execute the step with a timeout
            await asyncio.wait_for(
                self.step.run_step(device, definitions),
                timeout=self.timeout_seconds
            )
        except asyncio.TimeoutError:
            self.error(f"Step timed out after {self.timeout_seconds} seconds")
        except Exception:
            raise

@log
def timeout(step: Step, seconds: float) -> Timeout:
    """Apply a timeout to a step"""
    return Timeout(step=step, timeout_seconds=seconds)
