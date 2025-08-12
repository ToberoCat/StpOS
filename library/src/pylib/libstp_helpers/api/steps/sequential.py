from typing import List, Any, Optional

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step, StepProtocol
from libstp_helpers.utility.logging import log

class Sequential(Step):
    """
    Sequential step executor that runs steps in a sequential order.
    Each step will be executed only once.
    """

    def __init__(self, steps: List[Step]) -> None:
        """
        Initialize Sequential step executor.
    
        Args:
            steps: List of Step objects to execute sequentially.
            
        Raises:
            TypeError: If any element in steps is not a Step instance.
        """
        super().__init__()
        
        if not isinstance(steps, list):
            raise TypeError(f"Expected steps to be a List[Step], got {type(steps)}")

        for i, step in enumerate(steps):
            if not isinstance(step, StepProtocol):
                raise TypeError(f"Element at index {i} is not a Step instance: {type(step)}")

        self.steps: List[Step] = steps
        self._last_internal_step: Optional[Step] = steps[-1] if steps else None

    def should_continue_moving(self) -> bool:
        """
        Returns the should_continue_moving value of the first step in the sequence.
        
        Returns:
            bool: True if the first step in the sequence should continue moving, False otherwise.
            If there are no steps, returns False.
        """
        if not self.steps:
            return False
        return self.steps[0].should_continue_moving()

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute each step in sequence, passing device and definitions to each step.
        Can only be run once.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Raises:
            RuntimeError: If attempting to run this sequence more than once
        """
        # Call parent's run_step which will check if this step has already run
        # and set the _has_run flag to True
        await super().run_step(device, definitions)
        
        # Now execute all child steps
        for i, step in enumerate(self.steps):
            await step.run_step(device, definitions)

            # Call the on_exit callback with information about the next step
            if i < len(self.steps) - 1:
                next_step = self.steps[i + 1]
                step.call_on_exit(next_step)

    def call_on_exit(self, next_step: Optional[StepProtocol] = None) -> None:
        """
        Called when the Sequential step is about to exit.
        This is where we finally call the last internal step's on_exit with correct next step.
        """
        super().call_on_exit(next_step)
    
        if self._last_internal_step is not None:
            self._last_internal_step.call_on_exit(next_step)
            self._last_internal_step = None

@log
def seq(steps: List[Step]) -> Sequential:
    """Create a sequential sequence of steps"""
    return Sequential(steps)

