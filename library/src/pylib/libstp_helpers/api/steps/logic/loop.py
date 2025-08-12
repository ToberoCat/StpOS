from typing import Any

from libstp.device import NativeDevice

from libstp_helpers.api.steps import Step, StepProtocol


class LoopForeverStep(Step):
    def __init__(self, step: StepProtocol):
        """
        Initialize a LoopForeverStep that runs another step indefinitely.

        Args:
            step: The step to execute in a loop.

        Raises:
            TypeError: If step is not a Step instance.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        self.step = step

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Run the step indefinitely.

        Args:
            device: The device to run on.
            definitions: Additional definitions needed for execution.
        """
        await super().run_step(device, definitions)

        while True:
            await self.step.run_step(device, definitions)

class LoopForStep(Step):
    def __init__(self, step: StepProtocol, iterations: int):
        """
        Initialize a LoopForStep that runs another step a specified number of times.

        Args:
            step: The step to execute in a loop.
            iterations: Number of times to run the step.

        Raises:
            TypeError: If step is not a Step instance.
            ValueError: If iterations is not a positive integer.
        """
        super().__init__()

        if not isinstance(step, StepProtocol):
            raise TypeError(f"Expected step to be a Step instance, got {type(step)}")

        if not isinstance(iterations, int) or iterations <= 0:
            raise ValueError(f"Iterations must be a positive integer, got {iterations}")

        self.step = step
        self.iterations = iterations

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Run the step for the specified number of iterations.

        Args:
            device: The device to run on.
            definitions: Additional definitions needed for execution.
        """
        await super().run_step(device, definitions)

        for _ in range(self.iterations):
            await self.step.run_step(device, definitions)

def loop_forever(step: StepProtocol) -> LoopForeverStep:
    """
    Create a step that runs another step indefinitely.

    Args:
        step: The step to execute in a loop.

    Returns:
        LoopForeverStep: The step that runs the provided step indefinitely.
    """
    return LoopForeverStep(step=step)

def loop_for(step: StepProtocol, iterations: int) -> LoopForStep:
    """
    Create a step that runs another step a specified number of times.

    Args:
        step: The step to execute in a loop.
        iterations: Number of times to run the step.

    Returns:
        LoopForStep: The step that runs the provided step for the specified iterations.
    """
    return LoopForStep(step=step, iterations=iterations)