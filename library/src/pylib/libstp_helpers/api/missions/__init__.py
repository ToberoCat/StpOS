from abc import abstractmethod
from typing import Any

from libstp.device import NativeDevice
from libstp_helpers.api import ClassNameLogger

from libstp_helpers.api.steps.sequential import Sequential


class Mission(ClassNameLogger):
    async def run(self, device: NativeDevice, definitions: Any):
        """
        Execute the mission logic. Can only be run once.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Raises:
            RuntimeError: If attempting to run a mission that has already been executed
        """
        self.debug(f"Executing {self.__class__.__name__}")
        await self.sequence().run_step(device, definitions)
        self.debug(f"Completed {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> Sequential:
        raise NotImplementedError("Method sequence() not implemented")
