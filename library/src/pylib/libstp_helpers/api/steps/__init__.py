from abc import abstractmethod
from typing import Any, runtime_checkable, Protocol, Optional, TypeVar, Union, cast

from libstp.device import NativeDevice

from libstp_helpers.api import ClassNameLogger


@runtime_checkable
class StepProtocol(Protocol):
    async def run_step(self, device: NativeDevice, definitions: Any) -> None: ...

    def call_on_exit(self, next_step: Optional['StepProtocol'] = None) -> None: ...

    def should_continue_moving(self) -> bool: ...


T = TypeVar('T')

class Step(ClassNameLogger):
    def __init__(self) -> None:
        pass

    def call_on_exit(self, next_step: Optional[StepProtocol] = None) -> None:
        """
        Call the on_exit callback if it exists.

        Args:
            next_step: The next step that will be executed, or None if this is the last step.
        """
        pass  # not implemented by step

    def should_continue_moving(self) -> bool:
        """
        Indicates whether the robot should continue moving after this step.
        
        Returns:
            bool: True if the robot should continue moving, False if it should stop.
        """
        return False  # Default behavior is to stop

    def get_property_from_definitions(self, prop: Union[str, T], definitions: Any, prop_type: T) -> T:
        """
        Extract a property from definitions when given either a string attribute name
        or a direct object reference.

        Args:
            prop: Either a string attribute name in the definitions object or a direct object reference
            definitions: The definitions object containing attributes
            prop_type: The expected type of the property

        Returns:
            The resolved object

        Raises:
            RuntimeError: If the attribute does not exist, is None, or is of the wrong type
        """
        if isinstance(prop, str):
            if not hasattr(definitions, prop):
                raise RuntimeError(f"{prop_type.__name__} attribute '{prop}' does not exist in definitions")
            obj = getattr(definitions, prop)
        else:
            obj = prop

        if obj is None:
            prop_name = prop if isinstance(prop, str) else prop_type.__name__
            raise RuntimeError(f"{prop_type.__name__} reference '{prop_name}' is None")

        return cast(T, obj)

    @abstractmethod
    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute the step logic. Can only be run once.
        
        Args:
            device: The device to run on
            definitions: Additional definitions needed for execution
            
        Raises:
            RuntimeError: If attempting to run a step that has already been executed
        """
        self.debug(f"Executing {self.__class__.__name__} step")


from libstp_helpers.api.steps.drive import (
    drive_forward,
    drive_backward,
    strafe_left,
    strafe_right,
    turn_cw,
    turn_ccw,
)
from libstp_helpers.api.steps.sequential import seq
from libstp_helpers.api.steps.parallel import parallel
from libstp_helpers.api.steps.timeout import timeout
from libstp_helpers.api.steps.wait_for_seconds import wait
from libstp_helpers.api.steps.wait_for_checkpoint import wait_for_checkpoint
from libstp_helpers.api.steps.servo import servo, slow_servo
from libstp_helpers.api.steps.motion.lineup import (
    lineup,
    forward_lineup_on_black,
    forward_lineup_on_white,
    backward_lineup_on_black,
    backward_lineup_on_white
)
from libstp_helpers.api.steps.motion.drive_until import (
    drive_until_black,
    drive_until_white,
)
from libstp_helpers.api.steps.motion.line_follow import follow_line
from libstp_helpers.api.steps.motion.single_line_follower import follow_line_single
