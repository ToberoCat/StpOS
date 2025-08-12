from typing import Any, Callable, Union, Optional

from libstp.datatypes import ConditionalResult, Speed
from libstp.datatypes import for_ccw_rotation as for_ccw_condition
from libstp.datatypes import for_cw_rotation as for_cw_condition
from libstp.datatypes import for_seconds as for_seconds_condition
from libstp.device import NativeDevice

from libstp_helpers.api.steps import Step, StepProtocol
from libstp_helpers.utility import to_task


class Drive(Step):
    def __init__(self,
                 condition: Callable[[bool], ConditionalResult],
                 speed: Union[Speed, Callable[[ConditionalResult], Speed]],
                 do_correction: bool = True,
                 ):
        """
        Initialize the Drive step.

        Args:
            condition: A callable that returns a ConditionalResult.
            speed: The speed at which to drive, or a callable that takes a ConditionalResult and returns a Speed.
            do_correction: Whether to apply correction during driving.
        """
        if not callable(condition):
            raise TypeError(f"Condition must be callable, got {type(condition)}")
        if not (isinstance(speed, Speed) or callable(speed)):
            raise TypeError(f"Speed must be of type Speed or callable, got {type(speed)}")
        if not isinstance(do_correction, bool):
            raise TypeError(f"do_correction must be a boolean, got {type(do_correction)}")
        if condition(True) is None:
            raise ValueError("Condition cannot be None")

        super().__init__()
        self.condition = condition

        # If speed is a constant, convert it to a callable
        if isinstance(speed, Speed):
            self._speed_callable = lambda _: speed
        else:
            self._speed_callable = speed

        self.do_correction = do_correction
        self.device = None

    def should_continue_moving(self) -> bool:
        """
        Drive steps should allow continued movement if the next step is also a drive.
        
        Returns:
            bool: True to indicate that the robot can continue moving
        """
        return True

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Run the Drive step.
        
        :param device: The device to drive.
        :param definitions: Additional definitions required by the step.
        """
        await super().run_step(device, definitions)
        self.device = device  # Store device for on_exit usage
        await to_task(device.set_speed_while(self.condition,
                                             self._speed_callable,
                                             do_correction=self.do_correction,
                                             auto_stop_device=False,
                                             reset_ramps=False))

    def call_on_exit(self, next_step: Optional[StepProtocol] = None) -> None:
        self.device.reset_state()

        if next_step is None or not next_step.should_continue_moving():
            self.debug("Next step doesn't continue movement, stopping device")
            self.device.stop()
            self.device.reset_ramps()
        else:
            self.debug("Next step continues movement, skipping device reset")


def drive_forward(seconds: float, speed: float, do_correction=True) -> Drive:
    """Drive forward for a specified duration at a given speed"""
    return Drive(for_seconds_condition(seconds), Speed(speed, 0, 0), do_correction)


def drive_backward(seconds: float, speed: float, do_correction=True) -> Drive:
    """Drive backward for a specified duration at a given speed"""
    return Drive(for_seconds_condition(seconds), Speed(-speed, 0, 0), do_correction)


def strafe_left(seconds: float, speed: float, do_correction=True) -> Drive:
    """Strafe left for a specified duration at a given speed"""
    return Drive(for_seconds_condition(seconds), Speed(0, speed, 0), do_correction)


def strafe_right(seconds: float, speed: float, do_correction=True) -> Drive:
    """Strafe right for a specified duration at a given speed"""
    return Drive(for_seconds_condition(seconds), Speed(0, -speed, 0), do_correction)


def turn_cw(degrees: float, speed: float, do_correction=True) -> Drive:
    """Turn clockwise by specified degrees at a given speed"""
    return Drive(for_cw_condition(degrees), Speed(0, 0, -speed), do_correction)


def turn_ccw(degrees: float, speed: float, do_correction=True) -> Drive:
    """Turn counter-clockwise by specified degrees at a given speed"""
    return Drive(for_ccw_condition(degrees), Speed(0, 0, speed), do_correction)