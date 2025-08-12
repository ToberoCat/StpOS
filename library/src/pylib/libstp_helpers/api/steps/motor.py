import asyncio
from typing import Any, Optional, Union

from libstp.device import NativeDevice
from libstp.motor import Motor

from libstp_helpers.api.steps import Step

# Motor speed constants
MOTOR_MAX_SPEED = 1500  # max internal units
PERCENT_MIN = -100.0
PERCENT_MAX = 100.0


def percent_to_speed(percent: float) -> int:
    """
    Convert a velocity percentage (-100 to +100) into motor units (-1500 to +1500).
    """
    # clamp percent
    pct = max(PERCENT_MIN, min(PERCENT_MAX, percent))
    return int((pct / 100.0) * MOTOR_MAX_SPEED)


class SetMotorVelocity(Step):
    def __init__(
            self,
            motor: Union[str, Motor],
            velocity_pct: float,
            duration: Optional[Union[float, int]] = None
    ) -> None:
        """
        Step to run a motor at a given % of max speed, optionally for a fixed duration.

        Args:
            motor: name of the motor attribute in definitions (str), or a direct reference to the motor (class attribute).
            velocity_pct: desired speed as a percent (-100.0 to +100.0).
            duration: seconds to hold that speed. If None, it will run until another step changes it.

        Raises:
            ValueError: if velocity_pct is out of range or duration negative.
        """
        super().__init__()
        if not (PERCENT_MIN <= velocity_pct <= PERCENT_MAX):
            raise ValueError(f"velocity_pct must be between {PERCENT_MIN} and {PERCENT_MAX}, got {velocity_pct}")
        if duration is not None and duration < 0:
            raise ValueError(f"Duration cannot be negative, got {duration}")

        self.motor: Union[str, Motor] = motor
        self.velocity_pct = velocity_pct
        self.duration = float(duration) if duration is not None else None
        self.speed_value = percent_to_speed(self.velocity_pct)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        await super().run_step(device, definitions)

        # Accept either a string (attribute name) or a direct reference (class attribute)
        if isinstance(self.motor, str):
            if not hasattr(definitions, self.motor):
                raise RuntimeError(f"Motor attribute '{self.motor}' not found in definitions")
            motor_obj: Motor = getattr(definitions, self.motor)
        else:
            motor_obj: Motor = self.motor

        if motor_obj is None:
            raise RuntimeError(f"Motor reference '{self.motor}' is None")

        motor_obj.set_velocity(self.speed_value)

        if self.duration is not None:
            # hold for specified time, then stop
            await asyncio.sleep(self.duration)
            motor_obj.set_velocity(0)

class StopMotor(Step):

    def __init__(self, motor: Union[str, Motor]) -> None:
        """
        Step to stop a motor.

        Args:
            motor: name of the motor attribute in definitions (str), or a direct reference to the motor (class attribute).
        """
        super().__init__()
        self.motor: Union[str, Motor] = motor

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        await super().run_step(device, definitions)

        motor_obj = self.get_property_from_definitions(self.motor, definitions, Motor)
        motor_obj.stop()



def motor(motor: Union[str, Motor], pct: float, duration: Optional[float] = None) -> SetMotorVelocity:
    """
    Helper to create a SetMotorVelocity step.

    Args:
        motor: name of the motor in definitions (str), or a direct reference to the motor (class attribute).
        pct: speed percentage (-100 to +100).
        duration: optional seconds to run at that speed.
    """
    return SetMotorVelocity(motor, velocity_pct=pct, duration=duration)

def stop_motor(motor: Union[str, Motor]) -> StopMotor:
    """
    Helper to create a StopMotor step.

    Args:
        motor: name of the motor in definitions (str), or a direct reference to the motor (class attribute).
    """
    return StopMotor(motor)