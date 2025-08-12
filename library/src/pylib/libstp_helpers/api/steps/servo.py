import asyncio
from datetime import timedelta
from typing import Any, Union, Optional

from libstp.device import NativeDevice
from libstp.servo import Servo

from libstp_helpers.api.steps import Step
from libstp_helpers.utility import to_task

from libstp_helpers.utility.math import ease_in_ease_out

# Servo angle and speed constants
SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 170.0
# Speed at 4.8V: 0.17s per 60°
SERVO_SPEED_DPS = 60 / 0.3

# Servo position constants
SERVO_MIN_POSITION = 0
SERVO_MAX_POSITION = 2047

# When the servo doesnt know where it is, it will be at 0 - This is a issue for the time estimation!!!
def estimate_servo_move_time(start_angle: float, end_angle: float) -> float:
    """
    Estimate time (in seconds) for servo to move between two angles.
    
    Args:
        start_angle: Starting angle in degrees (0-180)
        end_angle: Ending angle in degrees (0-180)
        
    Returns:
        Estimated time in seconds
    """
    delta = abs(end_angle - start_angle)
    return delta / SERVO_SPEED_DPS


def angle_to_position(angle: float) -> int:
    """
    Convert an angle (0-180°) to a servo position (0-2047).

    Args:
        angle: Angle in degrees (0-180)

    Returns:
        Position value (0-2047)
    """
    angle = max(SERVO_MIN_ANGLE, min(angle, SERVO_MAX_ANGLE))
    return int((angle / SERVO_MAX_ANGLE) * SERVO_MAX_POSITION)


def position_to_angle(position: int) -> float:
    """
    Convert a servo position (0-2047) to an angle (0-180°).

    Args:
        position: Position value (0-2047)

    Returns:
        Angle in degrees (0-180)
    """
    position = max(SERVO_MIN_POSITION, min(position, SERVO_MAX_POSITION))
    return (position / SERVO_MAX_POSITION) * SERVO_MAX_ANGLE


class SetServoPosition(Step):
    def __init__(self, servo: Union[str, Servo], target_angle: float, time: Optional[Union[float, int]] = None) -> None:
        """
        Initialize the SetServoPosition step.

        Args:
            servo: The attribute name of the servo in the definitions object (str), or a direct reference to the servo (class attribute).
            target_angle: The target angle to set the servo to (0-180 degrees).
            time: The duration (in seconds) over which to move the servo. 
                 If None, time will be automatically calculated based on angle change.
        
        Raises:
            ValueError: If time is negative or target_angle is out of range (0.0 to 180.0).
        """
        super().__init__()

        if not (SERVO_MIN_ANGLE <= target_angle <= SERVO_MAX_ANGLE):
            raise ValueError(
                f"Target angle must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE} degrees, got {target_angle}")
        if time is not None and time < 0:
            raise ValueError(f"Time duration cannot be negative, got {time}")

        self.servo = servo
        self.target_angle = float(target_angle)
        self.time = float(time) if time is not None else None
        self.target_position = angle_to_position(self.target_angle)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Set the servo position.

        Args:
            device: The device to run on (not used in this step).
            definitions: The definitions object containing the servo attribute.
        
        Raises:
            RuntimeError: If the servo attribute does not exist or is invalid.
        """
        await super().run_step(device, definitions)

        # Get the servo object using the utility method
        servo_obj = self.get_property_from_definitions(self.servo, definitions, Servo)

        servo_obj.enable()
        if self.time is not None:
            # Use specified time with slowly_set_position
            await to_task(
                servo_obj.slowly_set_position(
                    self.target_position,
                    timedelta(seconds=self.time),
                    ease_in_ease_out
                )
            )
            return
        # Get current position (0-2047) and convert to angle
        current_position = servo_obj.get_position()
        current_angle = position_to_angle(current_position)

        # Calculate time based on current and target angle
        calculated_time = estimate_servo_move_time(current_angle, self.target_angle)

        # Use calculated time with slowly_set_position
        servo_obj.set_position(
            self.target_position,
        )
        await asyncio.sleep(calculated_time)


def servo(servo: Union[str, Servo], angle: float) -> SetServoPosition:
    """
    Create a step to set a servo to a specific angle.
    
    Args:
        servo: The attribute name of the servo in the definitions object (str), or a direct reference to the servo (class attribute).
        angle: Target angle in degrees (0-180)

    Returns:
        SetServoPosition step
    """
    return SetServoPosition(servo=servo, target_angle=angle, time=None)


def slow_servo(servo: Union[str, Servo], angle: float, time: float) -> SetServoPosition:
    """
    Create a step to set a servo to a specific angle over a specified time.

    Args:
        servo: The attribute name of the servo in the definitions object (str), or a direct reference to the servo (class attribute).
        angle: Target angle in degrees (0-180)
        time: Duration in seconds for the servo to reach the target angle

    Returns:
        SetServoPosition step
    """
    return SetServoPosition(servo=servo, target_angle=angle, time=time)


class ShakeServo(Step):
    def __init__(self, servo: Union[str, Servo], duration: float, angle_a: float, angle_b: float) -> None:
        """
        Initialize the ShakeServo step.

        Args:
            servo: The attribute name of the servo in the definitions object (str), or a direct reference to the servo (class attribute).
            duration: The duration (in seconds) to shake the servo.
            angle_a: The first angle for shaking (0-180 degrees).
            angle_b: The second angle for shaking (0-180 degrees).
        
        Raises:
            ValueError: If duration is negative or angles are out of range.
        """
        super().__init__()

        if duration < 0:
            raise ValueError(f"Duration cannot be negative, got {duration}")
        if not (SERVO_MIN_ANGLE <= angle_a <= SERVO_MAX_ANGLE):
            raise ValueError(
                f"Angle A must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE} degrees, got {angle_a}")
        if not (SERVO_MIN_ANGLE <= angle_b <= SERVO_MAX_ANGLE):
            raise ValueError(
                f"Angle B must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE} degrees, got {angle_b}")

        self.servo = servo
        self.duration = float(duration)
        self.angle_a = float(angle_a)
        self.angle_b = float(angle_b)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Shake the servo for a given duration.

        Args:
            device: The device to run on (not used in this step).
            definitions: The definitions object containing the servo attribute.
        
        Raises:
            RuntimeError: If the servo attribute does not exist or is invalid.
        """
        await super().run_step(device, definitions)

        servo_obj = self.get_property_from_definitions(self.servo, definitions, Servo)
        servo_obj.enable()

        end_time = asyncio.get_event_loop().time() + self.duration
        
        pos_a = angle_to_position(self.angle_a)
        pos_b = angle_to_position(self.angle_b)
        
        move_time = estimate_servo_move_time(self.angle_a, self.angle_b)

        while asyncio.get_event_loop().time() < end_time:
            servo_obj.set_position(pos_a)
            await asyncio.sleep(move_time)
            if asyncio.get_event_loop().time() >= end_time:
                break
            servo_obj.set_position(pos_b)
            await asyncio.sleep(move_time)


def shake_servo(servo: Union[str, Servo], duration: float, angle_a: float, angle_b: float) -> ShakeServo:
    """
    Create a step to shake a servo for a specific duration between two angles.
    
    Args:
        servo: The attribute name of the servo in the definitions object (str), or a direct reference to the servo (class attribute).
        duration: Duration in seconds for the servo to shake.
        angle_a: The first angle for shaking in degrees (0-180).
        angle_b: The second angle for shaking in degrees (0-180).

    Returns:
        ShakeServo step
    """
    return ShakeServo(servo=servo, duration=duration, angle_a=angle_a, angle_b=angle_b)
