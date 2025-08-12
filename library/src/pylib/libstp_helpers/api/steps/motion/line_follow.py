from typing import Any, Callable

from libstp import PIDController
from libstp.datatypes import ConditionalResult, Speed, while_false
from libstp.datatypes import for_seconds as for_seconds_condition
from libstp.device import NativeDevice
from libstp.sensor import LightSensor

from libstp_helpers.api.steps.drive import Drive


class LineFollow(Drive):
    def __init__(
            self,
            left_sensor_name: str,
            right_sensor_name: str,
            condition: Callable[[bool], ConditionalResult],
            forward_speed: float,
            strafe_adjustment: float = 0.05,
            forward_reduction: float = 0.5,
            pid_kp: float = 0.0,
            pid_ki: float = 0.0,
            pid_kd: float = 0.0,
    ):
        """
        Initialize a LineFollow step with PID-based steering and dynamic forward speed reduction.

        Args:
            left_sensor_name: The name of the left light sensor in definitions
            right_sensor_name: The name of the right light sensor in definitions
            condition: Callable returning ConditionalResult (stop condition)
            forward_speed: Base forward speed for line following
            strafe_adjustment: Scale factor for strafing correction
            forward_reduction: Fraction (0-1) of forward speed to reduce per unit error
            pid_kp: Proportional gain
            pid_ki: Integral gain
            pid_kd: Derivative gain
        """
        # Type checks
        if not isinstance(left_sensor_name, str):
            raise TypeError(f"left_sensor_name must be str, got {type(left_sensor_name)}")
        if not isinstance(right_sensor_name, str):
            raise TypeError(f"right_sensor_name must be str, got {type(right_sensor_name)}")
        for name, val in [
            ('forward_speed', forward_speed),
            ('strafe_adjustment', strafe_adjustment),
            ('forward_reduction', forward_reduction),
            ('pid_kp', pid_kp), ('pid_ki', pid_ki), ('pid_kd', pid_kd)
        ]:
            if not isinstance(val, (int, float)):
                raise TypeError(f"{name} must be a number, got {type(val)}")

        self.left_sensor_name = left_sensor_name
        self.right_sensor_name = right_sensor_name
        self.forward_speed = forward_speed
        self.strafe_adjustment = strafe_adjustment
        self.forward_reduction = forward_reduction

        # PID controller for steering based on sensor difference
        self.pid = PIDController(pid_kp, pid_ki, pid_kd)

        # Sensor references (populated in run_step)
        self.left_sensor: LightSensor
        self.right_sensor: LightSensor

        # Speed function using PID output for steering and dynamic forward speed
        def get_speed(_: ConditionalResult) -> Speed:
            left_conf = self.left_sensor.get_black_confidence()
            right_conf = self.right_sensor.get_black_confidence()
            diff = left_conf - right_conf  # error signal

            # PID output
            pid_out = self.pid.calculate(diff)

            # Dynamic forward speed: reduce proportional to error magnitude
            reduction = min(abs(pid_out) * self.forward_reduction, 1.0)
            fwd = self.forward_speed * (1 - reduction)

            # Strafing and rotation from PID
            strafe = -pid_out * self.strafe_adjustment
            rot = pid_out

            return Speed(fwd, strafe, rot)

        super().__init__(condition, get_speed, do_correction=False)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """Run the line follow step on the given device with the given definitions."""
        # Validate sensor properties exist in definitions before proceeding
        if not hasattr(definitions, self.left_sensor_name):
            raise RuntimeError(f"Left sensor '{self.left_sensor_name}' not found in definitions")
        if not hasattr(definitions, self.right_sensor_name):
            raise RuntimeError(f"Right sensor '{self.right_sensor_name}' not found in definitions")

        # Store sensor references for efficient access
        self.left_sensor = getattr(definitions, self.left_sensor_name)
        self.right_sensor = getattr(definitions, self.right_sensor_name)

        # Type check the sensors
        if not isinstance(self.left_sensor, LightSensor):
            raise TypeError(f"Left sensor must be a LightSensor, got {type(self.left_sensor)}")
        if not isinstance(self.right_sensor, LightSensor):
            raise TypeError(f"Right sensor must be a LightSensor, got {type(self.right_sensor)}")

        # Run the step through the parent class
        await super().run_step(device, definitions)


def follow_line(
        seconds: float,
        forward_speed: float = 1.0,
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        strafe_adjustment: float = 0.05,
        forward_reduction: float = 0.0,
        pid_kp: float = 0.75,
        pid_ki: float = 0.0,
        pid_kd: float = 0.5,
) -> LineFollow:
    """
    Create a LineFollow step that follows a line for a specified duration.
    
    Args:
        left_sensor_name: The name of the left light sensor property in definitions
        right_sensor_name: The name of the right light sensor property in definitions
        seconds: Duration to follow the line in seconds
        forward_speed: The forward speed for line following
        rotation_adjustment: How much to adjust rotation when sensor detects line
        strafe_adjustment: How much to adjust strafe when sensor detects line

    Returns:
        A LineFollow step configured to follow a line for the specified duration
    """
    return LineFollow(
        left_sensor_name=left_sensor_name,
        right_sensor_name=right_sensor_name,
        condition=for_seconds_condition(seconds),
        forward_speed=forward_speed,
        strafe_adjustment=strafe_adjustment,
        forward_reduction=forward_reduction,
        pid_kp=pid_kp,
        pid_ki=pid_ki,
        pid_kd=pid_kd,
    )


def follow_line_until_both_black(
        forward_speed: float,
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        strafe_adjustment: float = 0.05,
        forward_reduction: float = 0.0,
        pid_kp: float = 0.75,
        pid_ki: float = 0.0,
        pid_kd: float = 0.5,
) -> LineFollow:
    """
    Create a LineFollow step that follows a line until both sensors detect black.

    Args:
        forward_speed: The forward speed for line following
        left_sensor_name: The name of the left light sensor property in definitions
        right_sensor_name: The name of the right light sensor property in definitions
        rotation_adjustment: How much to adjust rotation when sensor detects line
        strafe_adjustment: How much to adjust strafe when sensor detects line

    Returns:
        A LineFollow step configured to follow a line until both sensors detect black
    """
    step_instance = None

    def both_sensors_black() -> bool:
        """Check if both sensors are detecting black"""
        if step_instance is None:
            return False

        left_black = step_instance.left_sensor.is_on_black()
        right_black = step_instance.right_sensor.is_on_black()

        return left_black and right_black

    step_instance = LineFollow(
        left_sensor_name=left_sensor_name,
        right_sensor_name=right_sensor_name,
        condition=while_false(both_sensors_black),
        forward_speed=forward_speed,
        strafe_adjustment=strafe_adjustment,
        forward_reduction=forward_reduction,
        pid_kp=pid_kp,
        pid_ki=pid_ki,
        pid_kd=pid_kd,
    )
    return step_instance
