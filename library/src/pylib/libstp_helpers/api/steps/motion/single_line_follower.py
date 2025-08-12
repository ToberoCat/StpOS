from typing import Any, Callable

from libstp.datatypes import ConditionalResult, Speed
from libstp.datatypes import for_seconds as for_seconds_condition
from libstp.device import NativeDevice
from libstp.sensor import LightSensor

from libstp_helpers.api.hardware.single_line_follow_sensor import SingleLineFollowSensor
from libstp_helpers.api.steps.drive import Drive


class SingleSensorLineFollow(Drive):
    def __init__(
            self,
            sensor_name: str,
            condition: Callable[[bool], ConditionalResult],
            forward_speed: float,
            threshold: float = 0.5,
            rotation_adjustment: float = 0.25,
            strafe_adjustment: float = 0.05,
    ):
        """
        Line-follow with a single light sensor.

        Args:
            sensor_name: Name of the LightSensor in definitions.
            condition: When to stop (e.g. for_seconds_condition).
            forward_speed: Constant forward velocity.
            threshold: Black confidence above this means “on line” (black); below means white.
            rotation_adjustment: Proportional gain for turning.
            strafe_adjustment: Proportional gain for strafing (optional).
        """
        if not isinstance(sensor_name, str):
            raise TypeError(f"sensor_name must be a string, got {type(sensor_name)}")
        for param, val in [("forward_speed", forward_speed),
                           ("threshold", threshold),
                           ("rotation_adjustment", rotation_adjustment),
                           ("strafe_adjustment", strafe_adjustment)]:
            if not isinstance(val, (int, float)):
                raise TypeError(f"{param} must be a number, got {type(val)}")

        self.sensor_name = sensor_name
        self.forward_speed = forward_speed
        self.threshold = threshold
        self.rotation_adjustment = rotation_adjustment
        self.strafe_adjustment = strafe_adjustment

        self._sensor = None

        def get_speed(_: ConditionalResult) -> Speed:
            conf = self._sensor.line_confidence()
            # positive diff = more black → steer one way; negative = more white → steer other
            diff = conf - self.threshold
            rotational = self.rotation_adjustment * diff
            strafing = -self.strafe_adjustment * diff
            return Speed(self.forward_speed, strafing, rotational)

        super().__init__(condition, get_speed, do_correction=False)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        if not hasattr(definitions, self.sensor_name):
            raise RuntimeError(f"Sensor '{self.sensor_name}' not found in definitions")
        self._sensor = getattr(definitions, self.sensor_name)
        if not isinstance(self._sensor, SingleLineFollowSensor):
            raise TypeError(f"Sensor must be SingleLineFollowSensor, got {type(self._sensor)}")
        await super().run_step(device, definitions)


def follow_line_single(
        seconds: float,
        forward_speed: float,
        sensor_name: str,
        threshold: float = 0.5,
        rotation_adjustment: float = 0.25,
        strafe_adjustment: float = 0.05,
) -> SingleSensorLineFollow:
    """
    Follow a line using one light sensor for a fixed duration.

    Args:
        seconds: How long to follow.
        forward_speed: Speed along the line.
        sensor_name: Name of the sensor in your robot definition.
        threshold: Black/white discrimination cutoff (0–1).
        rotation_adjustment: Turn gain.
        strafe_adjustment: Strafe gain (if your drivetrain supports it).

    Returns:
        Configured SingleSensorLineFollow step.
    """
    return SingleSensorLineFollow(
        sensor_name=sensor_name,
        condition=for_seconds_condition(seconds),
        forward_speed=forward_speed,
        threshold=threshold,
        rotation_adjustment=rotation_adjustment,
        strafe_adjustment=strafe_adjustment,
    )
