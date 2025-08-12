from typing import Any

from libstp.datatypes import Speed, while_false
from libstp.device import NativeDevice
from libstp.sensor import LightSensor

from libstp_helpers.api.steps import seq, drive_forward
from libstp_helpers.api.steps.drive import Drive
from libstp_helpers.api.steps.sequential import Sequential


class DriveUntil(Drive):
    def __init__(
            self,
            sensor_name: str,
            forward_speed: float,
            angular_speed: float,
            black_or_white: str,
            do_correction: bool = True,
    ):
        """
        Initialize a DriveUntil step.

        Args:
            sensor_name: The name of the left light sensor property in definitions
            forward_speed: The forward speed for line following
            black_or_white: Wether to drive until a black line or until a white spot
        """
        if not isinstance(sensor_name, str):
            raise TypeError(f"sensor_name must be a string, got {type(sensor_name)}")
        if not isinstance(forward_speed, (int, float)):
            raise TypeError(f"forward_speed must be a number, got {type(forward_speed)}")
        if not isinstance(angular_speed, (int, float)):
            raise TypeError(f"angular_speed must be a number, got {type(angular_speed)}")
        if not isinstance(black_or_white, str) or black_or_white not in ["black", "white"]:
            raise TypeError(f"black_or_white must be a string that is black or white, got {type(black_or_white)}")

        self.sensor_name = sensor_name
        self.angular_speed = angular_speed
        self.forward_speed = forward_speed
        self.black_or_white = black_or_white

        # Store sensor references for efficiency
        self._sensor = None

        # Checks if given black_or_white argument is "black"
        is_black = black_or_white == "black"

        # Create check condition that that determines whether to stop or continue drive until
        def check_condition():
            if self._sensor is None:
                return False

            if is_black:
                if self._sensor.is_on_black():
                    return True
            else:
                if self._sensor.is_on_white():
                    return True
            return False

        def get_speed(_):
            if self._sensor is None:
                return Speed(self.forward_speed, self.angular_speed)
            if is_black:
                confidence = self._sensor.get_white_confidence()
            else:
                confidence = self._sensor.get_black_confidence()
            return Speed(self.forward_speed * confidence, self.angular_speed * confidence)

        super().__init__(while_false(check_condition), get_speed, do_correction)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """Run the drive until step on the given device with the given definitions."""
        # Validate sensor properties exist in definitions before proceeding
        if not hasattr(definitions, self.sensor_name):
            raise RuntimeError(f"Sensor '{self.sensor_name}' not found in definitions")

        self._sensor = getattr(definitions, self.sensor_name)

        # Type check the sensors
        if not isinstance(self._sensor, LightSensor):
            raise TypeError(f"Left sensor must be a Sensor, got {type(self._sensor)}")

        await super().run_step(device, definitions)


def drive_until_white(
        sensor_name: str,
        forward_speed: float,
        ignorance_time: float = 0.1,
        do_correction: bool = True,
) -> Sequential:
    """
    Create a DriveUntil step that drives until a white line is detected.

    Args:
        sensor_name: The name of the light sensor property in definitions
        forward_speed: The forward speed for line following
        do_correction: Whether to correct the device while driving based on gyro values

    Returns:
        A DriveUntil step configured to drive until the specified expectations are met
    """
    return seq([
        drive_forward(ignorance_time, forward_speed),
        DriveUntil(
            sensor_name,
            forward_speed=forward_speed,
            angular_speed=0,
            black_or_white="white",
            do_correction=do_correction,
        )
    ])


def drive_until_black(
        sensor_name: str,
        forward_speed: float,
        ignorance_time: float = 0.1,
        do_correction: bool = True,
) -> Sequential:
    """
    Create a DriveUntil step that drives forward until the sensor detects a black line.

    Args:
        sensor_name: The name of the light sensor property in definitions
        forward_speed: The forward speed for line following
        do_correction: Whether to correct the device while driving based on gyro values

    Returns:
        A DriveUntil step configured to drive until the specified expectations are met
    """
    return seq([
        drive_forward(ignorance_time, forward_speed),
        DriveUntil(
            sensor_name,
            forward_speed=forward_speed,
            angular_speed=0,
            black_or_white="black",
            do_correction=do_correction,
        )
    ])


def turn_cw_until_black(sensor_name: str,
                        angular_speed: float,
                        ignorance_time: float = 0.1,
                        do_correction: bool = True) -> Sequential:
    return seq([
        drive_forward(ignorance_time, angular_speed),
        DriveUntil(
            sensor_name,
            forward_speed=0,
            angular_speed=-angular_speed,
            black_or_white="black",
            do_correction=do_correction,
        )
    ])

def turn_cw_until_white(sensor_name: str,
                        angular_speed: float,
                        ignorance_time: float = 0.1,
                        do_correction: bool = True) -> Sequential:
    return seq([
        drive_forward(ignorance_time, angular_speed),
        DriveUntil(
            sensor_name,
            forward_speed=0,
            angular_speed=-angular_speed,
            black_or_white="white",
            do_correction=do_correction,
        )
    ])

def turn_ccw_until_black(sensor_name: str,
                         angular_speed: float,
                         ignorance_time: float = 0.1,
                         do_correction: bool = True) -> Sequential:
    return seq([
        drive_forward(ignorance_time, angular_speed),
        DriveUntil(
            sensor_name,
            forward_speed=0,
            angular_speed=angular_speed,
            black_or_white="black",
            do_correction=do_correction,
        )
    ])


def turn_ccw_until_white(sensor_name: str,
                         angular_speed: float,
                         ignorance_time: float = 0.1,
                         do_correction: bool = True) -> Sequential:
    return seq([
        drive_forward(ignorance_time, angular_speed),
        DriveUntil(
            sensor_name,
            forward_speed=0,
            angular_speed=angular_speed,
            black_or_white="white",
            do_correction=do_correction,
        )
    ])

