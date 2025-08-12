from typing import Any

from libstp.datatypes import Speed, while_false
from libstp.device import NativeDevice
from libstp.filter import FirstOrderLowPassFilter
from libstp_helpers.api.steps.drive import Drive

from src.hardware.bottle_sensor import BottleSensor


class BottleSensorStep(Drive):

    def __init__(self, speed, do_correction: bool = True):

        self._bottle_sensor: BottleSensor | None = None
        self._low_pass = FirstOrderLowPassFilter(0.7)

        def condition():
            if self._bottle_sensor is None:
                return False

            return self._bottle_sensor.is_bottle_detected(self._low_pass)

        super().__init__(while_false(condition), speed, do_correction)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        if not definitions.bottle_sensor.detect_bottles_threshold:
            raise ValueError("Bottle sensor not calibrated. Please run the calibration step first.")

        self._bottle_sensor = definitions.bottle_sensor
        for i in range(100):  # warmup
            self._low_pass(self._bottle_sensor.distance_sensor.get_value())
        await super().run_step(device, definitions)


def drive_until_bottle_detected(
        speed: float,
        do_correction: bool = True
) -> BottleSensorStep:
    """
    Create a step that drives until a bottle is detected by the bottle sensor.

    Args:
        speed: The speed at which to drive.
        do_correction: Whether to apply correction during driving.

    Returns:
        BottleSensorStep: The step that drives until a bottle is detected.
    """
    return BottleSensorStep(Speed(speed, 0, 0), do_correction)
