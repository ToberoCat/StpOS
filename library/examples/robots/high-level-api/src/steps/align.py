import asyncio
from typing import Any
from libstp.datatypes import ConditionalResult, Speed, while_false
from libstp.device import NativeDevice
from libstp.sensor import LightSensor
from libstp_helpers.api.steps.drive import Drive


class LineUp(Drive):
    """
    Drives forward (or backward) until both sensors exceed a confidence threshold,
    and *scales* its speed down as you approach the line for precision.
    """
    def __init__(
            self,
            left_sensor_name: str,
            right_sensor_name: str,
            base_speed: float = 0.5,
            rotation_adjustment: float = 0.5,
            confidence_threshold: float = 0.9,
            reverse: bool = False,
    ):
        # Param checks
        for n in (left_sensor_name, right_sensor_name):
            if not isinstance(n, str):
                raise TypeError(f"Sensor name must be str, got {type(n)}")
        for v in (base_speed, rotation_adjustment, confidence_threshold):
            if not isinstance(v, (int, float)):
                raise TypeError(f"Numeric param must be int/float, got {type(v)}")
        if not 0 < confidence_threshold < 1:
            raise ValueError("confidence_threshold must be between 0 and 1")

        self.left_name = left_sensor_name
        self.right_name = right_sensor_name
        self.base_speed = base_speed
        self.rot_adj = rotation_adjustment
        self.thresh = confidence_threshold
        self.reverse = reverse

        self.left_sensor: LightSensor | None = None
        self.right_sensor: LightSensor | None = None

        def done() -> bool:
            if self.left_sensor is None or self.right_sensor is None:
                return False # This will happen as the function gets tested to return a bool
            lb = self.left_sensor.get_black_confidence()
            rb = self.right_sensor.get_black_confidence()
            if self.reverse:
                lb, rb = 1 - lb, 1 - rb
            return lb >= self.thresh and rb >= self.thresh

        def speed_fn(_: ConditionalResult) -> Speed:
            lb = self.left_sensor.get_black_confidence()
            rb = self.right_sensor.get_black_confidence()
            if self.reverse:
                lb, rb = 1 - lb, 1 - rb

            # stop if both over threshold
            if lb >= self.thresh and rb >= self.thresh:
                return Speed(0, 0, 0)

            # how far from threshold (0→far, 1→at threshold)
            max_conf = max(lb, rb)
            scale = max((self.thresh - max_conf) / self.thresh, 0.0)

            diff = lb - rb  # ∈ [–1,1]
            rot = self.rot_adj * diff
            fwd = self.base_speed * scale

            return Speed(fwd, 0, rot)

        super().__init__(while_false(done), speed_fn, do_correction=False)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        if not hasattr(definitions, self.left_name) or not hasattr(definitions, self.right_name):
            raise RuntimeError("Sensor names not found in definitions")

        self.left_sensor = getattr(definitions, self.left_name)
        self.right_sensor = getattr(definitions, self.right_name)
        if not isinstance(self.left_sensor, LightSensor) or not isinstance(self.right_sensor, LightSensor):
            raise TypeError("Both sensors must be LightSensor instances")

        await super().run_step(device, definitions)


def lineup(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.4,
        rotation_adjustment: float = 0.7,
        confidence_threshold: float = 0.8,
        reverse: bool = False,
) -> LineUp:
    """
    • base_speed: positive = forward, negative = backward (or use reverse=True)
    • rotation_adjustment & strafe_adjustment: steer gains on confidence diff
    • confidence_threshold: stop when each sensor’s target-confidence ≥ thresh
    • reverse=True: lineup on white instead of black
    """
    return LineUp(
        left_sensor_name=left_sensor_name,
        right_sensor_name=right_sensor_name,
        base_speed=base_speed,
        rotation_adjustment=rotation_adjustment,
        confidence_threshold=confidence_threshold,
        reverse=reverse,
    )
