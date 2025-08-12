from libstp.filter import FirstOrderLowPassFilter
from libstp.sensor import LightSensor

from libstp_helpers.api.hardware.single_line_follow_sensor import SingleLineFollowSensor


class AdvancedLightSensor(LightSensor, SingleLineFollowSensor):
    """
    This light sensor extends upon the normal light sensor.
    It will now try to handle dynamic thresholds,
    compute how much the sensor sees white or black
    and tries to use this information more smartly

    High values are black, low values are white
    """

    def line_confidence(self) -> float:
        return self.get_black_confidence()

    def __init__(self, port, calibration_factor=0.5, is_black_confidence: float = 0.8,
                 is_white_confidence: float = 0.8):
        super().__init__(port, calibration_factor)
        self.epsilon = 1e-5
        self.is_black_confidence = is_black_confidence
        self.is_white_confidence = is_white_confidence
        self._low_pass = FirstOrderLowPassFilter(0.7)

    def _get_normalized(self):
        """Returns a float between 0.0 (white) and 1.0 (black)."""
        raw = self.get_value()
        filtered = self._low_pass(raw)
        range_ = self.black_threshold - self.white_threshold
        if range_ < self.epsilon:
            raise ValueError(
                f"The black and white thresholds are not as expected: Black should be higher than white, but got: B: {self.black_threshold} W: {self.white_threshold}")
        norm = (filtered - self.white_threshold) / range_
        return max(0.0, min(1.0, norm))

    def get_black_confidence(self):
        """Returns the confidence of the sensor being on black."""
        return self._get_normalized()

    def get_white_confidence(self):
        """Returns the confidence of the sensor being on white."""
        return 1 - self._get_normalized()

    def is_on_black(self):
        return self.get_black_confidence() > self.is_black_confidence

    def is_on_white(self):
        return self.get_white_confidence() > self.is_white_confidence

    def print_info(self):
        print(f"Black: Mean: {self.black_mean}, Std: {self.black_std_dev}, Threshold: {self.black_threshold}")
