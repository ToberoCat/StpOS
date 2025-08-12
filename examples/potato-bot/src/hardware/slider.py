import asyncio

from libstp.logging import debug, info
from libstp.sensor import AnalogSensor, wait_for_button_click
from libstp_helpers.api.hardware.single_line_follow_sensor import SingleLineFollowSensor


class Slider(SingleLineFollowSensor):

    def __init__(self, slider: AnalogSensor, max_value = 4095):
        self.slider = slider
        self.max_value = max_value
        self.min_value = 0
        self.threshold = 0

    async def calibrate(self, samples = 30):
        info("Calibrating slider. Place at min position")

        async def collect_raw():
            data = []
            for i in range(samples):
                data.append(self.slider.get_value())
                await asyncio.sleep(1 / samples)
            return sum(data) / len(data)

        wait_for_button_click()
        self.min_value = await collect_raw()
        info(f"Slider min value: {self.min_value}. Calibrate max")

        wait_for_button_click()
        self.max_value = await collect_raw()
        info(f"Slider max value: {self.max_value}. Calibrate threshold")

        wait_for_button_click()
        self.threshold = (await collect_raw() - self.min_value) / self.max_value
        info(f"Slider threshold: {self.threshold}. Calibration done")

    def line_confidence(self) -> float:
        val = (self.slider.get_value() - self.min_value) / (self.max_value - self.min_value)
        debug(f"Slider confidence: {val}")
        return max(min(val, 1), 0)