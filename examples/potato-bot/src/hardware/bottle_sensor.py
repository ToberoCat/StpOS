import asyncio

from libstp.datatypes import for_seconds, Speed
from libstp.device import NativeDevice
from libstp.filter import FirstOrderLowPassFilter
from libstp.logging import info
from libstp.sensor import DistanceSensor, wait_for_button_click
from libstp_helpers.utility import to_task
import numpy as np


class BottleSensor:
    def __init__(self, distance_sensor: DistanceSensor):
        self.distance_sensor = distance_sensor
        self.detect_bottles_threshold = None

    async def calibrate(self):
        async def collect_sample(samples_cnt=30):
            samples = []
            for i in range(samples_cnt):
                samples.append(self.distance_sensor.get_value())
                await asyncio.sleep(1 / samples_cnt)
            return sum(samples) / len(samples)

        info("Place a bottle in front of the sensor and press enter to calibrate.")
        wait_for_button_click()

        val = await collect_sample()
        self.detect_bottles_threshold = val * 0.98
        info(f"Calibrated bottle detection threshold: {self.detect_bottles_threshold:.2f} (Raw value: {val:.2f})")

    def is_bottle_detected(self, low_pass) -> bool:
        return  low_pass(self.distance_sensor.get_value()) > self.detect_bottles_threshold