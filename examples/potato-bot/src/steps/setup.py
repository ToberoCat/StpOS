from typing import Any

from libstp.device import NativeDevice
from libstp_helpers.api.steps import Step, seq
from libstp_helpers.api.steps.custom_step import custom_step
from libstp_helpers.sensors import lazy_calibrate_light_sensors

from src.steps.servo_steps import flaschen_servo_grab


class CalibrateSlider(Step):
    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        await super().run_step(device, definitions)
        await definitions.slider.calibrate()


class CalibrateDistanceSensor(Step):
    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        await super().run_step(device, definitions)
        await definitions.bottle_sensor.calibrate()


def full_speed_pid():
    return custom_step(lambda device, defs: device.set_heading_pid(-5.0, -0.1, 0.0))


def slow_speed_pid():
    return custom_step(lambda device, defs: device.set_heading_pid(-1.0, 0.0, 0.0))


def calibrate_light_sensors():
    return custom_step(lambda device, defs: lazy_calibrate_light_sensors([
        defs.r_sensor,
        defs.l_sensor,
        defs.potato_sensor,
    ]))


def calibrate_distance_sensor():
    return seq([
        #flaschen_servo_grab(),
        CalibrateDistanceSensor()
    ])
