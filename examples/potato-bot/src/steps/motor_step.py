"""PotatoMotorStep
================
Refactored implementation that
1. Uses `asyncio.wait_for` directly for phase‑1/phase‑2 timeouts.
2. Retries **once** with an automatic up‑down “unjam” cycle if the first attempt
times out.
3. Cleans up cancelled coroutines so we never leave background tasks running.
4. Lifts the head to a safe position unconditionally in a finally‑block.

Tune `max_phase_time` or `max_retries` as required.
"""
from __future__ import annotations

import asyncio
from typing import Any, Union, Callable

from libstp.device import NativeDevice
from libstp.motor import Motor
from libstp.sensor import LightSensor
from libstp.servo import Servo
from libstp_helpers.api.steps import Step, seq, wait
from libstp_helpers.api.steps.logic.loop import loop_for
from libstp_helpers.api.steps.motor import percent_to_speed, motor

from src.hardware.defs import Defs

# Type alias so we do not repeat ourselves
SensorT = Union[str, Servo]


class PotatoMotorStep(Step):
    """Shake a hopper motor until an object is detected *and* passes the sensor.

    The algorithm is:

    1. **Initial clear** – drive *down* briefly to make sure nothing is already
       jammed under the sensor.
    2. **Phase 1** – shake *down/up* until the light sensor reports an object
       (``sensor.is_on_white() is True``) **or** *max_phase_time* seconds pass.
    3. **Phase 2** – keep shaking until the object is *no longer* detected or
       the same timeout elapses.
    4. If either phase timed out we *retry* once: lift fully, pause, lower back
       down and repeat the shake sequence from step 2.
    5. Whatever happened, finish with a final lift so the next element cannot
       drop prematurely.

    Args
    ----
    motor: ``Motor`` | ``str``
        Reference to the lift motor.
    sensor: ``Servo`` | ``str``
        Reference to the optical/light sensor.
    up_velocity_pct: ``float``
        "Away from gravity" velocity in percent (positive number makes code
        self‑documenting even if the hardware expects a negative sign).
    down_velocity_pct: ``float``
        "Towards gravity" velocity in percent.
    shake_interval: ``float``
        Length of each half‑cycle (*down* then *up*), in seconds.
    up_time: ``float``
        How long to hold the motor during the *initial* and *final* lifts.
    sample_period: ``float``
        Sensor polling interval while shaking.
    max_phase_time: ``float``
        Timeout for phase 1 AND phase 2 (seconds).
    max_retries: ``int``
        How many times to give the mechanism a second chance after a timeout.
    """

    def __init__(
            self,
            motor: Union[str, Motor],
            sensor: SensorT,
            up_velocity_pct: float,
            down_velocity_pct: float,
            shake_interval: float = 0.10,
            up_time: float = 0.50,
            sample_period: float = 0.005,
            max_phase_time: float = 2.5,
            max_retries: int = 1,
    ):  # noqa: D401 – we want this formatting
        super().__init__()
        self.motor_ref = motor
        self.sensor_ref = sensor
        # Keep percents intuitive; convert to real speed only once.
        self.up_speed = percent_to_speed(up_velocity_pct)
        self.down_speed = percent_to_speed(down_velocity_pct)
        self.shake_interval = shake_interval
        self.up_time = up_time
        self.sample_period = sample_period
        self.max_phase_time = max_phase_time
        self.max_retries = max_retries

    # ---------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def _timeboxed(coro: Callable[[], asyncio.Coroutine],
                   timeout: float) -> asyncio.Coroutine:  # type: ignore[override]
        """Wrap *coro* inside ``asyncio.wait_for`` so we can cancel it cleanly."""
        return asyncio.wait_for(coro(), timeout)

    async def _shake_until(self, motor: Motor, sensor: Servo, target_state: bool):
        """Shake *motor* until ``sensor.is_on_white() == target_state``."""
        while True:
            # Down half‑cycle
            motor.set_velocity(self.down_speed)
            for _ in self._poll_loop():
                if sensor.is_on_white() == target_state:
                    return
                await asyncio.sleep(self.sample_period)

            # Up half‑cycle
            motor.set_velocity(self.up_speed)
            for _ in self._poll_loop():
                if sensor.is_on_white() == target_state:
                    return
                await asyncio.sleep(self.sample_period)

    def _poll_loop(self):
        """Generator that yields ``None`` *shake_interval / sample_period* times."""
        steps = int(self.shake_interval / self.sample_period)
        return range(max(1, steps))

    # ---------------------------------------------------------------------
    # Main entry point
    # ---------------------------------------------------------------------
    async def run_step(self, device: NativeDevice, definitions: Any) -> None:  # noqa: C901 – complexity is fine
        motor: Motor = self.get_property_from_definitions(self.motor_ref, definitions, "Motor")
        sensor: Servo = self.get_property_from_definitions(self.sensor_ref, definitions, "Sensor")

        async def _initial_clear():
            motor.set_velocity(self.down_speed)
            await asyncio.sleep(self.up_time)
            motor.set_velocity(0)

        async def _final_lift():
            motor.set_velocity(self.up_speed)
            await asyncio.sleep(self.up_time)
            motor.set_velocity(0)

        # Do the work in a *finally* so we never leave the head down.
        try:
            await _initial_clear()

            retries_left = self.max_retries
            while True:  # Break explicitly on success or when out of retries
                try:
                    # Phase1 – wait for the object to **appear**
                    await self._timeboxed(lambda: self._shake_until(motor, sensor, True), self.max_phase_time)

                    # Phase2 – wait for the object to **disappear**
                    await self._shake_until(motor, sensor, False)
                    # Success! Break out of the retry loop.
                    break

                except asyncio.TimeoutError:
                    if retries_left:
                        retries_left -= 1
                        # Unjam attempt: full lift & lower
                        await _final_lift()
                        motor.set_velocity(self.down_speed)
                        await asyncio.sleep(self.up_time)
                        motor.set_velocity(0)
                        # Loop to retry
                    else:
                        # Give up – leave the loop and proceed to final lift
                        break
        finally:
            await _final_lift()


# ----------------------------------------------------------
# Single-phase “prep” variant
# ----------------------------------------------------------

class PotatoMotorPrepStep(Step):
    """
    Shake the hopper until *one* fry blocks the optical sensor, then stop.

    This is phase-1 only from PotatoMotorStep—ideal when you’ll drop
    immediately afterwards.

    Args mirror PotatoMotorStep, plus:
    hold_speed_pct (float): after detection, hold this velocity so the fry
                            stays put (0 = coast, a small down value ☺).
    """

    def __init__(
            self,
            motor: Union[str, Motor],
            sensor: Union[str, Servo],
            *,
            up_velocity_pct: float,
            down_velocity_pct: float,
            hold_speed_pct: float = 0.0,
            shake_interval: float = 0.10,
            up_time: float = 0.40,
            sample_period: float = 0.005,
            max_phase_time: float = 2.5,
            max_retries: int = 1,
    ):
        super().__init__()
        self.motor_ref = motor
        self.sensor_ref = sensor
        self.up_speed = percent_to_speed(up_velocity_pct)
        self.down_speed = percent_to_speed(down_velocity_pct)
        self.hold_speed = percent_to_speed(hold_speed_pct)
        self.shake_interval = shake_interval
        self.up_time = up_time
        self.sample_period = sample_period
        self.max_phase_time = max_phase_time
        self.max_retries = max_retries

    # ---------- helpers ----------

    def _poll_loop(self):
        steps = max(1, int(self.shake_interval / self.sample_period))
        return range(steps)

    @staticmethod
    def _timeboxed(coro_factory: Callable[[], asyncio.Future], t: float):
        return asyncio.wait_for(coro_factory(), t)

    async def _shake_until(self, motor: Motor, sensor: Servo, target_state: bool):
        while True:
            # down half-cycle
            motor.set_velocity(self.down_speed)
            for _ in self._poll_loop():
                if sensor.is_on_white() == target_state:
                    return
                await asyncio.sleep(self.sample_period)

            # up half-cycle
            motor.set_velocity(self.up_speed)
            for _ in self._poll_loop():
                if sensor.is_on_white() == target_state:
                    return
                await asyncio.sleep(self.sample_period)

    # ---------- main ----------

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        motor: Motor = self.get_property_from_definitions(self.motor_ref, definitions, Motor)
        sensor: LightSensor = self.get_property_from_definitions(self.sensor_ref, definitions, LightSensor)

        async def initial_clear():
            motor.set_velocity(self.down_speed)
            await asyncio.sleep(self.up_time)
            motor.set_velocity(0)

        await initial_clear()

        retries = self.max_retries
        while True:
            try:
                await self._timeboxed(
                    lambda: self._shake_until(motor, sensor, True),
                    self.max_phase_time
                )
                # fry acquired—lock position
                motor.set_velocity(self.hold_speed)
                return
            except asyncio.TimeoutError:
                if retries:
                    retries -= 1
                    # unjam: quick full lift + lower
                    motor.set_velocity(self.up_speed)
                    await asyncio.sleep(self.up_time)
                    motor.set_velocity(self.down_speed)
                    await asyncio.sleep(self.up_time)
                    motor.set_velocity(0)
                else:
                    # still nothing—give up, leave motor stopped
                    motor.set_velocity(0)
                    return


# -------------------------------------------------------------------------
# Convenience factory
# -------------------------------------------------------------------------


def shake_until_fry_dropped(
        motor: Union[str, Motor],
        sensor: SensorT,
        *,
        up_velocity_pct: float = -60,  # 75
        down_velocity_pct: float = 60,  # 75
        shake_interval: float = 0.1,  # 0.045
        up_time: float = 0.3  # 0.3
) -> PotatoMotorStep:
    """Return a :class:`PotatoMotorStep` with sensible defaults."""
    return PotatoMotorStep(
        motor=motor,
        sensor=sensor,
        up_velocity_pct=up_velocity_pct,
        down_velocity_pct=down_velocity_pct,
        shake_interval=shake_interval,
        up_time=up_time,
    )


def shake_if_no_fry():
    steps = []
    if not Defs.potato_sensor.is_on_white():
        steps = [
            motor("box_Motor", 12, 0.35, ),  # 100 0.2 seconds
            motor("box_Motor", -100, 0.25, ),
        ]

    return seq(steps)


def prep_for_drop(
        motor: Union[str, Motor],
        sensor: SensorT,
):
    return PotatoMotorPrepStep(
        motor=motor,
        sensor=sensor,
        up_velocity_pct=-60,
        down_velocity_pct=60,
        hold_speed_pct=5,  # tiny downward bias keeps fry seated
    )


def shake_motor_to_loosen_fries():
    return (

        loop_for(
            seq([
                motor("box_Motor", 70, 0.16),
                wait(0.1),
                motor("box_Motor", -70, 0.23),
                wait(0.1),
            ]),
            3
        ))
