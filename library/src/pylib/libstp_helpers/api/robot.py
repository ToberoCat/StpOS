import asyncio
import inspect
import time
from asyncio import CancelledError

from libstp.sensor import wait_for_button_click

from libstp_helpers import get_bool_argument
from libstp_helpers.api import ClassNameLogger
from libstp_helpers.api.missions import Mission
from libstp_helpers.api.missions.mission_controller import MissionController
from libstp_helpers.utility import to_task


# todo: Setup proper exception handling:
# todo: - when a exception gets thrown, properly log it, shutdown the robot safely
# todo: - Add Test mode and tournament mode
class Robot(ClassNameLogger):
    def __init__(self, device_cls, definitions_cls):
        self.definitions = definitions_cls()

        sig = inspect.signature(device_cls.__init__)
        params = list(sig.parameters.values())[1:]

        init_kwargs = {}
        for param in params:
            name = param.name
            if hasattr(self.definitions, name):
                init_kwargs[name] = getattr(self.definitions, name)
            elif param.default is not inspect._empty:
                continue
            else:
                raise ValueError(
                    f"Cannot bind parameter '{name}' for device {device_cls.__name__}; "
                    f"{definitions_cls.__name__} must have an attribute '{name}'."
                )

        self.device = device_cls(**init_kwargs)

        if not hasattr(self.device, "__enter__") or not hasattr(self.device, "__exit__"):
            raise ValueError(
                "The device you provided doesnt have an __enter__ or __exit__ method to be used in a context managed environment"
            )

        self._missions = []
        self._setup_mission = None
        self._shutdown_mission = None
        self._auto_shutdown_time = None
        self._light_sensor = None
        self._has_shutdown_missions_been_called = False

    def use_missions(self, *missions):
        self._missions.extend(missions)
        return self

    def set_shutdown_mission(self, mission):
        if not isinstance(mission, Mission):
            raise TypeError("The provided mission must be an instance of Mission.")

        self._shutdown_mission = mission

    def set_setup_mission(self, mission):
        if not isinstance(mission, Mission):
            raise TypeError("The provided mission must be an instance of Mission.")

        self._setup_mission = mission

    def set_auto_shutdown(self, seconds: int):
        self._auto_shutdown_time = seconds
        return self

    def set_light_sensor(self, light_sensor):
        """Set the light sensor to be used for wait_for_light."""
        self._light_sensor = light_sensor
        self.info("Light sensor set for wait_for_light.")
        return self

    async def __call_on_shutdown__(self):
        if self._shutdown_mission is None:
            return
        if self._has_shutdown_missions_been_called:
            return

        self._has_shutdown_missions_been_called = True
        start_time = time.perf_counter()
        mission_controller = MissionController(self.device, self.definitions)
        await mission_controller.execute_missions([self._shutdown_mission])
        elapsed_time = (time.perf_counter() - start_time) * 1000  # Convert to milliseconds
        if elapsed_time > 20:
            self.warn(f"Shutdown mission execution took {elapsed_time:.2f}ms, exceeding the 20ms limit.")

    async def __execute_missions__(self):
        mission_controller = MissionController(self.device, self.definitions)
        await mission_controller.execute_missions(self._missions)

    async def __stop_after__(self, main_task, start_time):
        if self._auto_shutdown_time is None:
            return

        await asyncio.sleep(self._auto_shutdown_time)
        main_task.cancel()
        self.info(f"Scheduled stop after {asyncio.get_event_loop().time() - start_time} seconds")
        self.device.stop()
        await self.__call_on_shutdown__()

    def wait_for_button_click(self):
        if not get_bool_argument("wait-for-button", True):
            return

        self.info("Waiting for button click to continue...")
        wait_for_button_click()
        self.info("Button clicked!")

    def wait_for_light(self):
        if not get_bool_argument("wait-for-light", True):
            return

        if self._light_sensor is None:
            self.warn("No light sensor set. Call set_light_sensor() before using wait_for_light.")
            return

        self.info("Waiting for light to turn on...")
        self._light_sensor.wait_for_light()
        self.info("Light turned on!")

    def start(self):
        async def _run():
            self.device.set_w_pid(0.5, 0.0, 0.0)
            self.device.set_vx_pid(1.0, 0.0, 0.0)
            self.device.set_vy_pid(1.0, 0.0, 0.0)
            self.device.set_heading_pid(5.0, 0.1, 0.0)
            self.device.set_max_accel(5, 5, 10)
            await to_task(self.device.imu.calibrate(100))

            if self._setup_mission:
                mission_controller = MissionController(self.device, self.definitions)
                await mission_controller.execute_missions([self._setup_mission])

            self.wait_for_button_click()
            self.wait_for_light()
            start_time = asyncio.get_event_loop().time()
            task = asyncio.create_task(self.__execute_missions__())
            timer = asyncio.create_task(self.__stop_after__(task, start_time))
            try:
                # Try to wait for mission execution to complete
                await task
                # If missions complete normally, cancel timer and perform shutdown
                timer.cancel()
                self.device.stop()
                await self.__call_on_shutdown__()
            except CancelledError:
                # This happens when timer cancels the main task
                self.device.stop()
                await self.__call_on_shutdown__()
                self.device.stop()

        with self.device:
            asyncio.run(_run())
