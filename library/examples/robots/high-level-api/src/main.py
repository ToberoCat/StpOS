import asyncio

from libstp_helpers.api.device.two_wheeled import TwoWheeledDevice
from libstp_helpers.api.robot import Robot
from libstp_helpers.sensors import lazy_calibrate_light_sensors

from src.hardware.definitions import Definitions
from src.missions.example_mission import ExampleMission

robot = Robot(TwoWheeledDevice, Definitions)

robot.use_missions(
    ExampleMission()
)


@robot.on_shutdown
async def shutdown(device, defs):
    print("Disabling servos...")
    await asyncio.sleep(1)  # simulate wait


@robot.on_startup
async def setup(device, defs: Definitions):
    device.set_heading_pid(-5.0, -0.1, 0.0)
    lazy_calibrate_light_sensors([
        defs.left_front_sensor,
        defs.right_front_sensor
    ])


# robot.set_auto_shutdown(seconds=119)

if __name__ == "__main__":
    robot.start()
