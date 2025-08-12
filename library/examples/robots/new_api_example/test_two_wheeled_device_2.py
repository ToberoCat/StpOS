import asyncio

from libstp.datatypes import Axis, Speed, for_cw_rotation, while_false, Direction
from libstp.device.two_wheeled import TwoWheeledDevice
from libstp.motor import Motor
from libstp.sensor import is_button_clicked
from libstp_helpers.utility import to_task

left_motor = Motor(0)
right_motor = Motor(1)


async def main():
    with TwoWheeledDevice(
            Axis.Z,
            Direction.Normal,
            left_motor,
            right_motor
    ) as device:  # type: TwoWheeledDevice
        device.wheel_base = 0.1796
        device.ticks_per_revolution = 1571
        # calibrate_ticks_per_revolution(device, 2, 5)
        # calibrate_wheel_base(device, 5)
        device.set_vx_pid(1.0, 0.0, 0.0)
        device.set_w_pid(1.0, 0.0, 0.0)
        # device.set_heading_pid(1.0, 0.0, 0.0)
        device.set_heading_pid(-5.0, 0.0, 0.0)

        await to_task(device.drive_straight(while_false(is_button_clicked), Speed(0.4, 0.0)))
        await asyncio.sleep(1)
        await to_task(device.rotate(for_cw_rotation(90), Speed.Medium))


asyncio.run(main())
