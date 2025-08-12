import asyncio

from libstp.datatypes import Axis, Direction
from libstp.device.two_wheeled import TwoWheeledDevice
from libstp.motor import Motor

left_motor = Motor(0)
right_motor = Motor(1)

async def main():
    with TwoWheeledDevice(
            Axis.Z,
            Direction.Normal,
            left_motor,
            right_motor
    ) as device:  # type: TwoWheeledDevice
        await asyncio.sleep(1)

        # device.wheel_base = 0.1796
        # device.ticks_per_revolution = 1571
        # # calibrate_ticks_per_revolution(device, 2, 5)
        # # calibrate_wheel_base(device, 5)
        # device.set_vx_pid(1.0, 0.0, 0.0)
        # device.set_w_pid(1.0, 0.0, 0.0)
        # device.set_heading_pid(-5.0, 0.0, 0.0)


        # # device.__apply_kinematics_model__(0.0, 0.0, -0.041560657)
        # # time.sleep(3)
        # # device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.15, 0.))
        # # device.set_speed_while(for_seconds(5), Speed(0.9, 0.))
        # # device.set_speed_while(for_distance(10), Speed(0.5, 0.0))
        # result = device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.0, 0.4))
        # result.advance()
        # # device.drive_straight(for_distance(0.5), constant(Speed.Medium))
        # # device.drive_straight(while_false(lambda: is_button_clicked()), Speed.Medium)
        # # time.sleep(5)
        # # device.rotate(for_cw_rotation(90), Speed.Medium)


asyncio.run(main())
