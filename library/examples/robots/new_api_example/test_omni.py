import asyncio

from libstp.datatypes import Axis, while_true, Speed, Direction, for_cw_rotation, for_seconds
from libstp.device.omni_wheeled import OmniWheeledDevice
from libstp.filter import MovingAverageFilter
from libstp.motor import Motor
from libstp.sensor import LightSensor
from libstp_helpers.sensors import lazy_calibrate_light_sensors
from libstp_helpers.utility import to_task
from libstp_helpers.utility.math import lerp
from libstp_helpers.collision_detection import until_collision, CollisionDetector

front_right_motor = Motor(0)
front_left_motor = Motor(1)
rear_right_motor = Motor(2)
rear_left_motor = Motor(3)

async def main():
    with OmniWheeledDevice(
            Axis.X,
            Direction.Reversed,
            front_left_motor,
            front_right_motor,
            rear_left_motor,
            rear_right_motor
    ) as device:  # type: OmniWheeledDevice
        device.wheel_distance_from_center = 0.12
        device.wheel_radius = 0.075 / 2
        device.ticks_per_revolution = 1571

        # calibrate_ticks_per_revolution(device, 2, 5)
        device.set_w_pid(0.5, 0.0, 0.0)
        device.set_vx_pid(1.0, 0.0, 0.0)
        device.set_vy_pid(1.0, 0.0, 0.0)
        device.set_heading_pid(5.0, 0.15, 0.0)

        # device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.5, 0.0, 0.0))
        # device.drive_straight(while_false(lambda: is_button_clicked()), Speed.Medium)
        # time.sleep(5)
        # device.rotate(while_false(lambda: is_button_clicked()), Speed.Medium)
        # time.sleep(5)
        # device.strafe(while_false(lambda: is_button_clicked()), 45.0, 0.3)

        # device.set_speed_while(for_distance(25), Speed(0.2, 0.0, 0.0))
        # device.set_speed_while(for_cw_rotation(90), Speed(0.0, 0.0, 0.3))
        # device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.0, 0.0, 0.1))
        sensor_left = LightSensor(1, 0.50)
        sensor_right = LightSensor(0, 0.50)

        lazy_calibrate_light_sensors([
            sensor_left,
            sensor_right
        ])
        while True:
            print(sensor_left.probability_of_black())


        # time.sleep(3)
        # await WallAligner(device, 0.3).align_strafe()
        #imu = IMU()
        #imu.calibrate(100)
        # aligner = WallAligner(device, imu, sample_rate=100.0)
        # await aligner.strafe(-1)
        #await to_task(device.drive_straight(until_collision(), Speed(-1.0, 0.0, 0.0)))
        # await to_task(device.imu.calibrate(100), frequency=100)
        # await to_task(device.rotate(for_cw_rotation(180), Speed.Fastest))
        # device.reset_state()
        # await to_task(device.drive_straight(for_seconds(2), Speed.Fastest))


asyncio.run(main())
