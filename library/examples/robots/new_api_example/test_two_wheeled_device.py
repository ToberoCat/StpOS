import asyncio
import threading
import time
from math import radians

import numpy as np
from libstp.datatypes import Axis, Speed, for_cw_rotation, while_false, Direction, for_seconds
from libstp.device.two_wheeled import TwoWheeledDevice
from libstp.motor import Motor
from libstp.sensor import is_button_clicked
from libstp_helpers.orientation import ImuFusionService
from libstp_helpers.utility import to_task

from position_fetcher import PositionFetcher

left_motor = Motor(0)
right_motor = Motor(1)
is_collecting_data = True

def collect_data_to_csv(service, position_fetcher, device, csv_file = "data"):
    # initial_angle = 0
    # for i in range(50):
    #     _, a = position_fetcher.fetch_next_position()
    #     initial_angle += a
    #
    # initial_angle /= 50
    # # moving window of 10 samples
    # queue = []
    # while is_collecting_data:
    #     position, angle = position_fetcher.fetch_next_position()
    #     queue.append(angle)
    #     if len(queue) > 10:
    #         queue.pop(0)
    #     if len(queue) == 10:
    #         smoothed_angle_radians = (sum([(i - initial_angle) for i in queue]) / 10) * np.pi / 180
    #         device.set_quaternion(0, 0, 0, smoothed_angle_radians)
            #print(f"angle: {angle}, smoothed: {smoothed_angle_radians}, initial: {initial_angle}")
        # # timestamp, gx, gy, gz, ax, ay, az, mx, my, mz, roll, pitch, yaw, ground_truth_x, ground_truth_y, ground_truth_theta
    csv_file += f"_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    with open(csv_file, "w") as csv:
        csv.write("timestamp,gx,gy,gz,ax,ay,az,mx,my,mz,roll,pitch,yaw,ground_truth_x,ground_truth_y,ground_truth_theta\n")
        while is_collecting_data:
            gyro, accel, magneto = service.imu.get_reading()
            roll, pitch, yaw = service.fusion.get_euler()
            position, angle = position_fetcher.fetch_next_position()
            stamp = time.perf_counter()
            csv.write(f"{stamp},{gyro[0]},{gyro[1]},{gyro[2]},{accel[0]},{accel[1]},{accel[2]},{magneto[0]},{magneto[1]},{magneto[2]},{roll},{pitch},{yaw},{position.x},{position.y},{angle}\n")

def integrate_gyro(service, device):
    heading = 0
    last = time.perf_counter()
    while is_collecting_data:
        now = time.perf_counter()
        dt = now - last
        last = now
        gyro, _, _ = service.imu.get_reading()
        heading += -gyro[2] * dt
        device.set_quaternion(0, 0, 0, heading)
        time.sleep(0.01)

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
        #device.set_heading_pid(1.0, 0.0, 0.0)
        device.set_heading_pid(5.0, 0.0, 0.0)

        # collection thread

        position_fetcher = PositionFetcher()
        service = ImuFusionService(
            frequency=100,
            use_madgwick=False,
            device=device,
            moving_average_window=10,
            calibration_samples=100,
            g_var_scale=0.1,
            a_var_scale=1,
            m_var_scale=5
        )
        #service = None
        collector_thread = threading.Thread(target=collect_data_to_csv, args=(service, position_fetcher, device))
        #integration_thread = threading.Thread(target=integrate_gyro, args=(service, device))
        service.start_daemon()
        #integration_thread.start()
        collector_thread.start()

        # device.__apply_kinematics_model__(0.0, 0.0, -0.041560657)
        # time.sleep(3)
        # device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.15, 0.))
        # device.set_speed_while(for_seconds(5), Speed(0.9, 0.))
        # device.set_speed_while(for_distance(10), Speed(0.5, 0.0))
        #result = device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.0, 0.4))
        #result.advance()
        # device.drive_straight(for_distance(0.5), constant(Speed.Medium))
        # device.drive_straight(while_false(lambda: is_button_clicked()), Speed.Medium)
        await to_task(device.drive_straight(for_seconds(20), Speed(0.4, 0.0)))

        global is_collecting_data
        is_collecting_data = False
        collector_thread.join()
        #integration_thread.join()
        # device.rotate(for_cw_rotation(90), Speed.Medium)

asyncio.run(main())