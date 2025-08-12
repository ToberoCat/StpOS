import time

from libstp.datatypes import Axis, Direction
from libstp.device.omni_wheeled import OmniWheeledDevice
from libstp.motor import Motor
from libstp.sensor import is_button_clicked, GyroZSensor, MagnetoXSensor, GyroYSensor

front_right_motor = Motor(0)
front_left_motor = Motor(1)
rear_right_motor = Motor(2)
rear_left_motor = Motor(3)

with OmniWheeledDevice(
        Axis.Z,
        Direction.Normal,
        front_left_motor,
        front_right_motor,
        rear_left_motor,
        rear_right_motor
) as device:  # type: OmniWheeledDevice
    gyro_z = GyroYSensor()
    heading = 0

    last = time.time()
    while not is_button_clicked():
        z = gyro_z.get_value()
        heading += z * (time.time() - last)
        last = time.time()

        print(f"Raw: {z}, Heading: {heading}")
        time.sleep(0.1)