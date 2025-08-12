from libstp.datatypes import Axis, Direction
from libstp.motor import Motor
from libstp.servo import Servo
from libstp_helpers.api.hardware.light_sensors import AdvancedLightSensor


class Definitions:
    left_front_motor = Motor(1)
    right_front_motor = Motor(0)
    left_front_sensor = AdvancedLightSensor(2, calibration_factor=0.1)
    right_front_sensor = AdvancedLightSensor(1, calibration_factor=0.1)
    back_sensor = AdvancedLightSensor(5, calibration_factor=0.1)
    servo_bottles = Servo(0)
    axis: Axis = Axis.Z
    direction: Direction = Direction.Normal

    def __init__(self):
        pass