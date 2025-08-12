from libstp.datatypes import Axis, Direction
from libstp.motor import Motor
from libstp.sensor import LightSensor, AnalogSensor, DistanceSensor
from libstp.servo import Servo
from libstp_helpers.api.hardware.light_sensors import AdvancedLightSensor
from libstp_helpers.synchronizer import Synchroniser

from src.hardware.bottle_sensor import BottleSensor
from src.hardware.slider import Slider
from src.hardware.tray_timer import TrayTimer


class Defs:
    left_front_motor = Motor(1)
    right_front_motor = Motor(0)
    potato_motor = Motor(3)
    Flaschen_motor = Motor(2)

    servo = Servo(0)
    l_sensor = AdvancedLightSensor(1)
    r_sensor = AdvancedLightSensor(0)
    back_sensor = AdvancedLightSensor(3)

    wait_for_light_sensor = LightSensor(5)
    potato_sensor = AdvancedLightSensor(2)

    axis: Axis = Axis.Z
    direction: Direction = Direction.Normal

    arm_servo_unten = Servo(3)
    flaschen_servo = Servo(2)
    arm_servo_oben = Servo(1)
    arm_mini_servo = Servo(0)

    box_Motor = Motor(3)
    synchronizer = Synchroniser()

    tray_timer = TrayTimer()
    bottle_sensor = BottleSensor(DistanceSensor(4))

    def __init__(self):
        pass
