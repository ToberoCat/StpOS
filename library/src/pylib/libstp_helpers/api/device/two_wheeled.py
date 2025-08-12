from libstp.datatypes import Axis, Direction
from libstp.device.two_wheeled import TwoWheeledNativeDevice
from libstp.motor import Motor


class TwoWheeledDevice(TwoWheeledNativeDevice):
    def __init__(self,
                 axis: Axis,
                 direction: Direction,
                 left_front_motor: Motor,
                 right_front_motor: Motor):
        super().__init__(axis, direction, left_front_motor, right_front_motor)
