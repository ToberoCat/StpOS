from libstp.datatypes import Axis, Speed, while_false, for_seconds, for_distance, for_ccw_rotation, for_cw_rotation, \
    constant
from libstp.device.two_wheeled import TwoWheeledDevice
from libstp.motor import Motor
from libstp.sensor import is_button_clicked

left_motor = Motor(0)
right_motor = Motor(1, reverse_polarity=True)

with TwoWheeledDevice(
        Axis.Z,
        left_motor,
        right_motor
) as device:  # type: TwoWheeledDevice
    device.wheel_base = 0.1796
    device.ticks_per_revolution = 1571
    # calibrate_ticks_per_revolution(device, 2, 5)
    # calibrate_wheel_base(device, 5)
    device.set_linear_pid(1.0, 0.0, 0.0)

    device.set_angular_pid(1.0, 0.0, 0.0)
    device.set_heading_pid(10.0, 0.0, 0.0)
    #device.set_speed_while(while_false(lambda: is_button_clicked()), Speed(0.5, 0.))
    device.set_speed_while(for_distance(10), Speed(0.5, 0.))
    #device.set_speed_while(for_seconds(5), Speed(0.9, 0.))
    #device.set_speed_while(for_distance(10), Speed(0.5, 0.0))
    #device.set_speed_while(for_ccw_rotation(90), Speed(0.0, 0.4))
    #device.drive_straight(while_false(lambda: is_button_clicked()), constant(Speed.Medium))

