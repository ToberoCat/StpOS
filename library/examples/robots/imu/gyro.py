import time

from libstp.backend import WombatRobotBackend
from libstp.sensor import GyroXSensor, GyroYSensor, GyroZSensor, any_button_clicked
from libstp.filter import FirstOrderLowPassFilter
from libstp.sensor import wait_for_any_button_click

robot = WombatRobotBackend()

gyro_x = GyroXSensor(FirstOrderLowPassFilter(0.7))
gyro_y = GyroYSensor(FirstOrderLowPassFilter(0.7))
gyro_z = GyroZSensor(FirstOrderLowPassFilter(0.7))

wait_for_any_button_click()
sums = [0, 0, 0]
time.sleep(1)
while not any_button_clicked():
    x = gyro_x.get_value()
    y = gyro_y.get_value()
    z = gyro_z.get_value()
    sums[0] += x
    sums[1] += y
    sums[2] += z
    print(f'X: {x}, Y: {y}, Z: {z} : {sums}')
    time.sleep(0.1)