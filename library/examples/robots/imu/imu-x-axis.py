import time

from libstp.backend import WombatRobotBackend
from libstp.attitude_estimator import get_attitude_x, get_attitude_y, get_attitude_z
from libstp.filter import FirstOrderLowPassFilter
from libstp.sensor import GyroXSensor, AccelXSensor, calibrate_magnetometer, MagnetoXSensor

robot = WombatRobotBackend()
gyro_x = GyroXSensor(FirstOrderLowPassFilter(0.7))
accel_x = AccelXSensor(FirstOrderLowPassFilter(0.7))
magneto_x = MagnetoXSensor(FirstOrderLowPassFilter(0.7))

#calibrate_magnetometer()

while True:
    print(f'Gyro X: {gyro_x.get_value()}, Accel X: {accel_x.get_value()}, Magneto X: {magneto_x.get_value()}')
    time.sleep(1)
    # x = get_attitude_x()
    # y = get_attitude_y()
    # z = get_attitude_z()
    #
    # print(f'X: {x}, Y: {y}, Z: {z}')
    # time.sleep(1)
