import time

from libstp.backend import WombatRobotBackend
from libstp.sensor import calibrate_light_sensors, LightSensor, WombatLightSensor

robot = WombatRobotBackend()
left_sensor: LightSensor = robot.create_light_sensor(0)
right_sensor: LightSensor = robot.create_light_sensor(1)
light_sensor: LightSensor = WombatLightSensor(5)

calibrate_light_sensors([left_sensor, right_sensor])
light_sensor.wait_for_light()

while True:
    left_value = left_sensor.is_on_black()
    right_value = right_sensor.is_on_black()

    print(f'Left: {left_value}, Right: {right_value}')
    time.sleep(1)
