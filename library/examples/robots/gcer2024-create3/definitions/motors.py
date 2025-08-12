import time

from libstp.logging import info
from libstp.motor import ServoLikeMotor
from libstp.sensor import DigitalSensor, WombatLightSensor


class LightSwitchFlipperMotor(ServoLikeMotor):
    def __init__(self, port: int,
                 up_click_sensor: DigitalSensor,
                 down_click_sensor: DigitalSensor):
        super().__init__(port)
        self.up_click_sensor = up_click_sensor
        self.down_click_sensor = down_click_sensor
        self.reset_position_estimate()

    def up_by(self, seconds: float):
        self.set_velocity(1000)
        time.sleep(seconds)
        self.set_velocity(0)

    def put_up(self, velocity=250):
        while not self.up_click_sensor.is_clicked():
            self.set_velocity(velocity)
        self.stop()
        self.reset_position_estimate()

    def put_down(self, velocity=-500):
        while not self.down_click_sensor.is_clicked():
            self.set_velocity(velocity)
        self.stop()

    def put_down_time(self):
        self.set_velocity(-500)
        time.sleep(0.1)
        self.set_velocity(-500)
        time.sleep(0.3)
        self.set_velocity(-500)
        time.sleep(0.6)
        self.stop()

    def is_down(self):
        return self.down_click_sensor.is_clicked()

    def is_up(self):
        return self.up_click_sensor.is_clicked()

    def calibrate(self):
        info("Calibrating light switch flipper motor.")
        self.put_up()
        self.reset_position_estimate()
