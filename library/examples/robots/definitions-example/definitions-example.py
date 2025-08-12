import time

from libstp.backend import RobotBackend, WombatRobotBackend
from libstp.sensor import any_button_clicked

from libstp_helpers.defintions import define_definition

robot = WombatRobotBackend()


class Definitions:
    def __init__(self, robot: RobotBackend):
        self.robot = robot
        self.left_light_sensor = self.robot.create_light_sensor(0)
        self.right_light_sensor = self.robot.create_light_sensor(1)
        self.wait_for_light_sensor = self.robot.create_light_sensor(5)


if __name__ == "__main__":
    definition = Definitions(robot)
    definitions: Definitions = define_definition(__file__, definition)
    print(definitions.left_light_sensor)
    while not any_button_clicked():
        print("Click any button to exit")
        time.sleep(1)