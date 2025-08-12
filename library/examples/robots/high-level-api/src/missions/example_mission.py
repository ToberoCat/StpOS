from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import *
from libstp_helpers.api.steps.motion.drive_until import drive_until_black
from libstp_helpers.api.steps.sequential import Sequential



class ExampleMission(Mission):

    def sequence(self) -> Sequential:
        return seq([
            forward_lineup_on_black(reverse=True),
            # follow_line_single(5, 0.7, "left_front_sensor"),
            # follow_line(5, 0.7),
            # parallel(
            #     wait(5),
            #     drive_forward(1, 1)
            # )
            # drive_until_black("left_front_sensor", 1),
            # drive_forward(1, 1),
            # drive_backward(1, 1),
            # turn_cw(90, 1),
            # turn_ccw(90, 1),
            # drive_until_white("left_front_sensor", 1),
        ])
