from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import *
from libstp_helpers.api.steps.motor import motor
from libstp_helpers.api.steps.sequential import Sequential

from src.steps.crazy_align import align_on_pipe
from src.steps.motor_step import shake_if_no_fry
from src.steps.servo_steps import arm_servo_unten_potatoe_grab, arm_servo_oben_potaoe_grab, \
    arm_mini_servo_potatoe_offen, arm_mini_servo_potatoe_zu, arm_servo_oben_potatoe_frie, arm_servo_unten_potatoe_frie


class PotatoMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            parallel(drive_backward(2.5, 1),
                     arm_servo_unten_potatoe_grab(),
                     arm_servo_oben_potaoe_grab(),
                     arm_mini_servo_potatoe_offen(), ),
            timeout(
                align_on_pipe(),
                2.5
            ),
            drive_forward(1.62, 0.7),
            arm_mini_servo_potatoe_zu(),
            parallel(
                arm_servo_oben_potatoe_frie(),
                arm_servo_unten_potatoe_frie(),
                follow_line(3.8, 1, "l_sensor", "r_sensor")
            ),
            motor("box_Motor", -12, 0.3, ),

            arm_mini_servo_potatoe_offen(),
            wait(2),
            shake_if_no_fry(),
            # motor("box_Motor", 70, 0.35,),
            # motor("box_Motor", -70, 0.35,),
            motor("box_Motor", 30, 0.5, ),
        ])
