from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import drive_backward, turn_cw, drive_forward, turn_ccw, parallel
from libstp_helpers.api.steps.custom_step import custom_step
from libstp_helpers.api.steps.motion.lineup import backward_lineup_on_white, forward_lineup_on_white, \
    backward_lineup_on_black
from libstp_helpers.api.steps.motor import motor
from libstp_helpers.api.steps.sequential import Sequential, seq

from src.steps.servo_steps import flaschen_servo_fahrt, arm_servo_unten_fahrt


class DriveToPotato(Mission):
    def sequence(self) -> Sequential:
        return seq([
            custom_step(lambda _, defs: defs.synchronizer.start_recording()),
            # Fährt aus der starting box
            drive_backward(1.5, 1),
            drive_forward(0.2, 1),
            turn_ccw(65, 1),
            backward_lineup_on_white("l_sensor", "r_sensor", ki=0.1),
            drive_forward(0.1, 1),
            turn_cw(90, 1),
            parallel(flaschen_servo_fahrt(),arm_servo_unten_fahrt(),motor("Flaschen_motor", -55, 1),),

            # Fährt an der Wand
            drive_backward(2.5, 1),
            turn_cw(5, 1),
            drive_backward(5, 1, False),

            turn_ccw(18, 1),
            drive_backward(2.2, 1),


            turn_cw(7, 1),
            drive_backward(2.5, 1),
            forward_lineup_on_white("l_sensor", "r_sensor",0.6),
            drive_forward(0.4, 1),
            turn_ccw(90, 1),

        ])#LOL LOL uincenccamucmulmdfdaewhjrcoceOM
