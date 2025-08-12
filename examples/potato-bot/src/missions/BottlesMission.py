from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import drive_backward, turn_cw, drive_until_black, follow_line, turn_ccw, drive_forward, \
    backward_lineup_on_black, drive_until_white, parallel, wait_for_checkpoint, wait
from libstp_helpers.api.steps.custom_step import custom_step
from libstp_helpers.api.steps.motion.lineup import forward_lineup_on_white
from libstp_helpers.api.steps.motor import motor
from libstp_helpers.api.steps.sequential import Sequential, seq
from libstp_helpers.api.steps.servo import shake_servo

from src.steps.bottle_sensor_step import drive_until_bottle_detected
from src.steps.servo_steps import flaschen_servo_grab, flaschen_servo_down, \
    flaschen_servo_fahrt, flaschen_servo_fahrt_midpoint, flaschen_servo_safe


class BottlesMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            # steht hinten an der Linie

            turn_ccw(82, 1),
            drive_backward(0.7, 1),
            backward_lineup_on_black("l_sensor", "r_sensor", 1),
            drive_forward(0.4, 1),
            turn_cw(90, 1),
            wait_for_checkpoint(86),
            follow_line(2, 1, "l_sensor", "r_sensor"),
            turn_cw(90, 1),
            parallel(drive_forward(2, 1, False),flaschen_servo_grab(),),

            drive_backward(0.2, 1),



            turn_ccw(90, 1),

            # auf Abstand fahren bis zur schwarzen Linie
            # forward_lineup_on_white("l_sensor", "r_sensor",1),
            parallel(forward_lineup_on_white("l_sensor", "r_sensor", 1),flaschen_servo_grab(),),

            turn_cw(5, 1),

            # auf Zeit bis zu den Flaschen nach vorne fahren
            custom_step(lambda device, _: device.set_max_speeds(0.8, 0.8, 0.8)),
            parallel(shake_servo("flaschen_servo", 3.3, 12, 16.5, ),drive_until_bottle_detected(0.7),),
            parallel(shake_servo("flaschen_servo", 0.15, 12, 16.5,),drive_forward(0.15, 0.7),),


            turn_ccw(6, 1),

            motor("Flaschen_motor", -60, 0.4),
            wait(0.2),
            parallel(motor("Flaschen_motor", 25, 1), flaschen_servo_fahrt()),

            custom_step(lambda device, _: device.set_max_speeds(9999, 9999, 999)),
            forward_lineup_on_white("l_sensor", "r_sensor", 1),

            turn_ccw(90, 1),
            #flaschen_servo_safe(), mit motor
            drive_until_black("l_sensor", 1),
            drive_forward(0.5, 1),


            forward_lineup_on_white("l_sensor", "r_sensor", 1),
            drive_backward(0.3, 1), #neuer Wert


            turn_ccw(89, 0.5),

            # custom_step(lambda device, _: device.set_max_speeds(0, 0, 0)),
            drive_forward(1.9, 1),
            parallel(flaschen_servo_down(),motor("Flaschen_motor", -73, 0.3)),
        ])
