from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import seq, drive_backward, turn_cw, drive_forward, turn_ccw, \
    follow_line, wait_for_checkpoint, parallel

from libstp_helpers.api.steps.motion.line_follow import follow_line_until_both_black
from libstp_helpers.api.steps.motor import motor, stop_motor
from libstp_helpers.api.steps.sequential import Sequential


from src.hardware.defs import Defs
from src.steps.motor_step import  prep_for_drop

Tray_Abstand = 1  # Zeit in Sekunden die der Roboter bei voller Geschwindigkeit benötigt um von einem Tray zum anderen zu fahren
Blackline_FirstTray = 2.1


# Werte wurde durch experimentieren ermittelt

class DriveToTraysMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            drive_backward(0.7, 1),
            parallel(drive_backward(2, 1),
                     motor("box_Motor", -30, 1),),


            # Box nach oben Bewegen

            turn_cw(90, 1),
            # steht an der hintesten Pipe

            ##### parallel
            #shake_motor_to_loosen_fries(),
            prep_for_drop(Defs.box_Motor, Defs.potato_sensor),
            motor(Defs.box_Motor, -75, 0.3),
            stop_motor(Defs.box_Motor),
            ####
            wait_for_checkpoint(51),

            follow_line(1.5, 1, "l_sensor", "r_sensor"),
            follow_line_until_both_black(1, "l_sensor", "r_sensor"),
            turn_ccw(90, 1),
            drive_forward(1.4, 1),
            drive_backward(0.2, 1),
            turn_cw(87, 1),

            # drive_forward(3.2, 1),
            # turn_ccw(5.5, 1),

            # Fährt gegen die Pipe bis zur Schwarzen linie
            # drive_until_black("r_sensor", 1, 0.1),
            # drive_backward(0.4,1),
            # turn_cw(8, 1),
            # forward_lineup_on_black("l_sensor", "r_sensor"),

            # ab hier: Hardcode
            # turn_ccw(3,1),
            drive_forward(Blackline_FirstTray, 1),
            # steht am vordersten Tray

        ])
