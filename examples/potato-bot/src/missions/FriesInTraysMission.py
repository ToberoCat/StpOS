from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import drive_until_black, turn_cw, drive_backward, turn_ccw, drive_until_white
from libstp_helpers.api.steps.motor import motor, stop_motor
from libstp_helpers.api.steps.sequential import Sequential, seq

from src.hardware.defs import Defs
from src.missions.DriveToTraysMission import Tray_Abstand
from src.steps.motor_step import shake_until_fry_dropped
from src.steps.tray_timer_step import timeout_tray


class FriesInTraysMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            # ab hier: Hardcode
            timeout_tray( # ToDo: Wenn das erste verkackt, das zweite skippen
                shake_until_fry_dropped(Defs.potato_motor, "potato_sensor"),
                Defs.tray_timer,
                13.3
            ),

            stop_motor(Defs.box_Motor),
            turn_cw(8.5, 1),

            # Erstes Tray befüllt

            drive_backward(Tray_Abstand + 0.1, 1),
            timeout_tray(
                shake_until_fry_dropped(Defs.potato_motor, "potato_sensor"),
                Defs.tray_timer,
                13.3
            ),
            stop_motor(Defs.box_Motor),

            # Zweites Tray befüllt

            drive_until_black("r_sensor", -1, 0.1, False),
            drive_until_white("r_sensor", -0.5, 0.1, False),
            drive_backward(Tray_Abstand / 2-0.6, 1),
            timeout_tray(
                shake_until_fry_dropped(Defs.potato_motor, "potato_sensor"),
                Defs.tray_timer,
                13.3
            ),
            stop_motor(Defs.box_Motor),

            # Drittes Tray befüllt
            drive_backward(Tray_Abstand, 1),
            timeout_tray(
                shake_until_fry_dropped(Defs.potato_motor, "potato_sensor"),
                Defs.tray_timer,
                13.3
            ),
            stop_motor(Defs.box_Motor),

            # Viertes Tray befüllt
            drive_backward(Tray_Abstand, 1),
            timeout_tray(
                shake_until_fry_dropped(Defs.potato_motor, "potato_sensor"),
                Defs.tray_timer,
                13.3
            ),
            stop_motor(Defs.box_Motor),
            # Fünftes Tray befüllt

            turn_ccw(14, 1),
            drive_backward(1.2, 1),
            # drive_backward(1.5,1),
            # turn_cw(10,1),
            # backward_lineup_on_white("l_sensor", "r_sensor"),

        ])
