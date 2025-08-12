import datetime
import time

from libstp.backend import Create3Backend, RobotBackend
from libstp.logging import info
from libstp.scheduler import shut_down_in
from libstp_helpers import Module, run_as_module
from libstp_helpers.defintions import define_definition
from libstp_helpers.utility import TournamentMode
from libstp_helpers.utility.timings import start_recording, print_with_timestamp, print_timestamp

from fullexamplecreate.definitions import Definitions
from fullexamplecreate.modules.Pommes import Pommes
from fullexamplecreate.modules.drive_to_grab_position import DriveToPlaceAstronautPosition
from fullexamplecreate.modules.drop_purple_noodles import DropPurpleNoodles
from fullexamplecreate.modules.get_5_habitats import Get5Habitats
from fullexamplecreate.modules.grab_purple_noodles import GrabPurpleNoodles
from fullexamplecreate.modules.klatschiklatschi import klatsch_aufeinmal
from fullexamplecreate.modules.place_astronaut import PlaceAstronaut
from fullexamplecreate.setup import SetupModule


class MainModule(Module):
    def __init__(self, robot: RobotBackend, definitions: Definitions):
        self.robot = robot
        self.definitions = definitions
        super().__init__("main")

    def run(self):
        start_recording()
        run_as_module("shutdown", lambda: shut_down_in(datetime.timedelta(seconds=119)))

        Get5Habitats(self.robot, self.definitions)
        print_with_timestamp("Habitats finished {}")

        DriveToPlaceAstronautPosition(self.robot, self.definitions)

        PlaceAstronaut(self.robot, self.definitions)
        print_with_timestamp("astronaut finished {}")

        GrabPurpleNoodles(self.robot, self.definitions)
        print_with_timestamp("pool noodles grabbed {}")

        drop_purple_noodles = DropPurpleNoodles(self.robot, self.definitions)
        drop_purple_noodles.get_to_corner_of_nooodles()
        print_with_timestamp("in se corner {}")
        pommes = Pommes(self.robot, self.definitions)
        # pommes.fettiges_pommsis()
        print_with_timestamp("pommes finished {}")
        klatsch_aufeinmal(self.robot, self.definitions).fettige_pommsis()
        # drop_purple_noodles.noodle_drop()
        print_with_timestamp("pool noodles dropped {}")
        pommes.get_to_lets_chat()

        pommes.pommes_lets_chat()
        print_with_timestamp("pommes finished {}")


def main():
    with TournamentMode(eth0=False, wlan0=True) as tournament:
        with Create3Backend() as robot:
            definitions = define_definition(__file__, Definitions(robot))

            SetupModule(robot, definitions)
            MainModule(robot, definitions)
            print_with_timestamp("Finished {}")


if __name__ == "__main__":
    main()
