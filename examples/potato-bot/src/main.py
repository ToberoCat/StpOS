from libstp_helpers.api.device.two_wheeled import TwoWheeledDevice
from libstp_helpers.api.robot import Robot
from libstp_helpers.api.steps import follow_line_single

from src.hardware.defs import Defs
from src.missions.BottlesMission import BottlesMission
from src.missions.DriveToPotatoMission import DriveToPotato
from src.missions.DriveToTraysMission import DriveToTraysMission
from src.missions.FriesInTraysMission import FriesInTraysMission
from src.missions.PotatoMission import PotatoMission
from src.missions.SetupMission import SetupMission
from src.missions.ShutdownMission import ShutdownMission
from src.missions.UPDownTest import Shake

robot = Robot(TwoWheeledDevice, Defs)

robot.use_missions(
     DriveToPotato(),
      PotatoMission(),
      DriveToTraysMission(),
      FriesInTraysMission(),
      BottlesMission()
     #Shake()
)

robot.set_setup_mission(SetupMission())
robot.set_shutdown_mission(ShutdownMission())
robot.set_light_sensor(Defs.wait_for_light_sensor)
robot.set_auto_shutdown(seconds=118)

if __name__ == "__main__":
    robot.start()
