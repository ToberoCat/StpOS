import asyncio

from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps.sequential import Sequential, seq
from src.pylib.libstp_helpers.api.steps.drive import strafe_left, turn_cw, drive_forward, turn_ccw
from src.pylib.libstp_helpers.api.steps.parallel import parallel
from src.pylib.libstp_helpers.api.steps.timeout import timeout
from src.pylib.libstp_helpers.api.steps.wait_for_checkpoint import wait_for_checkpoint
from src.pylib.libstp_helpers.api.steps.wait_for_seconds import wait


class GrabCupsMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            drive_forward(1, 0.3), # drive out of start box
            wait(5), # wait until other guy past me
            strafe_left(5, 0.3),
            wait_for_checkpoint(10),
            timeout(
                turn_cw(90, 0.3),
                5
            ),
            parallel([
                drive_forward(1, 0.3),
                turn_ccw(90, 0.3),  
            ], [
                drive_forward(1, 0.3),
                turn_ccw(90, 0.3),
            ])
        ])


if __name__ == "__main__":
    mission = GrabCupsMission()
    asyncio.run(mission.run(device=None, definitions=None))
