from typing import List

from libstp import initialize_timer
from libstp.device import NativeDevice
from libstp_helpers.api.steps.sequential import Sequential

from libstp_helpers.api.missions import Mission
from libstp_helpers.api.steps import seq


# todo: Track mission lead time in csv file for later analysis of how much variance each mission has and how it changed over time
class MissionController:
    def __init__(self, device: NativeDevice, definitions):
        self.device = device
        self.definitions = definitions

    async def execute_missions(self, missions: List[Mission]):
        initialize_timer()
        sequence: Sequential = seq([mission.sequence() for mission in missions])
        await sequence.run_step(self.device, self.definitions)
        sequence.call_on_exit(None)