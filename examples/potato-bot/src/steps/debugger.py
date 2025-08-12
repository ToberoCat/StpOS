from typing import Any

from libstp_helpers.api.steps import Step


class BreakpointStep(Step):
    async def run_step(self, device, definitions: Any) -> None:
        self.debug("Breakpoint passed")

def debug_breakpoint():
    return BreakpointStep()