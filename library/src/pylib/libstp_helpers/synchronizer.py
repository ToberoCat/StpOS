import asyncio

from libstp.logging import info, warn


class Synchroniser:
    def __init__(self):
        self.start_time = None

    def start_recording(self):
        self.start_time = asyncio.get_event_loop().time()

    def get_time(self):
        return asyncio.get_event_loop().time() - self.start_time

    async def wait_until_checkpoint(self, checkpoint_seconds):
        delta = checkpoint_seconds - self.get_time()
        if delta < 0:
            warn(f"[Synchroniser] Scheduler has passed the checkpoint at {checkpoint_seconds} seconds, delta: {delta}")
            return

        info(f"[Synchroniser] Scheduler waiting until {checkpoint_seconds} seconds, delta: {delta}")
        await asyncio.sleep(delta)
        info(f"[Synchroniser] Scheduler has reached the checkpoint at {checkpoint_seconds} seconds")

    async def do_until_checkpoint(self, checkpoint_seconds, func, *args, **kwargs):
        """
        Execute a function until the specified checkpoint is reached.

        Args:
            checkpoint_seconds: The time in seconds to wait until the checkpoint.
            func: The function to execute.
            *args: Positional arguments for the function.
            **kwargs: Keyword arguments for the function.
        """
        delta = checkpoint_seconds - self.get_time()
        if delta < 0:
            warn(f"[Synchroniser] Scheduler has passed the checkpoint at {checkpoint_seconds} seconds, delta: {delta}")
            return

        info(f"[Synchroniser] Scheduler executing until {checkpoint_seconds} seconds, delta: {delta}")
        task = asyncio.create_task(func(*args, **kwargs))
        await asyncio.sleep(delta)

        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        info(f"[Synchroniser] Scheduler has reached the checkpoint at {checkpoint_seconds} seconds")