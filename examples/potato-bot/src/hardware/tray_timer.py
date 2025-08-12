import time

from libstp.logging import info


class TrayTimer:
    def __init__(self):
        self.timer = 0
        self.start_time = None

    def pause_timer(self):
        if self.start_time is not None:
            self.timer += time.time() - self.start_time

        self.start_time = None
        info(f"[TrayTimer] Timer paused: {self.timer:.2f} seconds")

    def resume_timer(self):
        if self.start_time is not None:
            raise Exception("Timer already running")

        self.start_time = time.time()
        info(f"[TrayTimer] Timer resumed: {self.timer:.2f} seconds")

    def get_timer_progress(self):
        timer = self.timer
        if self.start_time is not None:
            timer += time.time() - self.start_time

        return timer