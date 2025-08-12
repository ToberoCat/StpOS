import inspect
import time
from libstp.logging import debug

_start_time: float = None


def start_recording():
    global _start_time
    _start_time = time.time()
    debug("Recording started.")


def print_timestamp():
    frame = inspect.currentframe()
    caller_frame = frame.f_back
    caller_function_name = caller_frame.f_code.co_name
    formatted_caller_function_name = caller_function_name.replace("_", " ").title().strip()
    print_with_timestamp(f"{formatted_caller_function_name} reached after " + "{}s")


def print_with_timestamp(message: str):
    elapsed = -1.0
    if _start_time is not None:
        elapsed = time.time() - _start_time
    debug(message.format(elapsed).strip())


def wait_for_total_seconds_passed(seconds: float):
    if _start_time is None:
        raise ValueError("Recording not started.")
    elapsed = time.time() - _start_time
    while elapsed < seconds:
        time.sleep(0.01)
        elapsed = time.time() - _start_time
