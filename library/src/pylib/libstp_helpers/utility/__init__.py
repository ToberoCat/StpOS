import asyncio
import multiprocessing
import os
import subprocess
import threading
import time
from queue import Queue
from typing import Dict, Union, Callable
import inspect

from libstp.logging import warn, info, error, debug
from libstp_helpers import get_bool_argument

properties_dir = f"{os.path.dirname(__file__)}/properties"

calibrated_velocity = 14.3 # cm/s, used for the robot's calibrated velocity

def seconds(cm: float):
    """
    Convert centimeters to seconds based on the calibrated velocity.
    :param cm: Distance in centimeters.
    :return: Time in seconds.
    """
    return cm / calibrated_velocity

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, text=True, capture_output=True)
        return result.stdout, result.stderr, result.returncode
    except Exception as e:
        return str(e), None, -1


def delete_properties(properties_file: str):
    file = f"{properties_dir}/{properties_file}.properties"
    if os.path.exists(file):
        os.remove(file)
    return True


def get_properties(properties_file: str) -> Union[Dict[str, str], None]:
    file = f"{properties_dir}/{properties_file}.properties"
    if not os.path.exists(file):
        return None

    properties = {}
    with open(file) as f:
        for line in f:
            if line.startswith("#"):
                continue
            key, value = line.strip().split("=")
            properties[key] = value
    return properties


def set_properties(properties_file: str, properties: Dict[str, str]):
    if not os.path.exists(properties_dir):
        os.mkdir(properties_dir)

    with open(f"{properties_dir}/{properties_file}.properties", "w") as f:
        for key, value in properties.items():
            f.write(f"{key}={value}\n")
    return True


def print_timestamp(msg: str = ""):
    frame = inspect.currentframe()
    caller_frame = frame.f_back
    caller_function_name = caller_frame.f_code.co_name
    formatted_caller_function_name = caller_function_name.replace("_", " ").title().strip()
    info(f"{formatted_caller_function_name} reached. {msg}".strip())


# ToDo: Natively implement this in the libstp library
async def to_task(algorithm, frequency=100):
    period = 1 / frequency
    while True:
        start_time = time.time()
        more = algorithm.advance()
        if not more:
            break
        elapsed_time = time.time() - start_time
        sleep_time = max(0.0, period - elapsed_time)
        await asyncio.sleep(sleep_time)  # let other tasks run
    # async def execute():
    #     process = multiprocessing.Process(target=block)
    #     process.start()
    #     while process.is_alive():
    #         await asyncio.sleep(0)
    #     process.join()
    # return asyncio.create_task(execute())


# ToDo: Natively implement this in the libstp library
class KillTimer:
    def __init__(self, timeout: float, code_block: Callable[[], None], alternative_code_block: Callable[[], None]):
        self.result_queue = Queue()

        proc = multiprocessing.Process(target=self.run_code_block, args=(code_block,))
        proc.daemon = True
        proc.start()
        proc.join(timeout)

        if not proc.is_alive():
            return

        warn("Code block execution timeout. Terminating.")
        proc.terminate()
        alternative_code_block()

    def run_code_block(self, code_block):
        result = code_block()
        self.result_queue.put(result)

    def get_result(self):
        """
        Get the result from the code block execution.
        This method should be called after the code block execution.
        """
        if not self.result_queue.empty():
            return self.result_queue.get()
        else:
            return None


class TournamentMode:
    def __init__(self, eth0: bool = False, wlan0: bool = True):
        self.encapsulated = False
        self.eth0 = eth0
        self.wlan0 = wlan0

    def encapsulate(self):
        if get_bool_argument("no-tournament"):
            warn("Tournament mode is disabled. This should only be used for development.")
            return

        if self.encapsulated:
            warn("Device is already encapsulated. Skipping encapsulation.")
            return

        debug("Encapsulating device. If the connection is lost, it's most likely due to the encapsulation.")
        if self.eth0:
            self.__shutdown_interface__("eth0")
        if self.wlan0:
            self.__shutdown_interface__("wlan0")

        self.encapsulated = True

    def decapsulate(self):
        if get_bool_argument("no-tournament"):
            warn("Tournament mode is disabled. This should only be used for development.")
            return

        if not self.encapsulated:
            warn("Device is not encapsulated. Skipping decapsulation.")
            return

        if self.eth0:
            self.__start_interface__("eth0")
        if self.wlan0:
            self.__start_interface__("wlan0")

    def __start_interface__(self, interface: str):
        try:
            subprocess.run(['sudo', 'ifconfig', interface, 'up'], check=True)
            info(f"{interface} interface is up.")
        except subprocess.CalledProcessError as e:
            error(f"Failed starting {interface}: {e}")

    def __shutdown_interface__(self, interface: str):
        try:
            subprocess.run(['sudo', 'ifconfig', interface, 'down'], check=True)
            info(f"{interface} interface is down.")
        except subprocess.CalledProcessError as e:
            error(f"Failed stopping {interface}: {e}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.decapsulate()

    def __enter__(self):
        self.encapsulate()
