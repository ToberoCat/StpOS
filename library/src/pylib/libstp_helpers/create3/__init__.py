import datetime
import os
import sys
import time
from abc import abstractmethod
from typing import Callable

import requests
from libstp.backend import Create3Backend
from libstp.logging import error, warn, info
from libstp.scheduler import shut_down_in
from libstp.sensor import wait_for_any_button_click, LightSensor

from libstp_helpers import Module, get_bool_argument, run_as_module
from libstp_helpers.utility import run_command


def get_create3_server_logs() -> str:
    return run_command(f"sudo podman logs create3_server")[0]


def restart_local_create3_server(force: bool = False) -> bool:
    if not force:
        warn("Are you sure you want to restart the create3 server? Press B to confirm.")
        wait_for_any_button_click()
    return run_command("sudo systemctl restart create3_server")


def reboot_create3(create3_ip: str = "192.168.186.2",
                   create3_port: int = 80,
                   timeout: int = 30) -> bool:
    url = f"http://{create3_ip}:{create3_port}/api/reboot"
    try:
        response = requests.post(url, timeout=timeout)
        response.raise_for_status()
        if response.status_code == 200:
            return True
        return False
    except requests.exceptions.RequestException as e:
        error(f"Failed to contact create3 server {e}")
        return False


def restart_create3_application_server(create3_ip: str = "192.168.186.2",
                                       create3_port: int = 80,
                                       timeout: int = 120) -> bool:
    url = f"http://{create3_ip}:{create3_port}/api/restart-app"
    try:
        response = requests.post(url, timeout=timeout)
        response.raise_for_status()
        if response.status_code == 200:
            return True
        return False
    except requests.exceptions.RequestException as e:
        error(f"Failed to contact create3 server {e}")
        return False


class Create3SetupModule(Module):
    def __init__(self, robot: Create3Backend, wait_for_light_sensors: LightSensor):
        self.parts = []
        self.robot = robot
        self.wait_for_light_sensors = wait_for_light_sensors
        super().__init__("create3_setup")

    def register_part(self, function: Callable[[Callable[[int], None]], None]):
        def wrapper(set_status):
            info(f"Running setup part {function.__name__}")
            function(set_status)
        self.parts.append(wrapper)

    @abstractmethod
    def register_parts(self):
        raise NotImplementedError("Method register_parts() not implemented")

    def __blink__(self, *ring_buffer):
        if get_bool_argument("no-light-ring"):
            return

        try:
            self.robot.blink_light_ring(*ring_buffer, 1)  # ToDo: Replace with actual light ring function
            self.robot.await_commands()
        except Exception as e:
            error(f"Failed to set light ring status {e}")

    def run(self):
        self.__blink__(
            0x555555,
            0x000000,
            0x555555,
            0x000000,
            0x555555,
            0x000000)  # ToDo: Replace with actual light ring function

        self.register_parts()
        ring_buffer = [
            0x000000,
            0x000000,
            0x000000,
            0x000000,
            0x000000,
            0x000000
        ]

        info("Click any button to start")
        wait_for_any_button_click()

        try:
            for part_idx in range(len(self.parts)):
                def set_status(status: int):
                    ring_buffer[part_idx] = status
                    self.__blink__(*ring_buffer)

                set_status(0x0000FF)
                self.parts[part_idx](set_status)
                set_status(0x00FF00)

            info("Calibration done. Click any button to wait for light")
            wait_for_any_button_click()
            run_as_module("wait-for-light", self.wait_for_light_sensors.wait_for_light)
        except Exception as e:
            warn(f"Failed with setup. Retrying. Press B to retry. Error: {e}")
            wait_for_any_button_click()
            os.execl(sys.executable, sys.executable, *sys.argv)
