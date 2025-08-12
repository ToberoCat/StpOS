#
# This class should be used to create an indefinitely running program that keeps the connected between the Create3
# and the wombat alive.
#
import time

from libstp.backend import Create3Backend
from libstp.logging import info
from libstp.sensor import Create3LightSensor

create3_backend = Create3Backend()

green_hex = 0x00FF00

light = Create3LightSensor(0)
while True:
    light_value = light.get_value()
    # create3_backend.blink_light_ring(
    #     green_hex,
    #     green_hex,
    #     green_hex,
    #     green_hex,
    #     green_hex,
    #     green_hex,
    #     1
    # )
    info("Keep alive signal sent")
    time.sleep(0.5)
