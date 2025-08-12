from libstp.device.create3 import Create3Device
from libstp.datatypes import for_distance, for_ccw_rotation, constant

with Create3Device() as create3:
    create3.drive_straight(for_distance(25), constant(Speed.Medium))