import ctypes
import time

from libstp.sensor import is_button_clicked

lib = ctypes.CDLL('/usr/lib/libkipr.so')
#lib.calibrate_compass()
lib.set_compass_params.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
lib.set_compass_params(-6.845924, -7.466255, 0.521250, 0.418732, 0.163766, 1.084129, 1.188670)

lib.get_compass_angle.restype = ctypes.c_float
while not is_button_clicked():
    print(lib.get_compass_angle())
    time.sleep(0.1)