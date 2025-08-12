import time

import numpy as np
from libstp.sensor import is_button_clicked

from orientation.magneto import Magneto


def apply_calibration(raw):
    hardIronOffset = np.array([-4.2320698237489109, 13.74832191248862, 3.8093311149317524])
    softIronMatrix = np.array([[1.2472940502452017, -0.15902122146951733, 0.031265583620391411],
                               [-0.15902122146951722, 1.1999243231909518, 0.074195863233602541],
                               [0.031265583620391439, 0.074195863233602541, 0.75815969268571803]])
    m_corrected = raw - hardIronOffset
    m_calibrated = np.dot(m_corrected, softIronMatrix)
    return m_calibrated


magneto = Magneto()
while not is_button_clicked():
    raw = magneto.get_reading()
    calibrated = apply_calibration(raw)
    print(f"Calibrated: {calibrated} - Magnitude: {np.linalg.norm(calibrated)}")
    time.sleep(1)
