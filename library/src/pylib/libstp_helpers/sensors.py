from libstp.logging import debug, info
from libstp.sensor import Sensor, calibrate_light_sensors
from libstp_helpers import get_bool_argument
from libstp_helpers.utility import get_properties, set_properties, delete_properties
from typing import List


def delete_calibrate_light_sensors_properties():
    delete_properties("calibration")


def lazy_calibrate_light_sensors(sensors: List[Sensor]):
    if get_bool_argument("calibrate", True):
        debug("Forcing calibration because no-calibrate argument was not passed.")
        delete_calibrate_light_sensors_properties()

    calibration_values = get_properties("calibration") or {}
    sensors_to_calibrate = []

    # Check which sensors need calibration
    for sensor in sensors:
        sensor_key = f"{sensor.__class__.__name__}.{sensor.get_port()}"
        required_keys = [
            f"{sensor_key}.white",
            f"{sensor_key}.white-mean",
            f"{sensor_key}.white-std",
            f"{sensor_key}.black",
            f"{sensor_key}.black-mean",
            f"{sensor_key}.black-std"
        ]

        if any(key not in calibration_values for key in required_keys):
            debug(f"Sensor on port {sensor.get_port()} needs calibration - missing values")
            sensors_to_calibrate.append(sensor)
        else:
            # Set calibration values for sensors with existing data
            sensor.white_threshold = int(calibration_values[f"{sensor_key}.white"])
            sensor.white_mean = float(calibration_values[f"{sensor_key}.white-mean"])
            sensor.white_std_dev = float(calibration_values[f"{sensor_key}.white-std"])

            sensor.black_threshold = int(calibration_values[f"{sensor_key}.black"])
            sensor.black_mean = float(calibration_values[f"{sensor_key}.black-mean"])
            sensor.black_std_dev = float(calibration_values[f"{sensor_key}.black-std"])

            debug(f"Using existing calibration for sensor on port {sensor.get_port()}: "
                  f"white: {sensor.white_threshold}, black: {sensor.black_threshold}")

    # Only calibrate sensors that need it
    if sensors_to_calibrate:
        info(f"Calibrating {len(sensors_to_calibrate)} sensor(s) with missing calibration data")
        calibrate_light_sensors(sensors_to_calibrate)

        # Update properties with new calibration values
        for sensor in sensors_to_calibrate:
            sensor_key = f"{sensor.__class__.__name__}.{sensor.get_port()}"
            calibration_values[f"{sensor_key}.white"] = str(sensor.white_threshold)
            calibration_values[f"{sensor_key}.white-mean"] = str(sensor.white_mean)
            calibration_values[f"{sensor_key}.white-std"] = str(sensor.white_std_dev)

            calibration_values[f"{sensor_key}.black"] = str(sensor.black_threshold)
            calibration_values[f"{sensor_key}.black-mean"] = str(sensor.black_mean)
            calibration_values[f"{sensor_key}.black-std"] = str(sensor.black_std_dev)

            debug(f"Set calibration for sensor on port {sensor.get_port()}: "
                  f"white: {sensor.white_threshold}, black: {sensor.black_threshold}")

        set_properties("calibration", calibration_values)
    else:
        info("All sensors have calibration data - no calibration needed")
