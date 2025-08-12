from libstp_helpers import TableSide
from fullexamplecreate.definitions import Definitions

__apply_for_side__ = TableSide.TABLE_1A


def __apply__(definitions: Definitions) -> Definitions:
    definitions.left_outer_light_sensor = Create3LightSensor(0, calibration_factor=0.95)
    return definitions
