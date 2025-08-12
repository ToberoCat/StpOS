import math
import time
from datetime import timedelta

from libstp.servo import Servo
from libstp.logging import error, info, debug

from libstp_helpers.utility.math import ease_in_ease_out


def calculate_unfolded_position(table_cm: float) -> int:
    create3_length = 67.86
    x = table_cm - create3_length
    debug(f"Table length: {table_cm}")
    if x <= 0:
        error(f"The table length is less than the create3 length!")
        return 1700
    calculated_position = int(math.tan((x - 22.5) / 10.3) * 450 + 1023)
    info(f"The folding servo will unfold to {calculated_position}")
    if calculated_position < 1500 or calculated_position > 2000:
        error(f"The calculated position is very likely to suck ass! This table is SHIT")

    return min(max(calculated_position, 1500), 2000)


class FoldingServo(Servo):
    def __init__(self, port: int, folded_position: int = 100,
                 table_length: float = 100.5, drop_position: int = 1550, backup_amount: int = 70, duration: int = 0.8):
        super().__init__(port)
        self.servo_offset = 0
        self.folded_position = folded_position
        self.unfolded_position = calculate_unfolded_position(table_length)
        self.drop_position = drop_position
        self.backup_amount = backup_amount
        self.duration = timedelta(seconds=duration)
        self.enable()

    def fold(self):
        self.slowly_set_position(self.folded_position + self.servo_offset, timedelta(seconds=1), ease_in_ease_out)

    def mini_fold(self):
        self.slowly_set_position(self.folded_position + self.servo_offset + 400, timedelta(seconds=0.2), ease_in_ease_out)
    def semi_fold(self):
        self.slowly_set_position(self.unfolded_position - self.backup_amount + self.servo_offset, self.duration,
                                      ease_in_ease_out)

    def unfold(self):
        self.slowly_set_position(self.unfolded_position + self.servo_offset, self.duration, ease_in_ease_out)

    def unfold_more(self):
        self.slowly_set_position(self.unfolded_position + self.servo_offset + 80, self.duration, ease_in_ease_out)

    def unfold_drop_one(self):
        self.slowly_set_position(self.drop_position + 50, timedelta(seconds=1.2), ease_in_ease_out)

    def unfold_drop_before(self):
        self.slowly_set_position(self.drop_position - 50, timedelta(seconds=0.2), ease_in_ease_out)

    def unfold_drop_before_fast(self):
        self.set_position(self.drop_position - 50)

    def shake_pool_noodles(self):
        start = time.time()
        self.shake_while_bounds(self.get_position() - 120, self.get_position(), timedelta(seconds=0.1),
                                     lambda: time.time() - start < 1)

    def shake_pool_noodles_forward(self):
        start = time.time()
        self.shake_while_bounds(self.get_position(), self.get_position() + 250, timedelta(seconds=0.1),
                                     lambda: time.time() - start < 1)


class PoolNoodleStopperServo(Servo):
    def __init__(self, port: int, open_position: int = 600, close_position: int = 1450, duration: int = 0.5):
        super().__init__(port)
        self.open_position = open_position
        self.close_position = close_position
        self.duration = timedelta(seconds=duration)
        self.enable()

    def open(self):
        self.enable()
        self.slowly_set_position(self.open_position, timedelta(seconds=0.2), ease_in_ease_out)

    def close(self):
        self.enable()
        self.slowly_set_position(self.close_position, self.duration, ease_in_ease_out)

    def semi_close(self):
        self.enable()
        self.slowly_set_position(1100, self.duration, ease_in_ease_out)


class PommesLifter(Servo):
    def __init__(self, port: int,
                 upper_position_small: int = 500,
                 upper_position_medium: int = 250,
                 setup_position: int = 500,
                 down_position: int = 130,
                 duration: int = 0.3):
        super().__init__(port)
        self.servo_offset = 0
        self.upper_position_small = upper_position_small + self.servo_offset
        self.upper_position_medium = upper_position_medium + self.servo_offset
        self.setup_position = setup_position + self.servo_offset
        self.down_position = down_position + self.servo_offset
        self.duration = timedelta(seconds=duration)

    def up_small(self):
        with self as servo:
            servo.slowly_set_position(self.upper_position_small, timedelta(seconds=1), ease_in_ease_out)

    def up_half_medium(self):
        with self as servo:
            servo.slowly_set_position(self.upper_position_medium, self.duration, ease_in_ease_out)

    def up_medium(self):
        with self as servo:
            servo.slowly_set_position(450, timedelta(seconds=1), ease_in_ease_out)

    def setup(self):
        with self as servo:
            servo.slowly_set_position(self.setup_position, self.duration, ease_in_ease_out)

    def setup_but_higher(self):
        with self as servo:
            servo.slowly_set_position(self.setup_position + 70, self.duration, ease_in_ease_out)

    def down_hovering_over_table(self):
        with self as servo:
            servo.slowly_set_position(self.down_position, self.duration, ease_in_ease_out)

    def down_touching_table(self):
        with self as servo:
            servo.slowly_set_position(50, self.duration, ease_in_ease_out)

    def down_for_middle(self):
        with self as servo:
            servo.slowly_set_position(300, self.duration, ease_in_ease_out)

    def down_for_small(self):
        with self as servo:
            servo.slowly_set_position(200, self.duration, ease_in_ease_out)

    def up_half_small(self):
        with self as servo:
            servo.slowly_set_position(120, self.duration, ease_in_ease_out)

    def pre_down(self):
        with self as servo:
            servo.slowly_set_position(self.down_position + 50, self.duration, ease_in_ease_out)


class PommesGatsch(Servo):
    def __init__(self, port: int, open_position: int = 1600, semi_open_position: int = 1000, close_position: int = 0, duration: int = 0.5):
        super().__init__(port)
        self.servo_offset = 0
        self.open_position = open_position + self.servo_offset
        self.close_position = close_position + self.servo_offset
        self.semiopen = semi_open_position + self.servo_offset
        self.duration = timedelta(seconds=duration)
        self.enable()

    def open(self):
        self.enable()
        self.slowly_set_position(self.open_position, self.duration, ease_in_ease_out)

    def open_faaast(self):
        self.enable()
        self.set_position(self.open_position)

    def semi_open(self):
        self.enable()
        self.set_position(self.semiopen)
        # self.slowly_set_position(self.semiopen, timedelta(seconds=0.2), ease_in_ease_out)

    def close(self):
        self.enable()
        self.slowly_set_position(self.close_position, self.duration, ease_in_ease_out)
