import time

from libstp.datatypes import for_seconds, for_ticks, while_true, ConditionalResult
from libstp.motor import Motor

left_motor = Motor(0)
right_motor = Motor(1, reverse_polarity=True)

def test_stop():
    left_motor.stop()
    right_motor.stop()

def test_set_velocity():
    left_motor.set_velocity(500)
    right_motor.set_velocity(500)
    time.sleep(2)

def test_move_while():
    left_motor.move_while(for_seconds(3), 500)

    right_motor.reset_position_estimate()
    right_motor.move_while(for_ticks(500), 100)

test_set_velocity()
test_stop()
#test_move_while()
