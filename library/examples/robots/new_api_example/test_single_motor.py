import time

from libstp.datatypes import for_seconds, for_ticks
from libstp.motor import Motor
from libstp.sensor import wait_for_button_click


def test_stop(m):
    m.stop()
    time.sleep(0.2)
    previous = m.get_current_position_estimate()
    time.sleep(0.1)
    print(m.get_current_position_estimate(), previous)
    assert m.get_current_position_estimate() == previous
    time.sleep(1)


def test_set_velocity(m, direction):
    m.reset_position_estimate()
    print(m.get_current_position_estimate())
    assert m.get_current_position_estimate() == 0

    m.set_velocity(500 * direction)
    time.sleep(2)

    print(m.get_current_position_estimate())
    if direction == 1:
        assert m.get_current_position_estimate() > 950
    else:
        assert m.get_current_position_estimate() < -950


def test_move_while_seconds(m, direction):
    m.reset_position_estimate()
    print(m.get_current_position_estimate())
    assert m.get_current_position_estimate() == 0

    m.move_while(for_seconds(2), 500 * direction)

    if direction == 1:
        assert m.get_current_position_estimate() > 950
    else:
        assert m.get_current_position_estimate() < -950


def test_move_while_ticks_from_zero(m, direction):
    m.reset_position_estimate()
    print(m.get_current_position_estimate())
    assert m.get_current_position_estimate() == 0

    m.move_while(for_ticks(1000 * direction), 500 * direction)

    if direction == 1:
        assert m.get_current_position_estimate() > 950
    else:
        assert m.get_current_position_estimate() < -950

def test_move_while_ticks_from_non_zero(m, direction):
    offset = m.get_current_position_estimate()
    print("Starting from", offset)

    m.move_while(for_ticks(1000 * direction), 500 * direction)

    if direction == 1:
        assert m.get_current_position_estimate() - offset > 950
    else:
        assert m.get_current_position_estimate() - offset < -950

def test_motor(motor):
    test_set_velocity(motor, direction=1)
    test_stop(motor)

    test_set_velocity(motor, direction=-1)
    test_stop(motor)

    test_move_while_seconds(motor, direction=1)
    test_stop(motor)

    test_move_while_seconds(motor, direction=-1)
    test_stop(motor)

    test_move_while_ticks_from_zero(motor, direction=1)
    test_stop(motor)

    test_move_while_ticks_from_zero(motor, direction=-1)
    test_stop(motor)

    test_move_while_ticks_from_non_zero(motor, direction=1)
    test_stop(motor)

    test_move_while_ticks_from_non_zero(motor, direction=-1)
    test_stop(motor)

Motor(0).move_while(for_ticks(1000), 0)

print("Red should be at the bottom now")
wait_for_button_click()
test_motor(Motor(0))
print("Reverse motor and click button once done")
wait_for_button_click()
test_motor(Motor(0, reverse_polarity=True))

print("Seems like everything is working")