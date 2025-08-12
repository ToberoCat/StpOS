import time

from libstp_drones.tello import Tello


class Challenge1:
    def __init__(self, tello: Tello):
        self.tello = tello
        self.run()

    def run(self):

        self.tello.move_up(50)
        time.sleep(0.5)
        self.tello.move_down(50)
        time.sleep(0.5)
        self.tello.move_left(50)
        time.sleep(0.5)
        self.tello.move_right(50)
        time.sleep(0.5)
        self.tello.rotate_ccw(90)
        time.sleep(0.5)
        self.tello.rotate_cw(90)
        time.sleep(1)
        self.tello.flip('f')
        time.sleep(1)
        self.tello.flip('b')
        time.sleep(1)
        self.tello.flip('l')
        time.sleep(1)
        self.tello.flip('r')
        time.sleep(1)



