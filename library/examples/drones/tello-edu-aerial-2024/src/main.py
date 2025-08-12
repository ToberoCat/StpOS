import time
from libstp_drones.tello import Tello
from libstp.sensor import wait_for_any_button_click
from src.challenge1 import Challenge1
from src.FlyCircle import fly_circle


def main():
    tello = Tello()
    tello.connect()

    battery_percent = tello.get_battery()
    print('Battery Percent:', battery_percent)

    wait_for_any_button_click()

    tello.takeoff()

    Challenge1(tello)
    # fly_circle(tello, radius=50, segments=36)

    tello.land()
    tello.end()


if __name__ == "__main__":
    main()
