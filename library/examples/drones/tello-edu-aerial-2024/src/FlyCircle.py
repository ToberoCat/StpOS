import math
import time


def fly_circle(tello, radius, segments):
    # Calculate the angle step for each segment
    angle_step = 2 * math.pi / segments

    time.sleep(2)  # Wait for the drone to stabilize

    # Fly in a circle
    for i in range(segments):
        angle = i * angle_step
        # Calculate x and y displacements
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        # Convert x and y to absolute movements
        if x > 0:
            tello.move_right(int(abs(x)))
        else:
            tello.move_left(int(abs(x)))

        if y > 0:
            tello.move_forward(int(abs(y)))
        else:
            tello.move_back(int(abs(y)))

        time.sleep(1)  # Short pause between movements
