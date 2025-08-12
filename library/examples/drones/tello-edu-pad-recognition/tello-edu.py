from libstp_drones.tello import Tello

tello = Tello()

tello.connect()

tello.enable_mission_pads()
tello.set_mission_pad_detection_direction(2)

tello.takeoff()

pad = tello.get_mission_pad_id()
while pad != 1:
    if pad == 3:
        tello.move_forward(30)
        tello.rotate_clockwise(90)
        tello.move_forward(50)

    if pad == 4:
        tello.move_up(30)
        tello.flip_forward()
        tello.rotate_counter_clockwise(90)

    pad = tello.get_mission_pad_id()
    tello.move_forward(20)
    print(pad)

tello.disable_mission_pads()
tello.land()
tello.end()