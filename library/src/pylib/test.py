import cv2, numpy as np

cap = cv2.VideoCapture(3)


def send_to_motors(v, omega):
    """
    User-supplied function to send velocity and angular velocity to the motors.
    Replace this with actual motor control code.
    """
    print(f"Velocity: {v:.2f}, Angular Velocity: {omega:.2f}")

Kp_yaw, Kp_lat, gap_px = 2.5, 0.015, 60
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
Kp_angle, Kp_offset = 2.0, 0.01

while True:
    ok, frame = cap.read();  h,w = frame.shape[:2];  cx,cy = w//2, h//2
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    msk  = cv2.morphologyEx(cv2.inRange(hsv,(0,0,0),(180,255,60)),
                            cv2.MORPH_CLOSE, kernel)
    ys,xs = np.where(msk);   pts = np.column_stack((xs,ys)).astype(np.float32)
    vx,vy,x0,y0 = cv2.fitLine(pts,cv2.DIST_L2,0,0.01,0.01)
    vx,vy,x0,y0 = map(float,(vx,vy,x0,y0))
    if vy<0: vx,vy = -vx,-vy                         # unify direction
    nx,ny =  vy, -vx                                 # right-hand normal
    x0o,y0o = x0+nx*gap_px, y0+ny*gap_px             # offset anchor

    theta = np.arctan2(vy,vx)
    dist  = ((cx-x0o)*nx + (cy-y0o)*ny)
    omega = Kp_yaw * theta
    v     = max(0.05, 0.25 - abs(Kp_lat*dist))

    send_to_motors(v, omega)                         # <- your driver

    # debug overlay (optional)
    L = max(h,w)
    cv2.line(frame,(int(x0-vx*L),int(y0-vy*L)),
             (int(x0+vx*L),int(y0+vy*L)),(0,255,0),2)      # main line
    cv2.line(frame,(int(x0o-vx*L),int(y0o-vy*L)),
             (int(x0o+vx*L),int(y0o+vy*L)),(255,0,0),2)    # offset path
    cv2.imshow('follow',frame); 0xFF & cv2.waitKey(1)
