import cv2


def list_cameras():
    """ Lists available camera devices by checking indexes 0-10. """
    available_cameras = []
    for i in range(10):  # Checking first 10 indexes
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras


def capture_image(camera_index=0):
    """ Opens the camera, captures an image, and saves it. """
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"Failed to open camera {camera_index}")
        return

    ret, frame = cap.read()
    cap.release()

    if ret:
        filename = "captured_image2.jpg"
        cv2.imwrite(filename, frame)
        print(f"Image saved as {filename}")
        cv2.imshow("Captured Image", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Failed to capture image.")


if __name__ == "__main__":
    cameras = list_cameras()
    if cameras:
        print(f"Available cameras: {cameras}")
        capture_image(cameras[1])  # Capture using the first available camera
    else:
        print("No available cameras found.")
