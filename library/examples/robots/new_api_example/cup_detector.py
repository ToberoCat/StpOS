from enum import Enum

import cv2
import numpy as np
from libstp.logging import error
from libstp.sensor import is_button_clicked


class CupColor(Enum):
    Red = 0
    Green = 1
    Blue = 2


def _find_largest_contour(mask):
    """
    Find the largest contour in the given mask and return its bounding box.
    Returns None if no contour is found.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    return x, y, w, h


class CupDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            raise Exception("Error: Could not open camera.")

    def __del__(self):
        self.cap.release()

    def _detect_cups(self, image_bgr):
        """
        Detect three cups (red, green, blue) in the given BGR image.
        Returns a dictionary { 'red': (x, y, w, h), 'green': (...), 'blue': (...) }
        if detection is successful for all three colors, otherwise returns None.
        """
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        # Define color ranges
        red_lower1, red_upper1 = np.array([0, 100, 80]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 100, 80]), np.array([180, 255, 255])
        green_lower, green_upper = np.array([35, 80, 70]), np.array([85, 255, 255])
        blue_lower, blue_upper = np.array([90, 80, 70]), np.array([130, 255, 255])

        # Create masks
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, red_lower1, red_upper1), cv2.inRange(hsv, red_lower2, red_upper2))
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        # Find bounding boxes
        red_box = _find_largest_contour(red_mask)
        green_box = _find_largest_contour(green_mask)
        blue_box = _find_largest_contour(blue_mask)
        if red_box is None or green_box is None or blue_box is None:
            return None

        for c_box in (red_box, green_box, blue_box):
            _, _, w, h = c_box
            if w < 30 or h < 30:
                return None

        return {'red': red_box, 'green': green_box, 'blue': blue_box}

    def detect_cups(self):
        """
        Capture a frame from the camera and detect three cups (red, green, blue).
        Returns a dictionary {'left': CupColor.Red, 'middle': CupColor.Green, 'right': CupColor.Blue}
        if detection is successful for all three colors, otherwise returns None.
        """
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to grab frame")

        results = self._detect_cups(frame)
        if results is None or any(box is None for box in results.values()):
            error("Failed to detect all three colors.")
            return None

        color_list = list(results.items())
        color_list.sort(key=lambda item: item[1][0])
        return {'left': CupColor[color_list[0][0].capitalize()], 'middle': CupColor[color_list[1][0].capitalize()],
                'right': CupColor[color_list[2][0].capitalize()]}


def main():
    detector = CupDetector()
    while not is_button_clicked():
        cups = detector.detect_cups()
        if cups is not None:
            print(cups)


if __name__ == '__main__':
    main()
