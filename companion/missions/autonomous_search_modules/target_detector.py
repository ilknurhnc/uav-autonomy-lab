import cv2
import numpy as np


def detect_red_target(frame):
    if frame is None:
        return None

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV,
    )

    lower_red_1 = np.array([0, 120, 70])
    upper_red_1 = np.array([10, 255, 255])

    lower_red_2 = np.array([170, 120, 70])
    upper_red_2 = np.array([180, 255, 255])

    mask_1 = cv2.inRange(
        hsv,
        lower_red_1,
        upper_red_1,
    )

    mask_2 = cv2.inRange(
        hsv,
        lower_red_2,
        upper_red_2,
    )

    mask = mask_1 | mask_2

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if not contours:
        return None

    largest_contour = max(
        contours,
        key=cv2.contourArea,
    )

    area = cv2.contourArea(
        largest_contour
    )

    if area < 500:
        return None

    x, y, width, height = (
        cv2.boundingRect(
            largest_contour
        )
    )

    center_x = (
        x + width // 2
    )

    center_y = (
        y + height // 2
    )

    return {
        "center_x": center_x,
        "center_y": center_y,
        "area": area,
    }