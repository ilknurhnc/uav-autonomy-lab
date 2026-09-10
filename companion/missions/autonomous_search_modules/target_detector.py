import cv2
import numpy as np


MIN_TARGET_AREA = 120


def detect_red_target(frame):
    if frame is None:
        return None

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV,
    )

    lower_red_1 = np.array(
        [0, 90, 50]
    )

    upper_red_1 = np.array(
        [12, 255, 255]
    )

    lower_red_2 = np.array(
        [168, 90, 50]
    )

    upper_red_2 = np.array(
        [180, 255, 255]
    )

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

    mask = (
        mask_1
        | mask_2
    )

    kernel = np.ones(
        (3, 3),
        np.uint8,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    contours, _ = (
        cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
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

    if (
        area
        < MIN_TARGET_AREA
    ):
        return None

    (
        x,
        y,
        width,
        height,
    ) = cv2.boundingRect(
        largest_contour
    )

    center_x = (
        x
        + width // 2
    )

    center_y = (
        y
        + height // 2
    )

    return {
        "center_x":
            center_x,

        "center_y":
            center_y,

        "area":
            area,

        "width":
            width,

        "height":
            height,
    }