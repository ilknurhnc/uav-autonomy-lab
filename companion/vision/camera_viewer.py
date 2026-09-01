import cv2
import numpy as np
import threading

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from companion.autonomy.motion_controller import (calculate_control_command, control_to_yaw_speed)


CAMERA_TOPIC = (
    "/world/vision_test/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)


latest_frame = None
frame_lock = threading.Lock()


def image_callback(msg: Image):
    global latest_frame

    frame = np.frombuffer(
        msg.data,
        dtype=np.uint8
    )

    frame = frame.reshape(
        (msg.height, msg.width, 3)
    )

    frame = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR
    )

    with frame_lock:
        latest_frame = frame.copy()


def main():
    global latest_frame

    node = Node()

    print("Subscribing to camera topic...")

    success = node.subscribe(
        Image,
        CAMERA_TOPIC,
        image_callback
    )

    if not success:
        print("Failed to subscribe to camera topic.")
        return

    print("Camera connected!")
    print("Press Q to stop.")

    while True:

        with frame_lock:
            if latest_frame is not None:
                frame_to_show = latest_frame.copy()
            else:
                frame_to_show = None

        if frame_to_show is not None:

            hsv = cv2.cvtColor(
                frame_to_show,
                cv2.COLOR_BGR2HSV
            )


            lower_red_1 = np.array([0, 120, 70])
            upper_red_1 = np.array([10, 255, 255])

            lower_red_2 = np.array([170, 120, 70])
            upper_red_2 = np.array([179, 255, 255])


            mask_1 = cv2.inRange(
                hsv,
                lower_red_1,
                upper_red_1
            )

            mask_2 = cv2.inRange(
                hsv,
                lower_red_2,
                upper_red_2
            )

            mask = cv2.bitwise_or(
                mask_1,
                mask_2
            )


            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )


            if contours:

                target = max(
                    contours,
                    key=cv2.contourArea
                )

                area = cv2.contourArea(target)

                if area > 500:

                    x, y, w, h = cv2.boundingRect(target)

                    target_x = x + w // 2
                    target_y = y + h // 2

                    height, width, _ = frame_to_show.shape

                    center_x = width // 2
                    center_y = height // 2

                    error_x = target_x - center_x
                    error_y = target_y - center_y

                    control_command = calculate_control_command(error_x)
                    yaw_speed = control_to_yaw_speed(control_command)

                    cv2.rectangle(
                        frame_to_show,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        frame_to_show,
                        (target_x, target_y),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    cv2.circle(
                        frame_to_show,
                        (center_x, center_y),
                        5,
                        (255, 0, 0),
                        -1
                    )

                    print(
                        f"Target: ({target_x}, {target_y}) "
                        f"Error: ({error_x}, {error_y}) "
                        f"Control: {control_command:.2f} "
                        f"Yaw speed: {yaw_speed:.1f} deg/s "
                        f"Area: {area:.0f}"
                    )

            cv2.imshow(
                "Red Mask",
                mask
            )

            cv2.imshow(
                "X500 Camera",
                frame_to_show
            )

        key = cv2.waitKey(10) & 0xFF

        if key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()