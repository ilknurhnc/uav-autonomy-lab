import cv2
import numpy as np
import threading

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


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