import threading

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan

from companion.vision.lidar_viewer import (
    LIDAR_TOPIC,
    extract_obstacles,
)


CAMERA_TOPIC = (
    "/world/vision_test/model/x500_lidar_camera_0/"
    "link/camera_link/sensor/camera/image"
)


latest_frame = None
frame_lock = threading.Lock()

latest_obstacles = []
obstacle_lock = threading.Lock()


def image_callback(msg: Image):
    global latest_frame

    frame = np.frombuffer(
        msg.data,
        dtype=np.uint8,
    )

    frame = frame.reshape(
        (msg.height, msg.width, 3)
    )

    frame = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR,
    )

    with frame_lock:
        latest_frame = frame.copy()


def lidar_callback(msg: LaserScan):
    global latest_obstacles

    obstacles = extract_obstacles(
        msg
    )

    with obstacle_lock:
        latest_obstacles = obstacles


def get_latest_frame():
    with frame_lock:
        if latest_frame is None:
            return None

        return latest_frame.copy()


def get_latest_obstacles():
    with obstacle_lock:
        return latest_obstacles.copy()


def start_sensors():
    node = Node()

    print("Connecting to LiDAR...")

    lidar_success = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback,
    )

    if not lidar_success:
        print(
            "Failed to subscribe to LiDAR."
        )

        return None

    print("LiDAR connected!")

    print("Connecting to camera...")

    camera_success = node.subscribe(
        Image,
        CAMERA_TOPIC,
        image_callback,
    )

    if not camera_success:
        print(
            "Failed to subscribe to camera."
        )

        return None

    print("Camera connected!")

    return node