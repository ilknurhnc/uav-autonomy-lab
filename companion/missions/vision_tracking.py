import asyncio
import threading

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityBodyYawspeed,
)

from companion.autonomy.motion_controller import (
    calculate_control_command,
    control_to_yaw_speed,
)


CAMERA_TOPIC = (
    "/world/vision_test/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)

TAKEOFF_ALTITUDE = 2.5

SEARCH = "SEARCH"
TRACK = "TRACK"
CENTERED = "CENTERED"

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


def detect_red_target(frame):
    hsv = cv2.cvtColor(
        frame,
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

    if not contours:
        return None, mask

    target = max(
        contours,
        key=cv2.contourArea
    )

    area = cv2.contourArea(target)

    if area < 500:
        return None, mask

    x, y, w, h = cv2.boundingRect(target)

    target_x = x + w // 2
    target_y = y + h // 2

    height, width, _ = frame.shape

    center_x = width // 2
    center_y = height // 2

    error_x = target_x - center_x
    error_y = target_y - center_y

    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "target_x": target_x,
        "target_y": target_y,
        "center_x": center_x,
        "center_y": center_y,
        "error_x": error_x,
        "error_y": error_y,
        "area": area,
    }, mask


async def run():
    global latest_frame

    node = Node()

    print("Connecting to Gazebo camera...")

    success = node.subscribe(
        Image,
        CAMERA_TOPIC,
        image_callback
    )

    if not success:
        print("Failed to subscribe to camera.")
        return

    print("Camera connected!")

    drone = System()

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for vehicle...")

    async for health in drone.telemetry.health():
        print(
            f"Local: {health.is_local_position_ok} | "
            f"Global: {health.is_global_position_ok} | "
            f"Home: {health.is_home_position_ok} | "
            f"Armable: {health.is_armable}"
        )

        if (
            health.is_local_position_ok
            and health.is_home_position_ok
            and health.is_armable
        ):
            print("Vehicle ready!")
            break

    await drone.action.set_takeoff_altitude(
        TAKEOFF_ALTITUDE
    )

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m
        print(f"Altitude: {altitude:.2f} m")

        if altitude >= TAKEOFF_ALTITUDE * 0.90:
            break

    print("Preparing Offboard...")

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,
            0.0,
            0.0,
            0.0
        )
    )

    try:
        await drone.offboard.start()
        print("Offboard started!")
    except OffboardError as error:
        print(f"Offboard failed: {error}")
        await drone.action.land()
        return

    print("Vision tracking active.")
    print("Press Q in camera window to stop.")

    state = SEARCH

    try:
        while True:

            with frame_lock:
                if latest_frame is not None:
                    frame = latest_frame.copy()
                else:
                    frame = None

            if frame is None:
                await asyncio.sleep(0.05)
                continue

            target, mask = detect_red_target(frame)

            yaw_speed = 0.0

            if target is None:
                state = SEARCH
                yaw_speed = 10.0

                print(
                    f"State: {state} | "
                    f"Target not visible | "
                    f"Yaw speed: {yaw_speed:.1f} deg/s"
                )

            else:
                error_x = target["error_x"]

                control_command = calculate_control_command(
                    error_x
                )

                yaw_speed = control_to_yaw_speed(
                    control_command
                )

                if abs(error_x) < 20:
                    state = CENTERED
                    yaw_speed = 0.0
                else:
                    state = TRACK

                x = target["x"]
                y = target["y"]
                w = target["w"]
                h = target["h"]

                target_x = target["target_x"]
                target_y = target["target_y"]

                center_x = target["center_x"]
                center_y = target["center_y"]

                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (target_x, target_y),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.circle(
                    frame,
                    (center_x, center_y),
                    5,
                    (255, 0, 0),
                    -1
                )

                print(
                    f"State: {state} | "
                    f"Error X: {error_x:4d} | "
                    f"Yaw speed: {yaw_speed:6.2f} deg/s"
                )

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    yaw_speed
                )
            )

            cv2.imshow(
                "Vision Tracking",
                frame
            )

            cv2.imshow(
                "Red Mask",
                mask
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            await asyncio.sleep(0.05)

    finally:
        print("Stopping vehicle motion...")

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                0.0
            )
        )

        await asyncio.sleep(0.5)

        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Offboard stop failed: {error}")

        print("Landing...")
        await drone.action.land()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(run())