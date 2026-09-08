import asyncio
import threading

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan

from mavsdk import System

from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
)

from companion.vision.lidar_viewer import (
    LIDAR_TOPIC,
    extract_obstacles,
)

from companion.autonomy.obstacle_map import (
    ObstacleMap,
)

from companion.autonomy.coordinate_transform import (
    sensor_to_local_ned,
)

from companion.autonomy.motion_controller import (
    calculate_control_command,
    control_to_yaw_speed,
)


CAMERA_TOPIC = (
    "/world/vision_test/model/x500_lidar_camera_0/"
    "link/camera_link/sensor/camera/image"
)

TAKEOFF_ALTITUDE = 2.5
SAFE_DISTANCE = 5.0
POSITION_TOLERANCE = 0.5


TAKEOFF = "TAKEOFF"
BUILD_MAP = "BUILD_MAP"
SELECT_OBJECT = "SELECT_OBJECT"
MOVE_TO_VIEWPOINT = "MOVE_TO_VIEWPOINT"
SEARCH_TARGET = "SEARCH_TARGET"
NEXT_OBJECT = "NEXT_OBJECT"
TRACK = "TRACK"
CENTERED = "CENTERED"


latest_frame = None
frame_lock = threading.Lock()

latest_obstacles = []
obstacle_lock = threading.Lock()

obstacle_map = ObstacleMap(
    merge_distance=4.0
)

inspected_object_ids = set()

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


def lidar_callback(msg: LaserScan):
    global latest_obstacles

    obstacles = extract_obstacles(msg)

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


async def get_local_pose(drone):
    async for position_velocity in (
        drone.telemetry.position_velocity_ned()
    ):
        north = position_velocity.position.north_m
        east = position_velocity.position.east_m

        break

    async for attitude in (
        drone.telemetry.attitude_euler()
    ):
        yaw = attitude.yaw_deg
        break

    return north, east, yaw


async def build_confirmed_obstacle_map(drone):
    print()
    print("Building obstacle map...")

    scan_count = 0
    max_scans = 10

    while scan_count < max_scans:
        scan_count += 1

        print()
        print(
            f"Map scan: "
            f"{scan_count}/{max_scans}"
        )

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(drone)

        obstacles = get_latest_obstacles()

        print(
            f"LiDAR clusters: {len(obstacles)}"
        )

        for obstacle in obstacles:
            obstacle_local = (
                sensor_to_local_ned(
                    x_sensor=obstacle["x_sensor"],
                    y_sensor=obstacle["y_sensor"],
                    drone_north=drone_north,
                    drone_east=drone_east,
                    yaw_deg=yaw_deg,
                )
            )

            obstacle_map.add_observation(
                north=obstacle_local["north_m"],
                east=obstacle_local["east_m"],
            )

        mapped_objects = (
            obstacle_map.get_objects()
        )

        confirmed_objects = (
            obstacle_map.get_confirmed_objects()
        )

        print(
            f"Mapped: {len(mapped_objects)} | "
            f"Confirmed: {len(confirmed_objects)}"
        )

        for obstacle_object in mapped_objects:
            status = (
                "CONFIRMED"
                if obstacle_object["confirmed"]
                else "TENTATIVE"
            )

            print(
                f"Object {obstacle_object['id']} | "
                f"N={obstacle_object['north_m']:.2f} | "
                f"E={obstacle_object['east_m']:.2f} | "
                f"Obs={obstacle_object['observations']} | "
                f"{status}"
            )

        if confirmed_objects:
            print()
            print(
                "Confirmed obstacle map ready!"
            )

            return confirmed_objects

        await asyncio.sleep(1.0)

    print()
    print(
        "No confirmed objects "
        "after maximum map scans."
    )

    return []


def select_next_object(
    confirmed_objects,
    drone_north,
    drone_east,
):
    selected_object = None
    selected_distance = None

    for obstacle_object in confirmed_objects:
        object_id = obstacle_object["id"]

        if object_id in inspected_object_ids:
            continue

        north_offset = (
            obstacle_object["north_m"]
            - drone_north
        )

        east_offset = (
            obstacle_object["east_m"]
            - drone_east
        )

        distance = (
            north_offset ** 2
            + east_offset ** 2
        ) ** 0.5

        if distance <= SAFE_DISTANCE:
            continue

        if (
            selected_distance is None
            or distance < selected_distance
        ):
            selected_object = obstacle_object
            selected_distance = distance

    return selected_object, selected_distance


def calculate_viewpoint(
    drone_north,
    drone_east,
    obstacle_north,
    obstacle_east,
    obstacle_distance,
):
    north_offset = (
        obstacle_north
        - drone_north
    )

    east_offset = (
        obstacle_east
        - drone_east
    )

    north_direction = (
        north_offset
        / obstacle_distance
    )

    east_direction = (
        east_offset
        / obstacle_distance
    )

    travel_distance = (
        obstacle_distance
        - SAFE_DISTANCE
    )

    target_north = (
        drone_north
        + north_direction
        * travel_distance
    )

    target_east = (
        drone_east
        + east_direction
        * travel_distance
    )

    return target_north, target_east


async def run():
    print("Autonomous search mission starting...")

    state = TAKEOFF

    node = Node()

    print("Connecting to LiDAR...")

    success = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback,
    )

    if not success:
        print("Failed to subscribe to LiDAR.")
        return

    print("LiDAR connected!")

    drone = System()

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    async for connection_state in (
        drone.core.connection_state()
    ):
        if connection_state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for vehicle to be ready...")

    async for health in drone.telemetry.health():
        print(
            f"Local: {health.is_local_position_ok} | "
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

    print()
    print(f"Current state: {state}")

    if state == TAKEOFF:
        await drone.action.set_takeoff_altitude(
            TAKEOFF_ALTITUDE
        )

        print("Arming...")
        await drone.action.arm()

        print("Taking off...")
        await drone.action.takeoff()

        async for position in (
            drone.telemetry.position()
        ):
            altitude = (
                position.relative_altitude_m
            )

            print(
                f"Altitude: {altitude:.2f} m"
            )

            if (
                altitude
                >= TAKEOFF_ALTITUDE * 0.90
            ):
                print(
                    "Takeoff altitude reached!"
                )
                break

        state = BUILD_MAP

        print()
        print(
            f"State transition: "
            f"TAKEOFF -> {state}"
        )

    if state == BUILD_MAP:
        print()
        print(
            f"Current state: {state}"
        )

        confirmed_objects = (
            await build_confirmed_obstacle_map(
                drone
            )
        )

        if not confirmed_objects:
            print(
                "No confirmed objects. "
                "Mission cannot continue."
            )

            await drone.action.land()
            return

        print()
        print(
            f"Confirmed objects available: "
            f"{len(confirmed_objects)}"
        )

        state = SELECT_OBJECT

        print()
        print(
            f"State transition: "
            f"BUILD_MAP -> {state}"
        )

    if state == SELECT_OBJECT:
        print()
        print(
            f"Current state: {state}"
        )

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(drone)

        (
            selected_object,
            selected_distance,
        ) = select_next_object(
            confirmed_objects,
            drone_north,
            drone_east,
        )

        if selected_object is None:
            print(
                "No uninspected object available."
            )

            await drone.action.land()
            return

        print()
        print(
            f"Selected Object "
            f"{selected_object['id']}"
        )

        print(
            f"Object position: "
            f"N={selected_object['north_m']:.2f} | "
            f"E={selected_object['east_m']:.2f}"
        )

        print(
            f"Distance: "
            f"{selected_distance:.2f} m"
        )

        state = MOVE_TO_VIEWPOINT

        print()
        print(
            f"State transition: "
            f"SELECT_OBJECT -> {state}"
        )

    if state == MOVE_TO_VIEWPOINT:
        print()
        print(
            f"Current state: {state}"
        )

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(drone)

        obstacle_north = (
            selected_object["north_m"]
        )

        obstacle_east = (
            selected_object["east_m"]
        )

        north_offset = (
            obstacle_north
            - drone_north
        )

        east_offset = (
            obstacle_east
            - drone_east
        )

        obstacle_distance = (
            north_offset ** 2
            + east_offset ** 2
        ) ** 0.5

        (
            target_north,
            target_east,
        ) = calculate_viewpoint(
            drone_north,
            drone_east,
            obstacle_north,
            obstacle_east,
            obstacle_distance,
        )

        target_down = (
            -TAKEOFF_ALTITUDE
        )

        print(
            f"Viewpoint: "
            f"N={target_north:.2f} | "
            f"E={target_east:.2f}"
        )

        print(
            "Preparing Offboard..."
        )

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                drone_north,
                drone_east,
                target_down,
                yaw_deg,
            )
        )

        try:
            await drone.offboard.start()

            print(
                "Offboard started!"
            )

        except OffboardError as error:
            print(
                f"Offboard start failed: "
                f"{error}"
            )

            await drone.action.land()
            return

        print(
            f"Flying toward viewpoint "
            f"for Object "
            f"{selected_object['id']}..."
        )

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                target_north,
                target_east,
                target_down,
                yaw_deg,
            )
        )

        async for position_velocity in (
            drone.telemetry.position_velocity_ned()
        ):
            position = (
                position_velocity.position
            )

            north_error = (
                target_north
                - position.north_m
            )

            east_error = (
                target_east
                - position.east_m
            )

            print(
                f"N={position.north_m:.2f} | "
                f"E={position.east_m:.2f} | "
                f"N error={north_error:.2f} | "
                f"E error={east_error:.2f}"
            )

            if (
                abs(north_error)
                < POSITION_TOLERANCE
                and
                abs(east_error)
                < POSITION_TOLERANCE
            ):
                print(
                    "Viewpoint reached!"
                )
                break

        state = SEARCH_TARGET

        print()
        print(
            f"State transition: "
            f"MOVE_TO_VIEWPOINT -> {state}"
        )

    print()
    print(
        f"Current state: {state}"
    )


if __name__ == "__main__":
    asyncio.run(run())