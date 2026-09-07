import asyncio
import math
import threading

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
)

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

from companion.vision.lidar_viewer import (
    LIDAR_TOPIC,
    extract_obstacles,
)

from companion.autonomy.coordinate_transform import (
    sensor_to_local_ned,
)


TAKEOFF_ALTITUDE = 2.5
SAFE_DISTANCE = 5.0
POSITION_TOLERANCE = 0.5


latest_obstacles = []
obstacle_lock = threading.Lock()


def lidar_callback(msg: LaserScan):

    global latest_obstacles

    obstacles = extract_obstacles(msg)

    with obstacle_lock:
        latest_obstacles = obstacles


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


async def run():

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

    async for state in drone.core.connection_state():

        if state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for vehicle to be ready...")

    async for health in drone.telemetry.health():

        if (
            health.is_global_position_ok
            and health.is_home_position_ok
        ):
            print("Vehicle is ready!")
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

        print(
            f"Altitude: {altitude:.2f} m"
        )

        if altitude >= TAKEOFF_ALTITUDE * 0.90:
            print("Takeoff altitude reached!")
            break

    await asyncio.sleep(1)

    print("Reading drone pose...")

    drone_north, drone_east, yaw_deg = (
        await get_local_pose(drone)
    )

    print(
        f"Drone: "
        f"North={drone_north:.2f} | "
        f"East={drone_east:.2f} | "
        f"Yaw={yaw_deg:.1f}"
    )

    obstacles = get_latest_obstacles()

    if not obstacles:

        print("No LiDAR obstacle detected.")
        await drone.action.land()
        return

    print(
        f"Detected obstacles: {len(obstacles)}"
    )

    selected_obstacle = min(
        obstacles,
        key=lambda obstacle: obstacle["distance"],
    )

    print(
        f"Selected obstacle: "
        f"Distance={selected_obstacle['distance']:.2f} m | "
        f"Angle={selected_obstacle['angle_deg']:.1f} deg"
    )

    obstacle_local = sensor_to_local_ned(
        x_sensor=selected_obstacle["x_sensor"],
        y_sensor=selected_obstacle["y_sensor"],
        drone_north=drone_north,
        drone_east=drone_east,
        yaw_deg=yaw_deg,
    )

    obstacle_north = obstacle_local["north_m"]
    obstacle_east = obstacle_local["east_m"]

    print(
        f"Obstacle local position: "
        f"North={obstacle_north:.2f} | "
        f"East={obstacle_east:.2f}"
    )

    north_offset = (
        obstacle_north - drone_north
    )

    east_offset = (
        obstacle_east - drone_east
    )

    obstacle_distance = math.sqrt(
        north_offset ** 2
        + east_offset ** 2
    )

    if obstacle_distance <= SAFE_DISTANCE:

        print(
            "Obstacle is already too close. "
            "Mission aborted."
        )

        await drone.action.land()
        return

    travel_distance = (
        obstacle_distance - SAFE_DISTANCE
    )

    north_direction = (
        north_offset / obstacle_distance
    )

    east_direction = (
        east_offset / obstacle_distance
    )

    target_north = (
        drone_north
        + north_direction * travel_distance
    )

    target_east = (
        drone_east
        + east_direction * travel_distance
    )

    target_down = -TAKEOFF_ALTITUDE

    print(
        f"Inspection target: "
        f"North={target_north:.2f} | "
        f"East={target_east:.2f}"
    )

    print("Preparing Offboard...")

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

        print("Offboard started!")

    except OffboardError as error:

        print(
            f"Offboard start failed: {error}"
        )

        await drone.action.land()
        return

    print("Flying toward obstacle...")

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

        position = position_velocity.position

        north_error = (
            target_north - position.north_m
        )

        east_error = (
            target_east - position.east_m
        )

        print(
            f"North={position.north_m:.2f} | "
            f"East={position.east_m:.2f} | "
            f"N error={north_error:.2f} | "
            f"E error={east_error:.2f}"
        )

        if (
            abs(north_error) < POSITION_TOLERANCE
            and
            abs(east_error) < POSITION_TOLERANCE
        ):

            print("Inspection position reached!")
            break

    print("Hovering near obstacle...")

    await asyncio.sleep(3)

    try:
        await drone.offboard.stop()

    except OffboardError as error:
        print(
            f"Offboard stop failed: {error}"
        )

    print("Landing...")

    await drone.action.land()

    async for in_air in drone.telemetry.in_air():

        if not in_air:
            print("Landed!")
            break


if __name__ == "__main__":

    try:
        asyncio.run(run())

    except KeyboardInterrupt:
        print("\nMission stopped.")