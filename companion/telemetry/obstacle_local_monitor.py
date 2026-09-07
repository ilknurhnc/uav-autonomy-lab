import asyncio
import threading

from mavsdk import System

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

from companion.vision.lidar_viewer import (
    LIDAR_TOPIC,
    extract_obstacles,
)

from companion.autonomy.coordinate_transform import (
    sensor_to_local_ned,
)


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

        position = position_velocity.position

        north = position.north_m
        east = position.east_m

        break

    async for attitude in (
        drone.telemetry.attitude_euler()
    ):

        yaw = attitude.yaw_deg

        break

    return north, east, yaw


async def run():

    # -----------------------------
    # LiDAR bağlantısı
    # -----------------------------

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

    # -----------------------------
    # PX4 bağlantısı
    # -----------------------------

    drone = System()

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    async for state in (
        drone.core.connection_state()
    ):

        if state.is_connected:

            print("Connected to PX4!")
            break

    print()
    print("Monitoring obstacle local positions...")
    print("Press Ctrl+C to stop.")

    # LiDAR'ın ilk scan'i gelsin
    await asyncio.sleep(1.0)

    # -----------------------------
    # Monitor döngüsü
    # -----------------------------

    while True:

        drone_north, drone_east, yaw_deg = (
            await get_local_pose(drone)
        )

        obstacles = get_latest_obstacles()

        print()
        print("=" * 80)

        print(
            f"DRONE | "
            f"North={drone_north:.2f} | "
            f"East={drone_east:.2f} | "
            f"Yaw={yaw_deg:.1f}"
        )

        print(
            f"Detected clusters: "
            f"{len(obstacles)}"
        )

        print("-" * 80)

        for index, obstacle in enumerate(
            obstacles,
            start=1,
        ):

            local_position = sensor_to_local_ned(
                x_sensor=obstacle["x_sensor"],
                y_sensor=obstacle["y_sensor"],
                drone_north=drone_north,
                drone_east=drone_east,
                yaw_deg=yaw_deg,
            )

            print(
                f"Cluster {index} | "
                f"D={obstacle['distance']:.2f} m | "
                f"A={obstacle['angle_deg']:.1f} deg | "
                f"Sensor X={obstacle['x_sensor']:.2f} | "
                f"Y={obstacle['y_sensor']:.2f} || "
                f"LOCAL N={local_position['north_m']:.2f} | "
                f"E={local_position['east_m']:.2f} | "
                f"Points={obstacle['points']}"
            )

        await asyncio.sleep(1.0)


if __name__ == "__main__":

    try:

        asyncio.run(run())

    except KeyboardInterrupt:

        print(
            "\nObstacle local monitor stopped."
        )