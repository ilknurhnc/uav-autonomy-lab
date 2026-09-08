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

from companion.autonomy.obstacle_map import (
    ObstacleMap,
)


latest_obstacles = []
obstacle_lock = threading.Lock()


obstacle_map = ObstacleMap(
    merge_distance=2.5
)


def lidar_callback(msg: LaserScan):

    global latest_obstacles

    obstacles = extract_obstacles(
        msg
    )

    with obstacle_lock:

        latest_obstacles = (
            obstacles
        )


def get_latest_obstacles():

    with obstacle_lock:

        return (
            latest_obstacles.copy()
        )


async def get_local_pose(drone):

    async for position_velocity in (
        drone.telemetry
        .position_velocity_ned()
    ):

        position = (
            position_velocity.position
        )

        drone_north = (
            position.north_m
        )

        drone_east = (
            position.east_m
        )

        break


    async for attitude in (
        drone.telemetry
        .attitude_euler()
    ):

        yaw_deg = (
            attitude.yaw_deg
        )

        break


    return (
        drone_north,
        drone_east,
        yaw_deg,
    )


async def run():

    node = Node()

    print(
        "Connecting to LiDAR..."
    )

    success = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback,
    )

    if not success:

        print(
            "Failed to subscribe "
            "to LiDAR."
        )

        return


    print(
        "LiDAR connected!"
    )


    drone = System()

    print(
        "Connecting to PX4..."
    )

    await drone.connect(
        system_address=(
            "udpin://0.0.0.0:14540"
        )
    )


    async for state in (
        drone.core.connection_state()
    ):

        if state.is_connected:

            print(
                "Connected to PX4!"
            )

            break


    print()
    print(
        "Building local obstacle map..."
    )

    print(
        "Press Ctrl+C to stop."
    )


    await asyncio.sleep(
        1.0
    )


    while True:

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(
            drone
        )


        obstacles = (
            get_latest_obstacles()
        )


        print()
        print(
            "=" * 80
        )

        print(
            f"DRONE | "
            f"North={drone_north:.2f} | "
            f"East={drone_east:.2f} | "
            f"Yaw={yaw_deg:.1f}"
        )

        print(
            f"Current LiDAR clusters: "
            f"{len(obstacles)}"
        )

        print(
            "-" * 80
        )


        for index, obstacle in enumerate(
            obstacles,
            start=1,
        ):

            local_position = (
                sensor_to_local_ned(
                    x_sensor=(
                        obstacle[
                            "x_sensor"
                        ]
                    ),

                    y_sensor=(
                        obstacle[
                            "y_sensor"
                        ]
                    ),

                    drone_north=(
                        drone_north
                    ),

                    drone_east=(
                        drone_east
                    ),

                    yaw_deg=(
                        yaw_deg
                    ),
                )
            )


            local_north = (
                local_position[
                    "north_m"
                ]
            )

            local_east = (
                local_position[
                    "east_m"
                ]
            )


            object_id = (
                obstacle_map
                .add_observation(
                    north=local_north,
                    east=local_east,
                )
            )


            print(
                f"Cluster {index} "
                f"-> Object {object_id} | "
                f"LOCAL N="
                f"{local_north:.2f} | "
                f"E="
                f"{local_east:.2f}"
            )


        mapped_objects = obstacle_map.get_objects()
        confirmed_objects = obstacle_map.get_confirmed_objects()

        print()
        print(
            "LOCAL OBJECT MAP"
        )

        print(
            "-" * 80
        )


        for obstacle_object in (
            mapped_objects
        ):

            status = (
                "CONFIRMED"
                if obstacle_object["confirmed"]
                else "TENTATIVE"
            )
            print(
                f"Object "
                f"{obstacle_object['id']} | "
                f"North="
                f"{obstacle_object['north_m']:.2f} | "
                f"East="
                f"{obstacle_object['east_m']:.2f} | "
                f"Observations="
                f"{obstacle_object['observations']} | "
                f"{status}"
            )


        print(
            f"\nMapped physical objects: "
            f"{len(mapped_objects)}"
        )

        print(
            f"Confirmed physical objects: "
            f"{len(confirmed_objects)}"
        )


        await asyncio.sleep(
            1.0
        )


if __name__ == "__main__":

    try:

        asyncio.run(
            run()
        )

    except KeyboardInterrupt:

        print(
            "\nObstacle map monitor stopped."
        )