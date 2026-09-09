import asyncio

from companion.autonomy.coordinate_transform import (
    sensor_to_local_ned,
)


async def get_local_pose(drone):
    async for position_velocity in (
        drone.telemetry.position_velocity_ned()
    ):
        north = (
            position_velocity.position.north_m
        )

        east = (
            position_velocity.position.east_m
        )

        break

    async for attitude in (
        drone.telemetry.attitude_euler()
    ):
        yaw = attitude.yaw_deg
        break

    return (
        north,
        east,
        yaw,
    )


async def build_confirmed_obstacle_map(
    drone,
    obstacle_map,
    get_latest_obstacles,
):
    print()
    print(
        "Building obstacle map..."
    )

    scan_count = 0
    max_scans = 10

    while scan_count < max_scans:
        scan_count += 1

        print()
        print(
            f"Map scan: "
            f"{scan_count}/"
            f"{max_scans}"
        )

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

        print(
            f"LiDAR clusters: "
            f"{len(obstacles)}"
        )

        for obstacle in obstacles:
            obstacle_local = (
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

            obstacle_map.add_observation(
                north=(
                    obstacle_local[
                        "north_m"
                    ]
                ),
                east=(
                    obstacle_local[
                        "east_m"
                    ]
                ),
            )

        mapped_objects = (
            obstacle_map.get_objects()
        )

        confirmed_objects = (
            obstacle_map
            .get_confirmed_objects()
        )

        print(
            f"Mapped: "
            f"{len(mapped_objects)} | "
            f"Confirmed: "
            f"{len(confirmed_objects)}"
        )

        for obstacle_object in (
            mapped_objects
        ):
            status = (
                "CONFIRMED"
                if obstacle_object[
                    "confirmed"
                ]
                else "TENTATIVE"
            )

            print(
                f"Object "
                f"{obstacle_object['id']} | "
                f"N="
                f"{obstacle_object['north_m']:.2f} | "
                f"E="
                f"{obstacle_object['east_m']:.2f} | "
                f"Obs="
                f"{obstacle_object['observations']} | "
                f"{status}"
            )

        if confirmed_objects:
            print()
            print(
                "Confirmed obstacle map ready!"
            )

            return confirmed_objects

        await asyncio.sleep(
            1.0
        )

    print()
    print(
        "No confirmed objects "
        "after maximum map scans."
    )

    return []