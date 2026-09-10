import asyncio

from mavsdk.offboard import (
    VelocityBodyYawspeed,
)

from companion.autonomy.coordinate_transform import (
    sensor_to_local_ned,
)


MAP_SCAN_COUNT = 10

MAP_SCAN_INTERVAL = 0.25


async def get_local_pose(
    drone,
):
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
    keep_offboard_alive=False,
):
    print()
    print(
        "Building / refreshing obstacle map..."
    )

    for scan_count in range(
        1,
        MAP_SCAN_COUNT + 1,
    ):

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
            f"Map scan: "
            f"{scan_count}/"
            f"{MAP_SCAN_COUNT} | "
            f"LiDAR clusters: "
            f"{len(obstacles)}"
        )

        for obstacle in obstacles:

            obstacle_local = (
                sensor_to_local_ned(
                    x_sensor=
                        obstacle["x_sensor"],

                    y_sensor=
                        obstacle["y_sensor"],

                    drone_north=
                        drone_north,

                    drone_east=
                        drone_east,

                    yaw_deg=
                        yaw_deg,
                )
            )

            obstacle_map.add_observation(
                north=
                    obstacle_local["north_m"],

                east=
                    obstacle_local["east_m"],
            )

        mapped_objects = (
            obstacle_map.get_objects()
        )

        confirmed_objects = (
            obstacle_map.get_confirmed_objects()
        )

        print(
            f"Mapped: "
            f"{len(mapped_objects)} | "
            f"Confirmed: "
            f"{len(confirmed_objects)}"
        )

        if keep_offboard_alive:

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )

        await asyncio.sleep(
            MAP_SCAN_INTERVAL
        )

    confirmed_objects = (
        obstacle_map.get_confirmed_objects()
    )

    print()
    print(
        "Map scan complete."
    )

    print(
        f"Confirmed objects: "
        f"{len(confirmed_objects)}"
    )

    for obstacle_object in (
        confirmed_objects
    ):

        print(
            f"Object "
            f"{obstacle_object['id']} | "
            f"N="
            f"{obstacle_object['north_m']:.2f} | "
            f"E="
            f"{obstacle_object['east_m']:.2f} | "
            f"Obs="
            f"{obstacle_object['observations']}"
        )

    return confirmed_objects