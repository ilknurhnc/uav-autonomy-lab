import asyncio
import time

from mavsdk.offboard import (
    VelocityBodyYawspeed,
)

from companion.missions.autonomous_search_modules.navigation_safety import (
    YAW_TOLERANCE,
    calculate_yaw_error,
    get_navigation_speed,
)

from companion.missions.autonomous_search_modules.viewpoint_planner import (
    calculate_yaw_to_target,
)


NORMAL_SPEED = 1.2

SLOW_SPEED = 0.35

MAX_ALIGN_YAW_SPEED = 30.0

MAX_NAV_YAW_SPEED = 15.0

NAVIGATION_TIMEOUT = 45.0

POSITION_TOLERANCE = 1.0


async def get_navigation_pose(
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


async def stop_motion(
    drone,
):
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,
            0.0,
            0.0,
            0.0,
        )
    )


async def align_yaw(
    drone,
    target_yaw,
):
    print(
        f"Aligning yaw to "
        f"{target_yaw:.1f} deg..."
    )

    while True:

        async for attitude in (
            drone.telemetry.attitude_euler()
        ):
            current_yaw = (
                attitude.yaw_deg
            )

            break

        yaw_error = (
            calculate_yaw_error(
                target_yaw,
                current_yaw,
            )
        )

        if (
            abs(yaw_error)
            <= YAW_TOLERANCE
        ):
            await stop_motion(
                drone
            )

            print(
                "Yaw aligned!"
            )

            return

        yaw_speed = (
            yaw_error * 1.5
        )

        yaw_speed = max(
            -MAX_ALIGN_YAW_SPEED,
            min(
                MAX_ALIGN_YAW_SPEED,
                yaw_speed,
            ),
        )

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                yaw_speed,
            )
        )

        await asyncio.sleep(
            0.1
        )


async def navigate_to_viewpoint(
    drone,
    target_north,
    target_east,
    get_latest_obstacles,
):
    (
        drone_north,
        drone_east,
        current_yaw,
    ) = await get_navigation_pose(
        drone
    )

    initial_yaw = (
        calculate_yaw_to_target(
            drone_north,
            drone_east,
            target_north,
            target_east,
        )
    )

    await align_yaw(
        drone,
        initial_yaw,
    )

    await asyncio.sleep(
        0.2
    )

    obstacles = (
        get_latest_obstacles()
    )

    (
        path_safe,
        _,
        closest_distance,
    ) = get_navigation_speed(
        obstacles,
        NORMAL_SPEED,
        SLOW_SPEED,
    )

    if not path_safe:

        print()
        print(
            "Viewpoint path is blocked "
            "before movement."
        )

        print(
            f"Obstacle distance: "
            f"{closest_distance:.2f} m"
        )

        await stop_motion(
            drone
        )

        return False

    start_time = (
        time.monotonic()
    )

    print()
    print(
        "Controlled flight started."
    )

    while True:

        if (
            time.monotonic()
            - start_time
            > NAVIGATION_TIMEOUT
        ):

            print()
            print(
                "Navigation timeout."
            )

            await stop_motion(
                drone
            )

            return False

        (
            drone_north,
            drone_east,
            current_yaw,
        ) = await get_navigation_pose(
            drone
        )

        north_error = (
            target_north
            - drone_north
        )

        east_error = (
            target_east
            - drone_east
        )

        distance_error = (
            north_error ** 2
            +
            east_error ** 2
        ) ** 0.5

        if (
            distance_error
            <= POSITION_TOLERANCE
        ):

            await stop_motion(
                drone
            )

            print()
            print(
                "Viewpoint reached!"
            )

            return True

        desired_yaw = (
            calculate_yaw_to_target(
                drone_north,
                drone_east,
                target_north,
                target_east,
            )
        )

        yaw_error = (
            calculate_yaw_error(
                desired_yaw,
                current_yaw,
            )
        )

        obstacles = (
            get_latest_obstacles()
        )

        (
            path_safe,
            forward_speed,
            closest_distance,
        ) = get_navigation_speed(
            obstacles,
            NORMAL_SPEED,
            SLOW_SPEED,
        )

        if not path_safe:

            print()
            print(
                "!!! OBSTACLE TOO CLOSE !!!"
            )

            print(
                f"Distance: "
                f"{closest_distance:.2f} m"
            )

            print(
                "Stopping navigation."
            )

            await stop_motion(
                drone
            )

            return False

        # Drone yönünü ciddi şekilde kaybettiyse
        # ileri gitmek yerine önce dönüyoruz.
        if (
            abs(yaw_error)
            > 20.0
        ):
            forward_speed = 0.0

        yaw_speed = (
            yaw_error * 0.8
        )

        yaw_speed = max(
            -MAX_NAV_YAW_SPEED,
            min(
                MAX_NAV_YAW_SPEED,
                yaw_speed,
            ),
        )

        if (
            distance_error
            < 2.5
        ):
            forward_speed = min(
                forward_speed,
                0.45,
            )

        if (
            closest_distance
            is None
        ):
            safety_text = "clear"

        else:
            safety_text = (
                f"{closest_distance:.2f} m"
            )

        print(
            f"N={drone_north:.2f} | "
            f"E={drone_east:.2f} | "
            f"Target distance="
            f"{distance_error:.2f} m | "
            f"Speed="
            f"{forward_speed:.2f} | "
            f"Safety="
            f"{safety_text}"
        )

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_speed,
                0.0,
                0.0,
                yaw_speed,
            )
        )

        await asyncio.sleep(
            0.1
        )