import asyncio

from mavsdk.offboard import VelocityBodyYawspeed

from companion.missions.autonomous_search_modules.target_detector import (
    detect_red_target,
)


SEARCH_HALF_ANGLE = 40.0

SEARCH_YAW_SPEED = 30.0

YAW_TOLERANCE = 4.0

SEARCH_LOOP_DELAY = 0.03


def normalize_yaw(yaw_deg):
    return yaw_deg % 360.0


def calculate_yaw_error(
    target_yaw,
    current_yaw,
):
    return (
        target_yaw
        - current_yaw
        + 180.0
    ) % 360.0 - 180.0


async def get_current_yaw(drone):
    async for attitude in (
        drone.telemetry.attitude_euler()
    ):
        return attitude.yaw_deg


async def stop_yaw(drone):
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,
            0.0,
            0.0,
            0.0,
        )
    )


def check_target(
    get_latest_frame,
):
    frame = get_latest_frame()

    if frame is None:
        return None

    return detect_red_target(
        frame
    )


async def rotate_and_search(
    drone,
    get_latest_frame,
    target_yaw,
):
    while True:

        target = check_target(
            get_latest_frame
        )

        if target is not None:

            await stop_yaw(
                drone
            )

            print()
            print(
                "RED TARGET ACQUIRED!"
            )

            print(
                f"X={target['center_x']} | "
                f"Y={target['center_y']} | "
                f"Area={target['area']:.0f}"
            )

            return target


        current_yaw = (
            await get_current_yaw(
                drone
            )
        )

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

            await stop_yaw(
                drone
            )

            return None


        if yaw_error > 0:
            yaw_speed = (
                SEARCH_YAW_SPEED
            )
        else:
            yaw_speed = (
                -SEARCH_YAW_SPEED
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
            SEARCH_LOOP_DELAY
        )


async def search_target(
    drone,
    selected_object,
    get_latest_frame,
    center_yaw,
):
    print()
    print(
        f"Searching around Object "
        f"{selected_object['id']}..."
    )

    print(
        f"Object direction: "
        f"{center_yaw:.1f} deg"
    )


    target = check_target(
        get_latest_frame
    )

    if target is not None:

        await stop_yaw(
            drone
        )

        print()
        print(
            "RED TARGET ALREADY VISIBLE!"
        )

        return target

    left_yaw = normalize_yaw(
        center_yaw
        - SEARCH_HALF_ANGLE
    )

    right_yaw = normalize_yaw(
        center_yaw
        + SEARCH_HALF_ANGLE
    )


    print()
    print(
        f"Fast scan to LEFT | "
        f"Target yaw="
        f"{left_yaw:.1f}"
    )

    target = await rotate_and_search(
        drone,
        get_latest_frame,
        left_yaw,
    )

    if target is not None:
        return target


    print()
    print(
        f"FAST SWEEP LEFT -> RIGHT | "
        f"Target yaw="
        f"{right_yaw:.1f}"
    )

    target = await rotate_and_search(
        drone,
        get_latest_frame,
        right_yaw,
    )

    if target is not None:
        return target


    print()
    print(
        "Returning to center..."
    )

    target = await rotate_and_search(
        drone,
        get_latest_frame,
        normalize_yaw(
            center_yaw
        ),
    )

    if target is not None:
        return target

    await stop_yaw(
        drone
    )

    print()
    print(
        "Target not found "
        "in this viewpoint."
    )

    return None