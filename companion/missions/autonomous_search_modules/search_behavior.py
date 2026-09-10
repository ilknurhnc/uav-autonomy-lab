import asyncio

from mavsdk.offboard import (
    VelocityBodyYawspeed,
)

from companion.missions.autonomous_search_modules.target_detector import (
    detect_red_target,
)


SEARCH_HALF_ANGLE = 35.0

MAX_YAW_SPEED = 15.0

YAW_TOLERANCE = 2.0


def normalize_yaw(
    yaw_deg,
):
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


async def get_current_yaw(
    drone,
):
    async for attitude in (
        drone.telemetry.attitude_euler()
    ):
        return attitude.yaw_deg


async def stop_yaw(
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

    left_yaw = normalize_yaw(
        center_yaw
        - SEARCH_HALF_ANGLE
    )

    right_yaw = normalize_yaw(
        center_yaw
        + SEARCH_HALF_ANGLE
    )

    scan_targets = [
        left_yaw,
        right_yaw,
        normalize_yaw(
            center_yaw
        ),
    ]

    for (
        scan_index,
        target_yaw,
    ) in enumerate(
        scan_targets,
        start=1,
    ):

        print()
        print(
            f"Sector scan "
            f"{scan_index}/"
            f"{len(scan_targets)} | "
            f"Target yaw="
            f"{target_yaw:.1f}"
        )

        while True:

            frame = (
                get_latest_frame()
            )

            target = (
                detect_red_target(
                    frame
                )
            )

            if target is not None:

                await stop_yaw(
                    drone
                )

                print()
                print(
                    "RED TARGET FOUND!"
                )

                print(
                    f"Target center: "
                    f"X="
                    f"{target['center_x']} | "
                    f"Y="
                    f"{target['center_y']} | "
                    f"Area="
                    f"{target['area']:.0f}"
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

                await asyncio.sleep(
                    0.3
                )

                frame = (
                    get_latest_frame()
                )

                target = (
                    detect_red_target(
                        frame
                    )
                )

                if target is not None:

                    print()
                    print(
                        "RED TARGET FOUND!"
                    )

                    print(
                        f"Target center: "
                        f"X="
                        f"{target['center_x']} | "
                        f"Y="
                        f"{target['center_y']} | "
                        f"Area="
                        f"{target['area']:.0f}"
                    )

                    return target

                break

            yaw_speed = (
                yaw_error * 0.8
            )

            yaw_speed = max(
                -MAX_YAW_SPEED,
                min(
                    MAX_YAW_SPEED,
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

    await stop_yaw(
        drone
    )

    print()
    print(
        "Target not found "
        "in this viewpoint."
    )

    return None