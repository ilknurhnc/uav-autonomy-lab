import asyncio

from mavsdk.offboard import (
    VelocityBodyYawspeed,
)

from companion.missions.autonomous_search_modules.target_detector import (
    detect_red_target,
)


async def search_target(
    drone,
    selected_object,
    get_latest_frame,
):
    print()
    print(
        f"Searching around Object "
        f"{selected_object['id']}..."
    )

    search_steps = 0
    max_search_steps = 72

    while search_steps < max_search_steps:
        frame = get_latest_frame()

        target = detect_red_target(
            frame
        )

        if target is not None:
            print()
            print(
                "RED TARGET FOUND!"
            )

            print(
                f"Target center: "
                f"X={target['center_x']} | "
                f"Y={target['center_y']} | "
                f"Area={target['area']:.0f}"
            )

            return target

        print(
            f"Search step: "
            f"{search_steps + 1}/"
            f"{max_search_steps} | "
            f"Target not visible"
        )

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                10.0,
            )
        )

        await asyncio.sleep(
            0.5
        )

        search_steps += 1

    print()
    print(
        "Target not found "
        "around this object."
    )

    return None