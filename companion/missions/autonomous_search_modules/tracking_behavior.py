import asyncio

from mavsdk.offboard import (
    VelocityBodyYawspeed,
)

from companion.autonomy.motion_controller import (
    calculate_control_command,
    control_to_yaw_speed,
)

from companion.missions.autonomous_search_modules.target_detector import (
    detect_red_target,
)


MAX_MISSED_FRAMES = 5


async def track_target(
    drone,
    get_latest_frame,
):
    print()
    print(
        "Tracking red target..."
    )

    missed_frames = 0

    while True:

        frame = (
            get_latest_frame()
        )

        if frame is None:

            await asyncio.sleep(
                0.1
            )

            continue

        target = (
            detect_red_target(
                frame
            )
        )

        if target is None:

            missed_frames += 1

            print(
                f"Target temporarily lost "
                f"{missed_frames}/"
                f"{MAX_MISSED_FRAMES}"
            )

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )

            if (
                missed_frames
                >= MAX_MISSED_FRAMES
            ):
                print(
                    "Target lost."
                )

                return False

            await asyncio.sleep(
                0.1
            )

            continue

        missed_frames = 0

        frame_width = (
            frame.shape[1]
        )

        camera_center_x = (
            frame_width // 2
        )

        error_x = (
            target["center_x"]
            - camera_center_x
        )

        control_command = (
            calculate_control_command(
                error_x
            )
        )

        yaw_speed = (
            control_to_yaw_speed(
                control_command
            )
        )

        print(
            f"Target X="
            f"{target['center_x']} | "
            f"Center X="
            f"{camera_center_x} | "
            f"Error X="
            f"{error_x} | "
            f"Yaw="
            f"{yaw_speed:.2f}"
        )

        if (
            abs(error_x)
            < 20
        ):

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )

            print()
            print(
                "TARGET CENTERED!"
            )

            return True

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