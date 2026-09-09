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


async def track_target(
    drone,
    get_latest_frame,
):
    print()
    print("Tracking red target...")

    while True:
        frame = get_latest_frame()

        if frame is None:
            print(
                "Waiting for camera frame..."
            )

            await asyncio.sleep(
                0.1
            )

            continue

        target = detect_red_target(
            frame
        )

        if target is None:
            print(
                "Target lost during tracking."
            )

            return False

        frame_width = frame.shape[1]

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

        if abs(error_x) < 20:
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