import asyncio

from mavsdk import System

from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityBodyYawspeed,
)

from companion.autonomy.obstacle_map import (
    ObstacleMap,
)

from companion.missions.autonomous_search_modules.sensor_manager import (
    get_latest_frame,
    get_latest_obstacles,
    start_sensors,
)

from companion.missions.autonomous_search_modules.mapping_behavior import (
    build_confirmed_obstacle_map,
    get_local_pose,
)

from companion.missions.autonomous_search_modules.viewpoint_planner import (
    calculate_yaw_to_target,
    generate_inspection_viewpoints,
    select_next_object,
)

from companion.missions.autonomous_search_modules.navigation_behavior import (
    align_yaw,
    navigate_to_viewpoint,
)

from companion.missions.autonomous_search_modules.search_behavior import (
    search_target,
)

from companion.missions.autonomous_search_modules.tracking_behavior import (
    track_target,
)


TAKEOFF = "TAKEOFF"

BUILD_MAP = "BUILD_MAP"

SELECT_OBJECT = "SELECT_OBJECT"

MOVE_TO_VIEWPOINT = "MOVE_TO_VIEWPOINT"

SEARCH_TARGET = "SEARCH_TARGET"

NEXT_VIEWPOINT = "NEXT_VIEWPOINT"

TRACK = "TRACK"

CENTERED = "CENTERED"


TAKEOFF_ALTITUDE = 3.0

MAX_MAP_REFRESHES = 3


obstacle_map = ObstacleMap(
    merge_distance=4.0
)

inspected_object_ids = set()

unreachable_object_ids = set()


async def finish_with_rtl(
    drone,
    offboard_started,
):
    print()
    print(
        "Mission search complete."
    )

    if offboard_started:

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                0.0,
            )
        )

        await asyncio.sleep(
            0.3
        )

        try:
            await drone.offboard.stop()

        except OffboardError:
            pass

    print(
        "Returning to launch..."
    )

    await drone.action.return_to_launch()


async def run():
    print(
        "Autonomous search mission starting..."
    )

    state = TAKEOFF

    confirmed_objects = []

    selected_object = None

    inspection_viewpoints = []

    current_viewpoint_index = 0

    searched_viewpoints = 0

    offboard_started = False

    target_yaw = 0.0

    map_refresh_count = 0

    sensor_node = (
        start_sensors()
    )

    if sensor_node is None:
        return

    drone = System()

    print(
        "Connecting to PX4..."
    )

    await drone.connect(
        system_address=
            "udpin://0.0.0.0:14540"
    )

    async for connection_state in (
        drone.core.connection_state()
    ):

        if (
            connection_state.is_connected
        ):

            print(
                "Connected to PX4!"
            )

            break

    print(
        "Waiting for vehicle..."
    )

    async for health in (
        drone.telemetry.health()
    ):

        if (
            health.is_local_position_ok
            and
            health.is_home_position_ok
            and
            health.is_armable
        ):

            print(
                "Vehicle ready!"
            )

            break

    while True:

        print()
        print(
            f"Current state: "
            f"{state}"
        )

        # =====================================
        # TAKEOFF
        # =====================================

        if state == TAKEOFF:

            await drone.action.set_takeoff_altitude(
                TAKEOFF_ALTITUDE
            )

            print(
                "Arming..."
            )

            await drone.action.arm()

            print(
                "Taking off..."
            )

            await drone.action.takeoff()

            async for position in (
                drone.telemetry.position()
            ):

                altitude = (
                    position.relative_altitude_m
                )

                print(
                    f"Altitude: "
                    f"{altitude:.2f} m"
                )

                if (
                    altitude
                    >= TAKEOFF_ALTITUDE
                    * 0.90
                ):

                    print(
                        "Takeoff altitude reached!"
                    )

                    break

            state = BUILD_MAP

            continue

        # =====================================
        # BUILD / REFRESH MAP
        # =====================================

        if state == BUILD_MAP:

            confirmed_objects = (
                await build_confirmed_obstacle_map(
                    drone,
                    obstacle_map,
                    get_latest_obstacles,
                    keep_offboard_alive=
                        offboard_started,
                )
            )

            if not confirmed_objects:

                print(
                    "No confirmed objects."
                )

                await finish_with_rtl(
                    drone,
                    offboard_started,
                )

                return

            state = SELECT_OBJECT

            continue

        # =====================================
        # SELECT OBJECT
        # =====================================

        if state == SELECT_OBJECT:

            (
                drone_north,
                drone_east,
                yaw_deg,
            ) = await get_local_pose(
                drone
            )

            (
                selected_object,
                selected_distance,
            ) = select_next_object(
                confirmed_objects,
                drone_north,
                drone_east,
                inspected_object_ids,
                unreachable_object_ids,
            )

            if (
                selected_object is None
            ):

                map_refresh_count += 1

                print()
                print(
                    "No selectable object."
                )

                print(
                    f"Map refresh "
                    f"{map_refresh_count}/"
                    f"{MAX_MAP_REFRESHES}"
                )

                if (
                    map_refresh_count
                    > MAX_MAP_REFRESHES
                ):

                    await finish_with_rtl(
                        drone,
                        offboard_started,
                    )

                    return

                state = BUILD_MAP

                continue

            map_refresh_count = 0

            print()
            print(
                f"Selected Object "
                f"{selected_object['id']}"
            )

            print(
                f"Object position: "
                f"N="
                f"{selected_object['north_m']:.2f} | "
                f"E="
                f"{selected_object['east_m']:.2f} | "
                f"Distance="
                f"{selected_distance:.2f} m"
            )

            inspection_viewpoints = (
                generate_inspection_viewpoints(
                    drone_north,
                    drone_east,
                    selected_object[
                        "north_m"
                    ],
                    selected_object[
                        "east_m"
                    ],
                )
            )

            current_viewpoint_index = 0

            searched_viewpoints = 0

            print()
            print(
                "Generated viewpoints: "
                f"{len(inspection_viewpoints)}"
            )

            for (
                index,
                viewpoint,
            ) in enumerate(
                inspection_viewpoints,
                start=1,
            ):

                print(
                    f"Viewpoint "
                    f"{index}: "
                    f"N="
                    f"{viewpoint['north_m']:.2f} | "
                    f"E="
                    f"{viewpoint['east_m']:.2f}"
                )

            state = (
                MOVE_TO_VIEWPOINT
            )

            continue

        # =====================================
        # MOVE TO VIEWPOINT
        # =====================================

        if state == MOVE_TO_VIEWPOINT:

            viewpoint = (
                inspection_viewpoints[
                    current_viewpoint_index
                ]
            )

            target_north = (
                viewpoint["north_m"]
            )

            target_east = (
                viewpoint["east_m"]
            )

            print()
            print(
                f"Moving to viewpoint "
                f"{current_viewpoint_index + 1}/"
                f"{len(inspection_viewpoints)}"
            )

            # -------------------------------
            # Start Offboard once
            # -------------------------------

            if not offboard_started:

                (
                    drone_north,
                    drone_east,
                    current_yaw,
                ) = await get_local_pose(
                    drone
                )

                await drone.offboard.set_position_ned(
                    PositionNedYaw(
                        drone_north,
                        drone_east,
                        -TAKEOFF_ALTITUDE,
                        current_yaw,
                    )
                )

                try:

                    await drone.offboard.start()

                    offboard_started = True

                    print(
                        "Offboard started!"
                    )

                except OffboardError as error:

                    print(
                        f"Offboard failed: "
                        f"{error}"
                    )

                    await drone.action.return_to_launch()

                    return

            reached = (
                await navigate_to_viewpoint(
                    drone,
                    target_north,
                    target_east,
                    get_latest_obstacles,
                )
            )

            if not reached:

                print()
                print(
                    "Viewpoint BLOCKED."
                )

                state = NEXT_VIEWPOINT

                continue

            # -------------------------------
            # Face object
            # -------------------------------

            inspection_yaw = (
                calculate_yaw_to_target(
                    target_north,
                    target_east,
                    selected_object[
                        "north_m"
                    ],
                    selected_object[
                        "east_m"
                    ],
                )
            )

            print()
            print(
                "Turning toward object..."
            )

            await align_yaw(
                drone,
                inspection_yaw,
            )

            target_yaw = (
                inspection_yaw
            )

            state = SEARCH_TARGET

            continue

        # =====================================
        # SEARCH TARGET
        # =====================================

        if state == SEARCH_TARGET:

            searched_viewpoints += 1

            target = (
                await search_target(
                    drone,
                    selected_object,
                    get_latest_frame,
                    target_yaw,
                )
            )

            if target is not None:

                state = TRACK

            else:

                state = NEXT_VIEWPOINT

            continue

        # =====================================
        # NEXT VIEWPOINT
        # =====================================

        if state == NEXT_VIEWPOINT:

            current_viewpoint_index += 1

            if (
                current_viewpoint_index
                < len(
                    inspection_viewpoints
                )
            ):

                print()
                print(
                    f"Trying viewpoint "
                    f"{current_viewpoint_index + 1}/"
                    f"{len(inspection_viewpoints)}"
                )

                state = (
                    MOVE_TO_VIEWPOINT
                )

                continue

            object_id = (
                selected_object["id"]
            )

            if (
                searched_viewpoints > 0
            ):

                inspected_object_ids.add(
                    object_id
                )

                print()
                print(
                    f"Object "
                    f"{object_id} "
                    f"INSPECTED."
                )

            else:

                unreachable_object_ids.add(
                    object_id
                )

                print()
                print(
                    f"Object "
                    f"{object_id} "
                    f"UNREACHABLE."
                )

            state = SELECT_OBJECT

            continue

        # =====================================
        # TRACK
        # =====================================

        if state == TRACK:

            target_centered = (
                await track_target(
                    drone,
                    get_latest_frame,
                )
            )

            if target_centered:

                state = CENTERED

            else:

                state = SEARCH_TARGET

            continue

        # =====================================
        # CENTERED
        # =====================================

        if state == CENTERED:

            print()
            print(
                "MISSION SUCCESS!"
            )

            print(
                "Red target found "
                "and centered."
            )

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            )

            await asyncio.sleep(
                2.0
            )

            await finish_with_rtl(
                drone,
                offboard_started,
            )

            return


if __name__ == "__main__":
    asyncio.run(
        run()
    )