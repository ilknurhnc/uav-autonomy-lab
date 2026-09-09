import asyncio

from mavsdk import System

from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityBodyYawspeed,
)

from companion.missions.autonomous_search_modules.sensor_manager import (
    get_latest_frame,
    get_latest_obstacles,
    start_sensors,
)

from companion.autonomy.obstacle_map import (
    ObstacleMap,
)

from companion.missions.autonomous_search_modules.mapping_behavior import (
    build_confirmed_obstacle_map,
    get_local_pose,
)

from companion.missions.autonomous_search_modules.viewpoint_planner import (
    calculate_viewpoint,
    select_next_object,
)

from companion.missions.autonomous_search_modules.search_behavior import (
    search_target,
)

from companion.missions.autonomous_search_modules.tracking_behavior import (
    track_target,
)


TAKEOFF_ALTITUDE = 2.5
POSITION_TOLERANCE = 0.5


TAKEOFF = "TAKEOFF"
BUILD_MAP = "BUILD_MAP"
SELECT_OBJECT = "SELECT_OBJECT"
MOVE_TO_VIEWPOINT = "MOVE_TO_VIEWPOINT"
SEARCH_TARGET = "SEARCH_TARGET"
NEXT_OBJECT = "NEXT_OBJECT"
TRACK = "TRACK"
CENTERED = "CENTERED"


obstacle_map = ObstacleMap(
    merge_distance=4.0
)

inspected_object_ids = set()

async def run():
    print("Autonomous search mission starting...")

    state = TAKEOFF

    sensor_node = start_sensors()

    if sensor_node is None:
        return

    drone = System()

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    async for connection_state in (
        drone.core.connection_state()
    ):
        if connection_state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for vehicle to be ready...")

    async for health in drone.telemetry.health():
        print(
            f"Local: {health.is_local_position_ok} | "
            f"Home: {health.is_home_position_ok} | "
            f"Armable: {health.is_armable}"
        )

        if (
            health.is_local_position_ok
            and health.is_home_position_ok
            and health.is_armable
        ):
            print("Vehicle ready!")
            break

    print()
    print(f"Current state: {state}")

    if state == TAKEOFF:
        await drone.action.set_takeoff_altitude(
            TAKEOFF_ALTITUDE
        )

        print("Arming...")
        await drone.action.arm()

        print("Taking off...")
        await drone.action.takeoff()

        async for position in (
            drone.telemetry.position()
        ):
            altitude = (
                position.relative_altitude_m
            )

            print(
                f"Altitude: {altitude:.2f} m"
            )

            if (
                altitude
                >= TAKEOFF_ALTITUDE * 0.90
            ):
                print(
                    "Takeoff altitude reached!"
                )
                break

        state = BUILD_MAP

        print()
        print(
            f"State transition: "
            f"TAKEOFF -> {state}"
        )

    if state == BUILD_MAP:
        print()
        print(
            f"Current state: {state}"
        )

        confirmed_objects = (
            await build_confirmed_obstacle_map(
                drone,
                obstacle_map,
                get_latest_obstacles,
            )
        )

        if not confirmed_objects:
            print(
                "No confirmed objects. "
                "Mission cannot continue."
            )

            await drone.action.land()
            return

        print()
        print(
            f"Confirmed objects available: "
            f"{len(confirmed_objects)}"
        )

        state = SELECT_OBJECT

        print()
        print(
            f"State transition: "
            f"BUILD_MAP -> {state}"
        )

    if state == SELECT_OBJECT:
        print()
        print(
            f"Current state: {state}"
        )

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(drone)

        (
            selected_object,
            selected_distance,
        ) = select_next_object(
            confirmed_objects,
            drone_north,
            drone_east,
            inspected_object_ids,
        )

        if selected_object is None:
            print(
                "No uninspected object available."
            )

            await drone.action.land()
            return

        print()
        print(
            f"Selected Object "
            f"{selected_object['id']}"
        )

        print(
            f"Object position: "
            f"N={selected_object['north_m']:.2f} | "
            f"E={selected_object['east_m']:.2f}"
        )

        print(
            f"Distance: "
            f"{selected_distance:.2f} m"
        )

        state = MOVE_TO_VIEWPOINT

        print()
        print(
            f"State transition: "
            f"SELECT_OBJECT -> {state}"
        )

    if state == MOVE_TO_VIEWPOINT:
        print()
        print(
            f"Current state: {state}"
        )

        (
            drone_north,
            drone_east,
            yaw_deg,
        ) = await get_local_pose(drone)

        obstacle_north = (
            selected_object["north_m"]
        )

        obstacle_east = (
            selected_object["east_m"]
        )

        north_offset = (
            obstacle_north
            - drone_north
        )

        east_offset = (
            obstacle_east
            - drone_east
        )

        obstacle_distance = (
            north_offset ** 2
            + east_offset ** 2
        ) ** 0.5

        (
            target_north,
            target_east,
        ) = calculate_viewpoint(
            drone_north,
            drone_east,
            obstacle_north,
            obstacle_east,
            obstacle_distance,
        )

        target_down = (
            -TAKEOFF_ALTITUDE
        )

        print(
            f"Viewpoint: "
            f"N={target_north:.2f} | "
            f"E={target_east:.2f}"
        )

        print(
            "Preparing Offboard..."
        )

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                drone_north,
                drone_east,
                target_down,
                yaw_deg,
            )
        )

        try:
            await drone.offboard.start()

            print(
                "Offboard started!"
            )

        except OffboardError as error:
            print(
                f"Offboard start failed: "
                f"{error}"
            )

            await drone.action.land()
            return

        print(
            f"Flying toward viewpoint "
            f"for Object "
            f"{selected_object['id']}..."
        )

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                target_north,
                target_east,
                target_down,
                yaw_deg,
            )
        )

        async for position_velocity in (
            drone.telemetry.position_velocity_ned()
        ):
            position = (
                position_velocity.position
            )

            north_error = (
                target_north
                - position.north_m
            )

            east_error = (
                target_east
                - position.east_m
            )

            print(
                f"N={position.north_m:.2f} | "
                f"E={position.east_m:.2f} | "
                f"N error={north_error:.2f} | "
                f"E error={east_error:.2f}"
            )

            if (
                abs(north_error)
                < POSITION_TOLERANCE
                and
                abs(east_error)
                < POSITION_TOLERANCE
            ):
                print(
                    "Viewpoint reached!"
                )
                break

        state = SEARCH_TARGET

        print()
        print(
            f"State transition: "
            f"MOVE_TO_VIEWPOINT -> {state}"
        )

    if state == SEARCH_TARGET:
        print()
        print(
            f"Current state: {state}"
        )

        target = await search_target(
            drone,
            selected_object,
            get_latest_frame,
        )

        if target is not None:
            state = TRACK

            print()
            print(
                f"State transition: "
                f"SEARCH_TARGET -> {state}"
            )

        else:
            inspected_object_ids.add(
                selected_object["id"]
            )

            state = NEXT_OBJECT

            print()
            print(
                f"Object "
                f"{selected_object['id']} "
                f"marked as inspected."
            )

            print(
                f"State transition: "
                f"SEARCH_TARGET -> {state}"
            )

    if state == TRACK:
        print()
        print(
            f"Current state: {state}"
        )

        target_centered = (
            await track_target(
                drone,
                get_latest_frame,
            )
        )

        if target_centered:
            state = CENTERED

            print()
            print(
                f"State transition: "
                f"TRACK -> {state}"
            )

        else:
            state = SEARCH_TARGET

            print()
            print(
                f"State transition: "
                f"TRACK -> {state}"
            )

    if state == CENTERED:
        print()
        print(
            f"Current state: {state}"
        )

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
            3.0
        )

        try:
            await drone.offboard.stop()

        except OffboardError:
            pass

        print(
            "Landing..."
        )

        await drone.action.land()

        return

    print()
    print(
        f"Current state: {state}"
    )


if __name__ == "__main__":
    asyncio.run(run())