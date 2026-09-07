import asyncio

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
)


TAKEOFF_ALTITUDE = 2.5

TARGET_NORTH = 3.0
TARGET_EAST = 0.0

POSITION_TOLERANCE = 0.5


async def run():

    drone = System()

    print("Connecting to PX4...")

    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    async for state in drone.core.connection_state():

        if state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for vehicle to be ready...")

    async for health in drone.telemetry.health():

        if (
            health.is_global_position_ok
            and health.is_home_position_ok
        ):
            print("Vehicle is ready!")
            break

    await drone.action.set_takeoff_altitude(
        TAKEOFF_ALTITUDE
    )

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    async for position in drone.telemetry.position():

        altitude = position.relative_altitude_m

        print(
            f"Altitude: {altitude:.2f} m"
        )

        if altitude >= TAKEOFF_ALTITUDE * 0.90:
            print("Takeoff altitude reached!")
            break

    print("Reading current local position...")

    async for position_velocity in (
        drone.telemetry.position_velocity_ned()
    ):

        start_north = (
            position_velocity.position.north_m
        )

        start_east = (
            position_velocity.position.east_m
        )

        break

    target_north = (
        start_north + TARGET_NORTH
    )

    target_east = (
        start_east + TARGET_EAST
    )

    target_down = -TAKEOFF_ALTITUDE

    print(
        f"Target: "
        f"North={target_north:.2f} | "
        f"East={target_east:.2f}"
    )

    print("Preparing Offboard...")

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            start_north,
            start_east,
            target_down,
            0.0,
        )
    )

    try:

        await drone.offboard.start()

        print("Offboard started!")

    except OffboardError as error:

        print(
            f"Offboard start failed: {error}"
        )

        await drone.action.land()

        return

    print("Flying to target...")

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            target_north,
            target_east,
            target_down,
            0.0,
        )
    )

    async for position_velocity in (
        drone.telemetry.position_velocity_ned()
    ):

        position = position_velocity.position

        north_error = (
            target_north - position.north_m
        )

        east_error = (
            target_east - position.east_m
        )

        print(
            f"North={position.north_m:.2f} | "
            f"East={position.east_m:.2f} | "
            f"N error={north_error:.2f} | "
            f"E error={east_error:.2f}"
        )

        if (
            abs(north_error) < POSITION_TOLERANCE
            and
            abs(east_error) < POSITION_TOLERANCE
        ):

            print("Target position reached!")
            break

    print("Hovering...")

    await asyncio.sleep(3)

    try:
        await drone.offboard.stop()

    except OffboardError as error:
        print(
            f"Offboard stop failed: {error}"
        )

    print("Landing...")

    await drone.action.land()

    async for in_air in drone.telemetry.in_air():

        if not in_air:
            print("Landed!")
            break


if __name__ == "__main__":

    try:
        asyncio.run(run())

    except KeyboardInterrupt:
        print("\nMission stopped.")