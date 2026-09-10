import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


TAKEOFF_ALTITUDE = 2.5
TARGET_DISTANCE_NORTH = 2.0


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
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Vehicle is ready!")
            break

    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    print("Waiting to reach target altitude...")

    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m

        print(f"Altitude: {altitude:.2f} m")

        if altitude >= TAKEOFF_ALTITUDE * 0.90:
            print("Target altitude reached!")
            break

    print("Preparing Offboard mode...")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    try:
        await drone.offboard.start()
        print("Offboard started!")

    except OffboardError as error:
        print(f"Offboard start failed: {error}")
        await drone.action.land()
        return

    print("Reading starting position...")

    async for position_ned in drone.telemetry.position_velocity_ned():
        start_north = position_ned.position.north_m
        start_east = position_ned.position.east_m
        break

    print(
        f"Start position: "
        f"North={start_north:.2f} m, "
        f"East={start_east:.2f} m"
    )

    print("Moving north...")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(1.0, 0.0, 0.0, 0.0)
    )

    async for position_ned in drone.telemetry.position_velocity_ned():
        current_north = position_ned.position.north_m

        distance_north = current_north - start_north

        print(f"Distance north: {distance_north:.2f} m")

        if distance_north >= TARGET_DISTANCE_NORTH:
            print("Target distance reached!")
            break

    print("Stopping...")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    print("Hovering for 2 seconds...")
    await asyncio.sleep(2)

    print("Stopping Offboard...")

    try:
        await drone.offboard.stop()

    except OffboardError as error:
        print(f"Offboard stop failed: {error}")

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