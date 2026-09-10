import asyncio
from mavsdk import System


TAKEOFF_ALTITUDE = 2.5


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

    print(f"Setting takeoff altitude to {TAKEOFF_ALTITUDE} meters...")
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

    print("Hovering for 5 seconds...")
    await asyncio.sleep(5)

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