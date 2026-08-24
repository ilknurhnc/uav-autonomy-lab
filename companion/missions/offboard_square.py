import asyncio
from companion.autonomy.motion_controller import move

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


TAKEOFF_ALTITUDE = 2.5
TARGET_DISTANCE = 2.0
MOVE_SPEED = 1.0


async def wait_until_ready(drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            return


async def wait_until_altitude(drone, target_altitude):
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m

        print(f"Altitude: {altitude:.2f} m")

        if altitude >= target_altitude * 0.90:
            return


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

    print("Waiting for vehicle...")
    await wait_until_ready(drone)

    print("Vehicle ready!")

    await drone.action.set_takeoff_altitude(
        TAKEOFF_ALTITUDE
    )

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    await wait_until_altitude(
        drone,
        TAKEOFF_ALTITUDE
    )

    print("Preparing Offboard...")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            0.0,
            0.0,
            0.0,
            0.0
        )
    )

    try:
        await drone.offboard.start()
        print("Offboard started!")

    except OffboardError as error:
        print(f"Offboard start failed: {error}")
        await drone.action.land()
        return

    print("\n1 - Moving North")
    await move(
        drone,
        north_speed=MOVE_SPEED,
        east_speed=0.0,
        distance=TARGET_DISTANCE,
        yaw=0.0
    )

    print("\n2 - Moving East")
    await move(
        drone,
        north_speed=0.0,
        east_speed=MOVE_SPEED,
        distance=TARGET_DISTANCE,
        yaw=90.0
    )

    print("\n3 - Moving South")
    await move(
        drone,
        north_speed=-MOVE_SPEED,
        east_speed=0.0,
        distance=TARGET_DISTANCE,
        yaw=180.0
    )

    print("\n4 - Moving West")
    await move(
        drone,
        north_speed=0.0,
        east_speed=-MOVE_SPEED,
        distance=TARGET_DISTANCE,
        yaw=270.0
    )

    print("Square completed!")

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