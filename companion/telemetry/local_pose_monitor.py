import asyncio

from mavsdk import System


async def print_local_position(drone):

    async for position_velocity in (
        drone.telemetry.position_velocity_ned()
    ):

        position = position_velocity.position

        print(
            f"North: {position.north_m:.2f} m | "
            f"East: {position.east_m:.2f} m | "
            f"Down: {position.down_m:.2f} m"
        )

        await asyncio.sleep(0.5)


async def print_yaw(drone):

    async for attitude in (
        drone.telemetry.attitude_euler()
    ):

        print(
            f"Yaw: {attitude.yaw_deg:.1f} deg"
        )

        await asyncio.sleep(0.5)


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

    print("Reading local pose...")

    position_task = asyncio.create_task(
        print_local_position(drone)
    )

    yaw_task = asyncio.create_task(
        print_yaw(drone)
    )

    await asyncio.gather(
        position_task,
        yaw_task
    )


if __name__ == "__main__":

    try:
        asyncio.run(run())

    except KeyboardInterrupt:
        print("\nLocal pose monitor stopped.")