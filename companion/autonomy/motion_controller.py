import asyncio

from mavsdk.offboard import VelocityNedYaw


async def get_local_position(drone):
    async for position_ned in drone.telemetry.position_velocity_ned():
        north = position_ned.position.north_m
        east = position_ned.position.east_m

        return north, east


async def stop_motion(drone, yaw):
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            0.0,
            0.0,
            0.0,
            yaw
        )
    )


async def move(
    drone,
    north_speed,
    east_speed,
    distance,
    yaw
):
    start_north, start_east = await get_local_position(drone)

    print(
        f"Start position -> "
        f"North: {start_north:.2f}, "
        f"East: {start_east:.2f}"
    )

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_speed,
            east_speed,
            0.0,
            yaw
        )
    )

    async for position_ned in drone.telemetry.position_velocity_ned():
        current_north = position_ned.position.north_m
        current_east = position_ned.position.east_m

        north_difference = current_north - start_north
        east_difference = current_east - start_east

        travelled_distance = (
            north_difference ** 2
            + east_difference ** 2
        ) ** 0.5

        print(f"Travelled: {travelled_distance:.2f} m")

        if travelled_distance >= distance:
            break

    await stop_motion(drone, yaw)

    print("Target reached. Hovering...")
    await asyncio.sleep(1)