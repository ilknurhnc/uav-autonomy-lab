import asyncio
from mavsdk import System


async def run():
    drone = System()

    print("Connecting to PX4...")

    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for position data...")

    async for position in drone.telemetry.position():
        print(
            f"Latitude:  {position.latitude_deg:.6f}\n"
            f"Longitude: {position.longitude_deg:.6f}\n"
            f"Altitude:  {position.relative_altitude_m:.2f} m\n"
        )

        await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nTelemetry monitor stopped.")