import math


def sensor_to_local_ned(
    x_sensor,
    y_sensor,
    drone_north,
    drone_east,
    yaw_deg,
):

    yaw_rad = math.radians(yaw_deg)

    north_offset = (
        x_sensor * math.cos(yaw_rad)
        - y_sensor * math.sin(yaw_rad)
    )

    east_offset = (
        x_sensor * math.sin(yaw_rad)
        + y_sensor * math.cos(yaw_rad)
    )

    obstacle_north = (
        drone_north + north_offset
    )

    obstacle_east = (
        drone_east + east_offset
    )

    return {
        "north_m": obstacle_north,
        "east_m": obstacle_east,
    }