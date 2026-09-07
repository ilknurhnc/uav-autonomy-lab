import math


def sensor_to_local_ned(
    x_sensor,
    y_sensor,
    drone_north,
    drone_east,
    yaw_deg,
):
    """
    Convert Gazebo/FLU-like LiDAR sensor coordinates
    to PX4 local NED coordinates.

    Sensor/body assumption:
        +X = forward
        +Y = left

    PX4 body FRD:
        +X = forward
        +Y = right

    Therefore:
        x_body = x_sensor
        y_body = -y_sensor
    """

    # Gazebo / FLU-like sensor frame
    #          ↓
    # PX4 FRD body frame
    x_body = x_sensor
    y_body = -y_sensor

    yaw_rad = math.radians(
        yaw_deg
    )

    # Rotate PX4 body-frame vector
    # into local NED frame.
    north_offset = (
        x_body * math.cos(yaw_rad)
        - y_body * math.sin(yaw_rad)
    )

    east_offset = (
        x_body * math.sin(yaw_rad)
        + y_body * math.cos(yaw_rad)
    )

    obstacle_north = (
        drone_north
        + north_offset
    )

    obstacle_east = (
        drone_east
        + east_offset
    )

    return {
        "north_m": obstacle_north,
        "east_m": obstacle_east,
    }