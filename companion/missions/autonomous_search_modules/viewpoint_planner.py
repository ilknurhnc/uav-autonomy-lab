import math


SAFE_DISTANCE = 7.0
SIDE_ANGLE = 65.0


def select_next_object(
    confirmed_objects,
    drone_north,
    drone_east,
    inspected_object_ids,
    unreachable_object_ids,
):
    selected_object = None
    selected_distance = None

    for obstacle_object in confirmed_objects:
        object_id = obstacle_object["id"]

        if object_id in inspected_object_ids:
            continue

        if object_id in unreachable_object_ids:
            continue

        north_offset = (
            obstacle_object["north_m"]
            - drone_north
        )

        east_offset = (
            obstacle_object["east_m"]
            - drone_east
        )

        distance = math.sqrt(
            north_offset ** 2
            +
            east_offset ** 2
        )

        if (
            selected_distance is None
            or distance < selected_distance
        ):
            selected_object = obstacle_object
            selected_distance = distance

    return (
        selected_object,
        selected_distance,
    )


def rotate_vector(
    north,
    east,
    angle_deg,
):
    angle_rad = math.radians(
        angle_deg
    )

    rotated_north = (
        north * math.cos(angle_rad)
        -
        east * math.sin(angle_rad)
    )

    rotated_east = (
        north * math.sin(angle_rad)
        +
        east * math.cos(angle_rad)
    )

    return (
        rotated_north,
        rotated_east,
    )


def generate_inspection_viewpoints(
    drone_north,
    drone_east,
    obstacle_north,
    obstacle_east,
):
    north_direction = (
        drone_north
        - obstacle_north
    )

    east_direction = (
        drone_east
        - obstacle_east
    )

    direction_length = math.sqrt(
        north_direction ** 2
        +
        east_direction ** 2
    )

    if direction_length < 0.01:
        north_direction = 1.0
        east_direction = 0.0
        direction_length = 1.0

    unit_north = (
        north_direction
        / direction_length
    )

    unit_east = (
        east_direction
        / direction_length
    )

    left_north, left_east = (
        rotate_vector(
            unit_north,
            unit_east,
            SIDE_ANGLE,
        )
    )

    right_north, right_east = (
        rotate_vector(
            unit_north,
            unit_east,
            -SIDE_ANGLE,
        )
    )

    viewpoints = [
        {
            "north_m":
                obstacle_north
                + unit_north
                * SAFE_DISTANCE,

            "east_m":
                obstacle_east
                + unit_east
                * SAFE_DISTANCE,
        },

        {
            "north_m":
                obstacle_north
                + left_north
                * SAFE_DISTANCE,

            "east_m":
                obstacle_east
                + left_east
                * SAFE_DISTANCE,
        },

        {
            "north_m":
                obstacle_north
                + right_north
                * SAFE_DISTANCE,

            "east_m":
                obstacle_east
                + right_east
                * SAFE_DISTANCE,
        },
    ]

    return viewpoints


def calculate_yaw_to_target(
    from_north,
    from_east,
    target_north,
    target_east,
):
    north_offset = (
        target_north
        - from_north
    )

    east_offset = (
        target_east
        - from_east
    )

    yaw_deg = math.degrees(
        math.atan2(
            east_offset,
            north_offset,
        )
    )

    return yaw_deg % 360.0