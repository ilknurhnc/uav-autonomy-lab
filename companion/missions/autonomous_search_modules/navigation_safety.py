FORWARD_HALF_ANGLE = 35.0

SLOWDOWN_DISTANCE = 5.0

EMERGENCY_STOP_DISTANCE = 3.0

YAW_TOLERANCE = 4.0


def calculate_yaw_error(
    target_yaw,
    current_yaw,
):
    return (
        target_yaw
        - current_yaw
        + 180.0
    ) % 360.0 - 180.0


def obstacle_overlaps_forward_sector(
    obstacle,
):
    min_angle = obstacle[
        "min_angle_deg"
    ]

    max_angle = obstacle[
        "max_angle_deg"
    ]

    return (
        min_angle
        <= FORWARD_HALF_ANGLE
        and
        max_angle
        >= -FORWARD_HALF_ANGLE
    )


def get_closest_forward_obstacle(
    obstacles,
):
    closest_obstacle = None
    closest_distance = None

    for obstacle in obstacles:

        if not obstacle_overlaps_forward_sector(
            obstacle
        ):
            continue

        distance = obstacle.get(
            "min_distance",
            obstacle["distance"],
        )

        if (
            closest_distance is None
            or distance < closest_distance
        ):
            closest_obstacle = obstacle
            closest_distance = distance

    return (
        closest_obstacle,
        closest_distance,
    )


def get_navigation_speed(
    obstacles,
    normal_speed,
    slow_speed,
):
    (
        closest_obstacle,
        closest_distance,
    ) = get_closest_forward_obstacle(
        obstacles
    )

    if closest_distance is None:
        return (
            True,
            normal_speed,
            None,
        )

    if (
        closest_distance
        <= EMERGENCY_STOP_DISTANCE
    ):
        return (
            False,
            0.0,
            closest_distance,
        )

    if (
        closest_distance
        <= SLOWDOWN_DISTANCE
    ):
        return (
            True,
            slow_speed,
            closest_distance,
        )

    return (
        True,
        normal_speed,
        closest_distance,
    )
