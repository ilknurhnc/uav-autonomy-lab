SAFE_DISTANCE = 5.0


def select_next_object(
    confirmed_objects,
    drone_north,
    drone_east,
    inspected_object_ids,
):
    selected_object = None
    selected_distance = None

    for obstacle_object in confirmed_objects:
        object_id = obstacle_object["id"]

        if object_id in inspected_object_ids:
            continue

        north_offset = (
            obstacle_object["north_m"]
            - drone_north
        )

        east_offset = (
            obstacle_object["east_m"]
            - drone_east
        )

        distance = (
            north_offset ** 2
            + east_offset ** 2
        ) ** 0.5

        if distance <= SAFE_DISTANCE:
            continue

        if (
            selected_distance is None
            or distance < selected_distance
        ):
            selected_object = obstacle_object
            selected_distance = distance

    return selected_object, selected_distance


def calculate_viewpoint(
    drone_north,
    drone_east,
    obstacle_north,
    obstacle_east,
    obstacle_distance,
):
    north_offset = (
        obstacle_north
        - drone_north
    )

    east_offset = (
        obstacle_east
        - drone_east
    )

    north_direction = (
        north_offset
        / obstacle_distance
    )

    east_direction = (
        east_offset
        / obstacle_distance
    )

    travel_distance = (
        obstacle_distance
        - SAFE_DISTANCE
    )

    target_north = (
        drone_north
        + north_direction
        * travel_distance
    )

    target_east = (
        drone_east
        + east_direction
        * travel_distance
    )

    return (
        target_north,
        target_east,
    )