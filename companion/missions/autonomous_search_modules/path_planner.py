import math


OBSTACLE_CLEARANCE = 5.0

DETOUR_DISTANCE = 6.0


def distance_between(
    north_1,
    east_1,
    north_2,
    east_2,
):
    return math.sqrt(
        (north_2 - north_1) ** 2
        +
        (east_2 - east_1) ** 2
    )


def point_to_segment_distance(
    point_north,
    point_east,
    start_north,
    start_east,
    end_north,
    end_east,
):
    segment_north = (
        end_north - start_north
    )

    segment_east = (
        end_east - start_east
    )

    segment_length_squared = (
        segment_north ** 2
        +
        segment_east ** 2
    )

    if segment_length_squared == 0.0:

        return distance_between(
            point_north,
            point_east,
            start_north,
            start_east,
        )

    t = (
        (
            (point_north - start_north)
            * segment_north
        )
        +
        (
            (point_east - start_east)
            * segment_east
        )
    ) / segment_length_squared

    t = max(
        0.0,
        min(
            1.0,
            t,
        ),
    )

    closest_north = (
        start_north
        + t * segment_north
    )

    closest_east = (
        start_east
        + t * segment_east
    )

    return distance_between(
        point_north,
        point_east,
        closest_north,
        closest_east,
    )


def find_blocking_object(
    start_north,
    start_east,
    target_north,
    target_east,
    confirmed_objects,
    selected_object_id=None,
):
    blocking_object = None

    closest_to_path = None

    for obstacle_object in confirmed_objects:

        if (
            selected_object_id is not None
            and
            obstacle_object["id"]
            == selected_object_id
        ):
            continue

        distance_to_path = (
            point_to_segment_distance(
                obstacle_object["north_m"],
                obstacle_object["east_m"],
                start_north,
                start_east,
                target_north,
                target_east,
            )
        )

        if (
            distance_to_path
            < OBSTACLE_CLEARANCE
        ):

            if (
                closest_to_path is None
                or
                distance_to_path
                < closest_to_path
            ):
                closest_to_path = (
                    distance_to_path
                )

                blocking_object = (
                    obstacle_object
                )

    return blocking_object


def generate_detour_waypoint(
    start_north,
    start_east,
    target_north,
    target_east,
    blocking_object,
):
    path_north = (
        target_north - start_north
    )

    path_east = (
        target_east - start_east
    )

    path_length = math.sqrt(
        path_north ** 2
        +
        path_east ** 2
    )

    if path_length == 0.0:

        return None

    unit_north = (
        path_north / path_length
    )

    unit_east = (
        path_east / path_length
    )

    # Path'in soluna dik vektor.
    perpendicular_north = (
        -unit_east
    )

    perpendicular_east = (
        unit_north
    )

    detour_north = (
        blocking_object["north_m"]
        +
        perpendicular_north
        * DETOUR_DISTANCE
    )

    detour_east = (
        blocking_object["east_m"]
        +
        perpendicular_east
        * DETOUR_DISTANCE
    )

    return {
        "north_m": detour_north,
        "east_m": detour_east,
    }


def plan_route(
    start_north,
    start_east,
    target_north,
    target_east,
    confirmed_objects,
    selected_object_id=None,
):
    blocking_object = (
        find_blocking_object(
            start_north,
            start_east,
            target_north,
            target_east,
            confirmed_objects,
            selected_object_id,
        )
    )

    if blocking_object is None:

        return [
            {
                "north_m":
                    target_north,

                "east_m":
                    target_east,
            }
        ]

    detour = (
        generate_detour_waypoint(
            start_north,
            start_east,
            target_north,
            target_east,
            blocking_object,
        )
    )

    if detour is None:

        return [
            {
                "north_m":
                    target_north,

                "east_m":
                    target_east,
            }
        ]

    return [
        detour,
        {
            "north_m":
                target_north,

            "east_m":
                target_east,
        },
    ]