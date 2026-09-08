import math


MERGE_DISTANCE = 2.5
MIN_CONFIRMATION_OBSERVATIONS = 3


def calculate_distance(
    north_1,
    east_1,
    north_2,
    east_2,
):
    north_difference = north_2 - north_1

    east_difference = east_2 - east_1

    return math.sqrt(north_difference ** 2 + east_difference ** 2)


class ObstacleMap:

    def __init__(
        self,
        merge_distance=MERGE_DISTANCE,
    ):

        self.merge_distance = (
            merge_distance
        )

        self.objects = []

        self.next_object_id = 1


    def add_observation(
        self,
        north,
        east,
    ):

        closest_object = None
        closest_distance = None

        for obstacle_object in self.objects:

            distance = calculate_distance(
                obstacle_object["north_m"],
                obstacle_object["east_m"],
                north,
                east,
            )

            if (
                closest_distance is None
                or
                distance < closest_distance
            ):

                closest_distance = distance
                closest_object = obstacle_object


        if (
            closest_object is not None
            and
            closest_distance
            <= self.merge_distance
        ):

            observation_count = (
                closest_object[
                    "observations"
                ]
            )

            new_count = (
                observation_count + 1
            )

            closest_object["north_m"] = (
                (
                    closest_object["north_m"]
                    * observation_count
                )
                + north
            ) / new_count

            closest_object["east_m"] = (
                (
                    closest_object["east_m"]
                    * observation_count
                )
                + east
            ) / new_count

            closest_object["observations"] = new_count

            if (
                new_count
                >= MIN_CONFIRMATION_OBSERVATIONS
            ):
                closest_object["confirmed"] = True

            return (
                closest_object["id"]
            )


        new_object = {
            "id": self.next_object_id,
            "north_m": north,
            "east_m": east,
            "observations": 1,
            "confirmed": False,
        }

        self.objects.append(
            new_object
        )

        self.next_object_id += 1

        return new_object["id"]


    def get_objects(self):

        return [
            obstacle_object.copy()
            for obstacle_object
            in self.objects
        ]

    def get_confirmed_objects(self):

        return [
            obstacle_object.copy()
            for obstacle_object
            in self.objects
            if obstacle_object["confirmed"]
        ]

    def reset(self):

        self.objects.clear()

        self.next_object_id = 1