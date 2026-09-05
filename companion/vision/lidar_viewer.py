import math

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan


LIDAR_TOPIC = (
    "/world/vision_test/model/x500_lidar_2d_0/"
    "link/link/sensor/lidar_2d_v2/scan"
)


def lidar_callback(msg: LaserScan):
    ranges = list(msg.ranges)

    if not ranges:
        return

    valid_points = []

    for i, distance in enumerate(ranges):

        if math.isinf(distance):
            continue

        if math.isnan(distance):
            continue

        if distance <= 0.0:
            continue

        angle = msg.angle_min + i * msg.angle_step

        valid_points.append(
            (
                distance,
                angle
            )
        )

    if not valid_points:
        print("No obstacle detected.")
        return

    closest_distance = min(
        point[0]
        for point in valid_points
    )

    DISTANCE_TOLERANCE = 0.5

    obstacle_points = [
        point
        for point in valid_points
        if point[0] <= closest_distance + DISTANCE_TOLERANCE
    ]

    average_distance = sum(
        point[0]
        for point in obstacle_points
    ) / len(obstacle_points)

    average_angle = sum(
        point[1]
        for point in obstacle_points
    ) / len(obstacle_points)

    average_angle_deg = math.degrees(
        average_angle
    )

    print(
        f"Obstacle center: "
        f"{average_distance:.2f} m | "
        f"Angle: {average_angle_deg:.1f} deg | "
        f"Points: {len(obstacle_points)}"
    )


def main():
    node = Node()

    print("Connecting to LiDAR...")

    success = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback
    )

    if not success:
        print("Failed to subscribe to LiDAR.")
        return

    print("LiDAR connected!")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            pass

    except KeyboardInterrupt:
        print("\nStopping LiDAR viewer.")


if __name__ == "__main__":
    main()