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

    valid_ranges = []

    for i, distance in enumerate(ranges):
        if math.isinf(distance):
            continue

        if math.isnan(distance):
            continue

        if distance <= 0.0:
            continue

        angle = msg.angle_min + i * msg.angle_step

        valid_ranges.append(
            (
                distance,
                angle
            )
        )

    if not valid_ranges:
        print("No obstacle detected.")
        return

    closest_distance, closest_angle = min(
        valid_ranges,
        key=lambda item: item[0]
    )

    closest_angle_deg = math.degrees(
        closest_angle
    )

    print(
        f"Closest obstacle: "
        f"{closest_distance:.2f} m | "
        f"Angle: {closest_angle_deg:.1f} deg"
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