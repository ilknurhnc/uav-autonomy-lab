import math

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan


LIDAR_TOPIC = (
    "/world/vision_test/model/x500_lidar_2d_0/"
    "link/link/sensor/lidar_2d_v2/scan"
)


DISTANCE_JUMP_THRESHOLD = 1.0
INDEX_GAP_THRESHOLD = 2
MIN_CLUSTER_POINTS = 3


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
            {
                "distance": distance,
                "angle": angle,
                "index": i,
            }
        )

    if not valid_points:
        print("No obstacle detected.")
        return

    clusters = []
    current_cluster = [valid_points[0]]

    for point in valid_points[1:]:

        previous_point = current_cluster[-1]

        distance_difference = abs(
            point["distance"]
            - previous_point["distance"]
        )

        index_difference = (
            point["index"]
            - previous_point["index"]
        )

        same_cluster = (
            distance_difference <= DISTANCE_JUMP_THRESHOLD
            and
            index_difference <= INDEX_GAP_THRESHOLD
        )

        if same_cluster:
            current_cluster.append(point)

        else:

            if len(current_cluster) >= MIN_CLUSTER_POINTS:
                clusters.append(current_cluster)

            current_cluster = [point]

    if len(current_cluster) >= MIN_CLUSTER_POINTS:
        clusters.append(current_cluster)

    if not clusters:
        print("No obstacle clusters detected.")
        return

    print()
    print(
        f"Detected clusters: {len(clusters)}"
    )

    for cluster_index, cluster in enumerate(
        clusters,
        start=1
    ):

        average_distance = sum(
            point["distance"]
            for point in cluster
        ) / len(cluster)

        average_angle = sum(
            point["angle"]
            for point in cluster
        ) / len(cluster)

        average_angle_deg = math.degrees(
            average_angle
        )

        min_angle_deg = math.degrees(
            cluster[0]["angle"]
        )

        max_angle_deg = math.degrees(
            cluster[-1]["angle"]
        )

        print(
            f"Cluster {cluster_index}: "
            f"Distance={average_distance:.2f} m | "
            f"Center Angle={average_angle_deg:.1f} deg | "
            f"Angular Width="
            f"{min_angle_deg:.1f}..{max_angle_deg:.1f} deg | "
            f"Points={len(cluster)}"
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