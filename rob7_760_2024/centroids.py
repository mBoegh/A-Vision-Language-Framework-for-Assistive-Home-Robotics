import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN


class GetCentroids(Node):
    def __init__(self):
        super().__init__('get_centroids')

        # Subscribe to the 3D points topic from object_det_cloud
        self.point_sub = self.create_subscription(
            PointCloud2, '/transformed_points', self.transformed_points_callback, 10)

        # Subscribe to the obstacles cloud topic from SegmentationNode
        self.timestamp_sub = self.create_subscription(
            PointCloud2, '/cloud_obstacles', self.cloud_obstacles_callback, 10)

        # Publisher for the filtered points as PointCloud2
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/centroids', 10)

        # Lists to store points from the topics
        self.transformed_points = []
        self.cloud_obstacles = []

        self.get_logger().info("GetCentroids Node initialized.")

    def transformed_points_callback(self, msg):
        """
        Callback to handle transformed points.
        Extracts x, y, z, and label_id from the incoming PointCloud2 message.
        """
        self.transformed_points = list(pc2.read_points(
            msg, field_names=["x", "y", "z", "label"], skip_nans=True))
        self.get_logger().info(f"Received {len(self.transformed_points)} transformed points.")

    def cloud_obstacles_callback(self, msg):
        """
        Callback to handle cloud obstacles.
        Extracts x, y, and z from the incoming PointCloud2 message.
        """
        self.cloud_obstacles = list(pc2.read_points(
            msg, field_names=["x", "y", "z"], skip_nans=True))
        self.get_logger().info(f"Received {len(self.cloud_obstacles)} obstacle points.")

        # Once obstacles are received, process and publish the filtered points
        self.process_and_publish_centroids()

    def process_and_publish_centroids(self):
        """
        Process the points to compute centroids for subclusters based on label_id.
        """
        if not self.transformed_points or not self.cloud_obstacles:
            self.get_logger().warning("No data to process. Waiting for both topics.")
            return

        # Convert obstacle points to numpy array for efficient distance computation
        obstacle_array = np.array([[p[0], p[1], p[2]] for p in self.cloud_obstacles])

        # Distance threshold for filtering
        distance_threshold = 0.1  # in meters

        # Filter transformed points based on proximity to obstacles
        filtered_points = []
        for point in self.transformed_points:
            x, y, z, label_id = point
            distances = np.linalg.norm(obstacle_array - np.array([x, y, z]), axis=1)
            if np.any(distances < distance_threshold):
                filtered_points.append((x, y, z, label_id))

        self.get_logger().info(f"Filtered down to {len(filtered_points)} points near obstacles.")

        if not filtered_points:
            self.get_logger().warning("No points to cluster after filtering.")
            return

        # Perform clustering and compute centroids
        centroids = self.compute_centroids(filtered_points)

        # Merge centroids if they are too close
        centroids = self.merge_close_centroids(centroids)

        # Publish the centroids
        self.publish_centroids(centroids)

    def compute_centroids(self, filtered_points):
        """
        Perform DBSCAN clustering within each label group and compute centroids.

        Args:
            filtered_points (list): List of tuples [(x, y, z, label_id), ...]

        Returns:
            list: List of centroids [(x, y, z, label_id), ...].
        """
        # Group points by label_id
        clusters_by_label = {}
        for point in filtered_points:
            x, y, z, label_id = point
            clusters_by_label.setdefault(label_id, []).append((x, y, z))

        centroids = []

        # DBSCAN parameters
        eps = 0.1  # Maximum distance for clustering
        min_samples = 15  # Minimum points to form a cluster

        # Perform DBSCAN clustering and compute centroids for each label group
        for label_id, points in clusters_by_label.items():
            points_array = np.array(points)

            # Apply DBSCAN clustering
            dbscan = DBSCAN(eps=eps, min_samples=min_samples)
            labels = dbscan.fit_predict(points_array)

            # Organize points into subclusters
            subclusters = {}
            for idx, subcluster_id in enumerate(labels):
                if subcluster_id != -1:  # Ignore noise points
                    subclusters.setdefault(subcluster_id, []).append(points[idx])

            # Compute centroids for each subcluster
            for subcluster_id, subcluster_points in subclusters.items():
                subcluster_array = np.array(subcluster_points)
                centroid = np.mean(subcluster_array, axis=0)  # Mean of x, y, z
                centroids.append((*centroid, label_id))  # Add label_id to centroid

        return centroids

    def merge_close_centroids(self, centroids, merge_threshold=0.2, obstacle_threshold=0.2):
        """
        Merge centroids that are too close to each other based on a threshold.
        The merged centroid will only be added if it is near enough to any obstacle point.
        The non-merged centroids will be retained.

        Args:

def generate_launch_description():

    param = [{
            centroids (list): List of centroids [(x, y, z, label_id), ...].
            merge_threshold (float): Distance threshold for merging centroids (meters).
            obstacle_threshold (float): Distance threshold for checking if the merged centroid is near an obstacle.

        Returns:
            list: List of merged centroids [(x, y, z, label_id), ...].
        """
        merged_centroids = []
        # Convert obstacle points to numpy array for efficient distance computation
        obstacle_array = np.array([[p[0], p[1], p[2]] for p in self.cloud_obstacles])

        # Group centroids by label_id
        centroids_by_label = {}
        for centroid in centroids:
            x, y, z, label_id = centroid
            centroids_by_label.setdefault(label_id, []).append(np.array([x, y, z]))

        # Process each group of centroids with the same label
        for label_id, group_centroids in centroids_by_label.items():
            group_centroids = np.array(group_centroids)
            temp_merged = []

            while len(group_centroids) > 0:
                current = group_centroids[0]
                distances = np.linalg.norm(group_centroids - current, axis=1)

                # Find centroids within merge threshold
                close_indices = np.where(distances < merge_threshold)[0]
                close_centroids = group_centroids[close_indices]

                # Compute average (middle) centroid
                avg_centroid = np.mean(close_centroids, axis=0)

                # Check if the merged centroid is near any obstacle
                is_near_obstacle = np.any(np.linalg.norm(obstacle_array - avg_centroid, axis=1) < obstacle_threshold)

                # If the merged centroid is near an obstacle, keep the merged centroid
                if is_near_obstacle:
                    temp_merged.append((*avg_centroid, label_id))
                else:
                    # If the merged centroid is not near an obstacle, keep the original centroids
                    temp_merged.extend([(*c, label_id) for c in close_centroids])

                # Remove the centroids that were merged from the group
                group_centroids = np.delete(group_centroids, close_indices, axis=0)

            merged_centroids.extend(temp_merged)

        return merged_centroids

    def publish_centroids(self, centroids):
        """
        Publish the centroids as a PointCloud2 message.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # Replace with the appropriate frame

        # Define the fields for the new PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='label', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Serialize the centroids into a PointCloud2 message
        centroid_cloud = pc2.create_cloud(header, fields, centroids)

        # Publish the centroid point cloud
        self.point_cloud_pub.publish(centroid_cloud)
        self.get_logger().info("Published centroid point cloud.")


def main(args=None):
    rclpy.init(args=args)
    node = GetCentroids()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GetCentroids Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()