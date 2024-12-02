import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import struct
import random
import math


class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')

        # Subscribe to the 3D points topic from SegmentationNode
        self.point_sub = self.create_subscription(
            Float32MultiArray, '/object_detected/middle_point', self.point_callback, 10)

        # Subscribe to the timestamp topic from SegmentationNode
        self.timestamp_sub = self.create_subscription(
            Header, '/object_detected/image_timestamp', self.timestamp_callback, 10)

        # Publisher for the transformed points as PointCloud2
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)

        # Create a TF buffer and listener to get the transforms
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Counter for processing every 4th message
        self.message_counter = 0

        # List to store transformed points and labels
        self.transformed_points = []
        self.label_map = []

        # Store the latest timestamp from the SegmentationNode
        self.latest_timestamp = None

        # Maximum percentage of points to keep (e.g., 50% of points)
        self.sampling_percentage = 0.5

    def timestamp_callback(self, msg):
        """Callback for the timestamp message."""
        self.latest_timestamp = msg.stamp
        self.get_logger().info(f"Received timestamp: {self.latest_timestamp}")

    def point_callback(self, msg):
        """
        Callback for the /object_detected/middle_point topic.
        Processes points and transforms them to the map frame.
        """
        # Only process every 4th message
        self.message_counter += 1
        if self.message_counter % 4 != 0:
            return

        if self.latest_timestamp is None:
            self.get_logger().warn("No timestamp received yet, skipping point processing.")
            return

        try:
            # Get the transform from the camera frame to the map frame using the latest timestamp
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'head_front_camera_rgb_optical_frame',  # Source frame
                self.latest_timestamp,  # Use the latest timestamp received from SegmentationNode
                timeout=rclpy.duration.Duration(seconds=0.000001)  # Adjust timeout to avoid delays
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f"Connectivity issue: {e}")
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"Extrapolation error: {e}")
            return

        # Parse the Float32MultiArray message
        points = self.parse_points_from_message(msg)

        # Reduce the number of points by random sampling before processing
        points = self.reduce_points(points)

        # Transform each point, filter by z, and store it
        for point in points:
            transformed_point = self.transform_point(point, transform)
            if transformed_point:  # Check if transformation succeeded
                self.transformed_points.append(transformed_point)  # Add point directly
                self.label_map.append(point['label'])  # Save the label separately

        # Publish the transformed points as PointCloud2
        self.publish_point_cloud()

    def parse_points_from_message(self, message):
        """
        Parse 3D points with labels from the Float32MultiArray message.
        """
        points = []
        data = message.data
        for i in range(0, len(data), 4):  # Each point is [x, y, z, label_id]
            x, y, z, label_id = data[i:i+4]
            points.append({'x': x, 'y': y, 'z': z, 'label': int(label_id)})
        return points


    def transform_point(self, point, transform):
        """
        Transforms a point from the camera frame to the map frame.
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'head_front_camera_rgb_optical_frame'
        point_stamped.point.x = point['x']
        point_stamped.point.y = point['y']
        point_stamped.point.z = point['z']

        try:
            transformed_point_stamped = do_transform_point(point_stamped, transform)
            return {
                'x': transformed_point_stamped.point.x,
                'y': transformed_point_stamped.point.y,
                'z': transformed_point_stamped.point.z,
            }
        except Exception as e:
            self.get_logger().error(f"Failed to transform point: {e}")
            return None

    def publish_point_cloud(self):
        """
        Publish transformed points as a PointCloud2 message for visualization in RViz2.
        Only publishes x, y, z coordinates.
        """
        if not self.transformed_points:
            return

        # Create a PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'map'  # Use the map frame for visualization

        # Define the PointField structure for x, y, z
        cloud_msg.height = 1
        cloud_msg.width = len(self.transformed_points)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # Each point consists of 3 floats (4 bytes each)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True

        # Serialize the point data
        cloud_data = []
        for point in self.transformed_points:
            cloud_data.append(struct.pack('fff', point['x'], point['y'], point['z']))
        cloud_msg.data = b''.join(cloud_data)

        # Publish the PointCloud2 message
        self.point_cloud_pub.publish(cloud_msg)
        self.get_logger().info(f"Published PointCloud2 with {len(self.transformed_points)} points")

    def reduce_points(self, points):
        """
        Reduces the number of points by randomly sampling from the given points.
        Returns a subset of points based on the sampling percentage.
        """
        num_points = len(points)
        num_points_to_keep = int(num_points * self.sampling_percentage)
        sampled_points = random.sample(points, num_points_to_keep)
        return sampled_points


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()