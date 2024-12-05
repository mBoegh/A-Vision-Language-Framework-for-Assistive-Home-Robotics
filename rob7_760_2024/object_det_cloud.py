from rob7_760_2024.LIB import JSON_Handler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import struct
import math
import sensor_msgs_py.point_cloud2 as pc2

class MapBuilder(Node):
    def __init__(self):
        
        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'map_builder')
        self.logger = self.get_logger()

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")

        # Subscribe to the 3D points topic from SegmentationNode
        self.point_sub = self.create_subscription(
            Float32MultiArray, '/object_detected/middle_point', self.point_callback, 10)

        # Subscribe to the timestamp topic from SegmentationNode
        self.timestamp_sub = self.create_subscription(
            Header, '/object_detected/image_timestamp', self.timestamp_callback, 10)

        # Publisher for the transformed points as PointCloud2
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)

        # Create a TF buffer and listener to get the transforms
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=100))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List to store transformed points with labels
        self.transformed_points = []

        # Store the latest timestamp from the SegmentationNode
        self.latest_timestamp = None  

    def timestamp_callback(self, msg):
        """Callback for the timestamp message."""
        self.latest_timestamp = msg.stamp
        self.logger.info(f"Received timestamp: {self.latest_timestamp}")

    def point_callback(self, msg):
        """
        Callback for the /object_detected/middle_point topic.
        Processes points and transforms them to the map frame.
        """
        if self.latest_timestamp is None:
            self.logger.warn("No timestamp received yet, skipping point processing.")
            return

        # Wait for the transform if it's not ready
        
        #put a while
        
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'head_front_camera_rgb_optical_frame',  # Source frame
                self.latest_timestamp,  # Use the latest timestamp received from SegmentationNode
                timeout=rclpy.duration.Duration(seconds=0.01)  # Adjust timeout as needed
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.logger.warn(f"Transform lookup failed: {e}. Retrying...")
            return  # Exit early if transform is unavailable

        # Parse the Float32MultiArray message
        points = self.parse_points_from_message(msg)

        # Reduce the number of points by filtering based on proximity
        points = self.reduce_points(points)

        # Transform each point, filter by z, and store it
        for point in points:
            transformed_point = self.transform_point(point, transform)
            if transformed_point:  # Check if transformation succeeded
                # Check if this point is too close to any point in the transformed points list
                if self.is_point_too_close(transformed_point):
                    continue  # Skip the point if it's too close to an existing point
                
                # Combine point and label in a single dictionary
                transformed_point['label'] = point['label']
                self.transformed_points.append(transformed_point)

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

    def is_valid_point(self, point):
        """
        Checks if a point has valid numerical values for x, y, and z.
        """
        return not any(math.isnan(point[dim]) for dim in ['x', 'y', 'z'])

    def transform_point(self, point, transform):
        """
        Transforms a point from the camera frame to the map frame.
        """
        if transform is None:
            self.logger.error("Transform is None. Skipping point transformation.")
            return None

        try:
            # Check for NaN values in translation and rotation explicitly
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            if any(math.isnan(v) for v in [translation.x, translation.y, translation.z]):
                self.logger.error(f"Transform translation contains NaN values: {translation}")
                return None

            if any(math.isnan(v) for v in [rotation.x, rotation.y, rotation.z, rotation.w]):
                self.logger.error(f"Transform rotation contains NaN values: {rotation}")
                return None

            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'head_front_camera_rgb_optical_frame'
            point_stamped.point.x = point['x']
            point_stamped.point.y = point['y']
            point_stamped.point.z = point['z']

            transformed_point_stamped = do_transform_point(point_stamped, transform)
            return {
                'x': transformed_point_stamped.point.x,
                'y': transformed_point_stamped.point.y,
                'z': transformed_point_stamped.point.z,
            }
        except Exception as e:
            self.logger.error(f"Failed to transform point: {e}")
            return None

    def publish_point_cloud(self):
        """
        Publish transformed points as a PointCloud2 message for visualization in RViz2.
        Publishes x, y, z coordinates and label_id.
        """
        if not self.transformed_points:
            return

        # Create a PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.latest_timestamp
        cloud_msg.header.frame_id = 'map'  # Use the map frame for visualization

        # Define the PointField structure for x, y, z, and label
        cloud_msg.height = 1
        cloud_msg.width = len(self.transformed_points)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='label', offset=12, datatype=PointField.UINT32, count=1),  # Add label field
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # Each point consists of 3 floats (x, y, z) and 1 uint32 (label)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True

        # Serialize the point data with labels
        cloud_data = []
        for point in self.transformed_points:
            cloud_data.append(struct.pack('fffI', point['x'], point['y'], point['z'], point['label']))
        cloud_msg.data = b''.join(cloud_data)

        # Publish the PointCloud2 message
        self.point_cloud_pub.publish(cloud_msg)
        self.logger.info(f"Published PointCloud2 with {len(self.transformed_points)} points")

    def reduce_points(self, points):
        """
        Reduces the number of points by keeping only those that are not
        within a specified distance of another point.
        """
        distance_threshold = 0.002  # Minimum distance between points
        filtered_points = []

        for point in points:
            if not self.is_valid_point(point):
                self.logger.warn(f"Invalid point detected and skipped: {point}")
                continue

            too_close = False
            for f_point in filtered_points:
                dist = math.sqrt(
                    (point['x'] - f_point['x']) ** 2 +
                    (point['y'] - f_point['y']) ** 2 +
                    (point['z'] - f_point['z']) ** 2
                )
                if dist < distance_threshold:
                    too_close = True
                    break
            if not too_close:
                filtered_points.append(point)

        return filtered_points

    def is_point_too_close(self, point):
        """
        Check if the point is too close to any other point in the output point cloud.
        """
        if not self.is_valid_point(point):
            self.logger.warn(f"Point contains invalid values and will be skipped: {point}")
            return True

        distance_threshold = 0.01  # Minimum distance between points

        for f_point in self.transformed_points:
            if not self.is_valid_point(f_point):
                self.logger.warn(f"Existing transformed point contains invalid values: {f_point}")
                continue

            dist = math.sqrt(
                (point['x'] - f_point['x']) ** 2 +
                (point['y'] - f_point['y']) ** 2 +
                (point['z'] - f_point['z']) ** 2
            )
            if dist < distance_threshold:
                return True  # The point is too close to an existing point

        return False


####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("Object_det_cloud", "NODE_LOG_LEVEL")

    # Initialize the rclpy library.
    rclpy.init()

    # Sets the logging level of importance. 
    # When setting, one is setting the lowest level of importance one is interested in logging.
    # Logging level is defined in settings.json.
    # Logging levels:
    # - DEBUG
    # - INFO
    # - WARNING
    # - ERROR
    # - FATAL
    # The eval method interprets a string as a command.
    rclpy.logging.set_logger_level("map_builder", eval(NODE_LOG_LEVEL))
    
    # Instance the main class
    map_builder = MapBuilder()

    # Begin looping the node
    rclpy.spin(map_builder)
    

if __name__ == "__main__":
    main()