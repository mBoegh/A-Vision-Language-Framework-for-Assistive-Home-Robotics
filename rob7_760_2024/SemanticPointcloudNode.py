from rob7_760_2024.LIB import JSON_Handler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import sensor_msgs_py.point_cloud2 as pc2

import struct
import math
import random

class SemanticPointcloudNode(Node):
    def __init__(self):
        # Initializing parsed variables.


        # Initializing the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'semantic_pointcloud_node')
        self.logger = self.get_logger()

        # Example logging to show node is active
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")
        
        # Subscribe to the PointCloud2 topic from the SegmentationNode
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/object_detected/pointcloud', self.pointcloud_callback, 10)

        # Publisher for the transformed points as PointCloud2
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)

        # Create a TF buffer and listener to get the transforms
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=100))

        #sampling percentage to reduce the incoming pointcloud
        self.sampling_percentage = 0.1

        # List to store transformed points with labels
        self.transformed_points = []

    def pointcloud_callback(self, msg):
        """
        Callback for the /object_detected/pointcloud topic.
        Processes points and transforms them to the map frame.
        """
        # Get the timestamp from the PointCloud2 message
        msg_timestamp = msg.header.stamp
        
        # Wait for the transform to be available using the PointCloud2's timestamp
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        try:
            self.logger.debug(f"tf_buffer: '{self.tf_buffer}'")
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                msg.header.frame_id,  # Source frame
                rclpy.time.Time.from_msg(msg_timestamp),  # Timestamp from the message
                timeout=rclpy.duration.Duration(seconds=0.1)  # Adjust timeout as needed
            )

            transform_time = transform.header.stamp

            # Compute time difference using sec and nanosec
            time_diff_sec = transform_time.sec - msg_timestamp.sec
            time_diff_nsec = transform_time.nanosec - msg_timestamp.nanosec

            # If nanosec difference is negative, adjust the seconds
            if time_diff_nsec < 0:
                time_diff_sec -= 1
                time_diff_nsec += 1e9  # Adding one second worth of nanoseconds
                
            time_diff = time_diff_sec + time_diff_nsec / 1e9  # Time difference in seconds

            self.logger.debug(f"Using transform (time diff: {time_diff:.3f} seconds)")

            if time_diff > 0.05:  # If the time difference is greater than 0.1 seconds, skip processing
                self.logger.warn(f"Transform is too old ({time_diff:.3f} seconds), skipping point cloud processing.")
                return

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.logger.warn(f"Transform lookup failed: {e}. Skipping point cloud processing.")
            return
        
        # Extract points from the PointCloud2 message
        points = self.extract_points_from_pointcloud2(msg)

        # Reduce the number of points by filtering based on proximity
        points = self.reduce_points(points)

        # Transform each point and store it
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

    def extract_points_from_pointcloud2(self, msg):
        """
        Extracts points with labels from a PointCloud2 message.
        """
        points = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z', 'label'), skip_nans=True):
            x, y, z, label = point
            points.append({'x': x, 'y': y, 'z': z, 'label': int(label)})
        return points
    
    def transform_point(self, point, transform):
        """
        Transforms a point from the source frame to the target frame.
        """
        if transform is None:
            self.logger.error("Transform is None. Skipping point transformation.")
            return None

        try:
            # Check if transform contains NaN or Inf values
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            if any(math.isnan(val) or math.isinf(val) for val in [translation.x, translation.y, translation.z]):
                self.logger.error(f"Transform contains invalid translation values: {translation}")
                return None
            if any(math.isnan(val) or math.isinf(val) for val in [rotation.x, rotation.y, rotation.z, rotation.w]):
                self.logger.error(f"Transform contains invalid rotation values: {rotation}")
                return None

            point_stamped = PointStamped()
            point_stamped.header.frame_id = transform.header.frame_id
            point_stamped.point.x = float(point['x'])
            point_stamped.point.y = float(point['y'])
            point_stamped.point.z = float(point['z'])

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
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
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
        self.logger.fatal(f"Published PointCloud2 with {len(self.transformed_points)} points")

    def reduce_points(self, points):
        """
        Reduces the number of points by keeping only those that are not
        within a specified distance of another point.
        """
     
        num_points = len(points)
        num_points_to_keep = int(num_points * self.sampling_percentage)
        
        sampled_points = random.sample(points, num_points_to_keep)

        return sampled_points

    def is_point_too_close(self, point):
        """
        Check if the point is too close to any other point in the output point cloud.
        """

        distance_threshold = 0.01  # Minimum distance between points

        for f_point in self.transformed_points:

            dist = math.sqrt(
                (point['x'] - f_point['x']) ** 2 +
                (point['y'] - f_point['y']) ** 2 +
                (point['z'] - f_point['z']) ** 2  # Ensure valid z values
            )
            if dist < distance_threshold:
                return True  # The point is too close to an existing point

        return False

def main():
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"
    
    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("SemanticPointcloudNode", "NODE_LOG_LEVEL")

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
    rclpy.logging.set_logger_level("semantic_pointcloud_node", eval(NODE_LOG_LEVEL))
    
    # Instance the MapBuilerNode class
    semantic_pointcloud_node = SemanticPointcloudNode()
    
    # Begin looping the node
    rclpy.spin(semantic_pointcloud_node)

if __name__ == "__main__":
    main()
