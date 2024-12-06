from rob7_760_2024.LIB import JSON_Handler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import cv2
from ultralytics import YOLO
import torch
import struct


class ImageSegmentationNode(Node):
    def __init__(self):
        # Initializing parsed variables.

        # Initializing the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'image_segmentation_node')
        self.logger = self.get_logger()

        # Example logging to show node is active
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")
        
        # Frame counters to process every 10th frame (reduces computational load)
        self.rgb_frame_counter = 0
        self.depth_frame_counter = 0

        # Subscriptions for RGB, depth images, and camera info
        self.rgb_subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',  # Topic for raw RGB images
            self.rgb_callback,  # Callback function to process RGB images
            10  # Queue size
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/head_front_camera/depth_registered/image_raw',  # Topic for raw depth images
            self.depth_callback,  # Callback function to process depth images
            10  # Queue size
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',  # Topic for camera info (intrinsics)
            self.camera_info_callback,  # Callback to process camera intrinsics
            10  # Queue size
        )

        # Publisher for 3D points as a PointCloud2 message
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,  # Publish as PointCloud2 message
            '/object_detected/pointcloud',  # Topic for the point cloud
            10  # Queue size
        )

        # Initialize utilities
        self.bridge = CvBridge()  # For converting ROS Image messages to OpenCV
        self.device = "cuda" if torch.cuda.is_available() else "cpu"  # Select computation device
        self.get_logger().info(f"Using device: {self.device}")  # Log the chosen device
        self.model = YOLO("yolo11x-seg.pt")  # Load the YOLO model for segmentation
        self.camera_matrix = None  # Placeholder for camera intrinsic matrix
        self.depth_image = None  # Placeholder for the latest depth image
        self.camera_info_received = False  # Flag to ensure camera info is received

        # Mapping object labels to unique IDs for easier handling
        self.label_mapping = {
            'person': 1,
            'couch': 2,
            'chair': 3,
            'tv': 4,
            'cup': 5,
            'sink': 6,
            'spoon': 7,
            'vase': 8,
            'refrigerator': 9,
            'dining table': 10,
            'sports ball': 11,
            'cell phone': 12,
            'bench': 13,
            'bed': 14
        }

    def rgb_callback(self, msg):
        """Callback to process incoming RGB image messages."""
        self.rgb_frame_counter += 1  # Increment the frame counter

        # Skip processing for all but every 10th frame to reduce load
        if self.rgb_frame_counter % 30 != 0:
            return

        # Periodically reset the frame counter to avoid overflow
        if self.rgb_frame_counter >= 10000:
            self.rgb_frame_counter = 0

        # Ensure camera info and depth image are available
        if not self.camera_info_received or self.depth_image is None:
            self.get_logger().warn("CameraInfo or Depth Image not received yet, skipping processing.")
            return

        # Convert ROS RGB image message to OpenCV format
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Reference the latest depth image
        depth_image = self.depth_image

        # Perform segmentation to get labeled masks
        _, labeled_masks = self.segment_image(rgb_image)

        # Compute 3D positions for detected objects
        labeled_points_3d = self.find_3d_positions(labeled_masks, depth_image)

        # Publish the 3D points as a PointCloud2 message
        self.publish_pointcloud(labeled_points_3d, msg.header.stamp)

    def depth_callback(self, msg):
        """Callback to process incoming depth image messages."""
        self.depth_frame_counter += 1  # Increment the frame counter

        # Skip processing for all but every 10th frame to reduce load
        if self.depth_frame_counter % 30 != 0:
            return

        # Periodically reset the frame counter to avoid overflow
        if self.depth_frame_counter >= 10000:
            self.depth_frame_counter = 0

        # Convert ROS depth image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        """Callback to process incoming camera intrinsic parameters."""
        # Extract the camera intrinsic matrix from the CameraInfo message
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.camera_info_received = True  # Mark that camera info is received
        self.get_logger().info('Camera info received!')

    def segment_image(self, image):
        """Perform YOLO-based segmentation on the RGB image."""
        results = self.model.predict(image, device=self.device, task='segment')
        labeled_masks = {}

        for result in results:
            if result.masks is None:  # Skip if no masks are found
                continue

            # Process detected masks and bounding boxes
            for mask, box in zip(result.masks.data, result.boxes):
                confidence = float(box.conf[0])
                label_index = int(box.cls[0])
                label = self.model.names[label_index]

                # Include only objects with high confidence and relevant labels
                if confidence > 0.65 and label in self.label_mapping:
                    mask_resized = mask.cpu().numpy().astype(np.uint8)
                    labeled_masks[label] = mask_resized

        return image, labeled_masks

    def find_3d_positions(self, labeled_masks, depth_image):
        """Compute 3D positions of objects using depth data."""
        labeled_points_3d = []
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        for label, mask in labeled_masks.items():
            label_id = self.label_mapping[label]
            height, width = mask.shape
            for v in range(height):
                for u in range(width):
                    if mask[v, u]:
                        depth_value = depth_image[v, u]
                        if depth_value == 0:  # Skip invalid depth values
                            continue
                        z = depth_value
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        labeled_points_3d.append((x, y, z, label_id))

        return labeled_points_3d

    def publish_pointcloud(self, labeled_points_3d, timestamp):
        """Publish detected 3D points as a PointCloud2 message."""
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.stamp = timestamp
        pointcloud_msg.header.frame_id = 'head_front_camera_rgb_optical_frame'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='label', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pointcloud_msg.fields = fields

        pointcloud_data = [struct.pack('fffI', x, y, z, label_id) for x, y, z, label_id in labeled_points_3d]
        pointcloud_msg.data = b''.join(pointcloud_data)
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(labeled_points_3d)
        pointcloud_msg.is_dense = True
        pointcloud_msg.width = len(labeled_points_3d)
        pointcloud_msg.height = 1

        self.pointcloud_pub.publish(pointcloud_msg)
        self.get_logger().info(f"Published PointCloud2 with {len(labeled_points_3d)} points")




def main():
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"
    
    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("ImageSegmentationNode", "NODE_LOG_LEVEL")

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
    rclpy.logging.set_logger_level("image_segmentation_node", eval(NODE_LOG_LEVEL))
    
    # Instance the Main class
    image_segmentation_node = ImageSegmentationNode()
    
    # Begin looping the node
    rclpy.spin(image_segmentation_node)

if __name__ == "__main__":
    main()