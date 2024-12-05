from rob7_760_2024.LIB import JSON_Handler

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

import time


class ImageSegmentationNode(Node):
    def __init__(self):
        
        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'image_segmentation_node')
        self.logger = self.get_logger()

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")
        
        # Frame counters to process every 4th frame
        self.rgb_frame_counter = 0
        self.depth_frame_counter = 0

        # Subscriptions
        self.rgb_subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',  # Change to your RGB topic
            self.rgb_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/head_front_camera/depth_registered/image_raw',  # Change to your depth topic
            self.depth_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',  # Change to your CameraInfo topic
            self.camera_info_callback,
            10
        )

        # Publisher for 3D points
        self.point_pub = self.create_publisher(
            Float32MultiArray, 
            '/object_detected/middle_point', 
            10
        )

        # Publisher for timestamp
        self.timestamp_pub = self.create_publisher(
            Header, 
            '/object_detected/image_timestamp', 
            10
        )

        # Other initializations (e.g., YOLO model, etc.)
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.logger.info(f"Using device: {self.device}")
        self.model = YOLO("yolo11x-seg.pt")
        self.camera_matrix = None
        self.depth_image = None
        self.camera_info_received = False

        # Define label mapping
        self.label_mapping = {
            'person': 1.0,
            'couch': 2.0,
            'chair': 3.0,
            'tv': 4.0,
            'cup': 5.0,
            'sink': 6.0,
            'spoon': 7.0,
            'vase': 8.0,
            'refrigerator': 9.0,
            'dining table': 10.0,
            'sports ball': 11.0,
            'cell phone': 12.0,
            'bench': 13.0,
            'bed': 14.0
        }

    def rgb_callback(self, msg):
        """Callback for RGB Image messages."""
        self.rgb_frame_counter += 1
        
        # Process only the 4th RGB frame
        if self.rgb_frame_counter % 10 != 0:
            return  # Skip processing and publishing
        
        # Reset counter periodically to avoid overflow
        if self.rgb_frame_counter >= 10000:
            self.rgb_frame_counter = 0

        # Check if required data is available
        if not self.camera_info_received or self.depth_image is None:
            self.logger.warn("CameraInfo or Depth Image not received yet, skipping processing.")
            return
        
        # Convert the ROS Image message to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize rgb and depth image to 320x240
        #rgb_image = cv2.resize(rgb_image, (640, 480))
        depth_image = self.depth_image #cv2.resize(self.depth_image, (320, 240)) 

        # Segment the image and get masks with labels
        _, labeled_masks = self.segment_image(rgb_image)

        # Ensure masks are resized to match the depth image size
        #labeled_masks_resized = self.resize_masks_to_depth_size(labeled_masks, depth_image)

        # Find 3D positions in the segmented masks
        labeled_points_3d = self.find_3d_positions(labeled_masks, depth_image)

        # Publish the 3D points with labels
        self.publish_points(labeled_points_3d)

        # Publish the timestamp at the same time
        self.publish_timestamp(msg.header.stamp)

    def depth_callback(self, msg):
        """Callback for Depth Image messages."""
        self.depth_frame_counter += 1
        
        # Process only the 4th depth frame
        if self.depth_frame_counter % 10 != 0:
            return  # Skip processing the depth image
        
        # Reset counter periodically to avoid overflow
        if self.depth_frame_counter >= 10000:
            self.depth_frame_counter = 0

        # Convert the ROS Image message to OpenCV
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        """Callback for CameraInfo messages."""
        self.camera_matrix = np.array(msg.k).reshape((3, 3))  # Intrinsic camera matrix
        self.camera_info_received = True
        self.logger.info('Camera info received!')

    def segment_image(self, image):
        """Perform YOLO-based segmentation and return labeled masks."""
        # Use the original 640x480 image directly (no resizing)
        results = self.model.predict(image, device=self.device, task='segment')
        labeled_masks = {}

        for result in results:
            if result.masks is None:
                continue

            for mask, box in zip(result.masks.data, result.boxes):
                confidence = float(box.conf[0])
                label_index = int(box.cls[0])
                label = self.model.names[label_index]  # Use YOLO model's label for detected objects

                # Only process objects with confidence > 0.65 and in label mapping
                if confidence > 0.65 and label in self.label_mapping:
                    mask_resized = mask.cpu().numpy().astype(np.uint8)
                    labeled_masks[label] = mask_resized

        return image, labeled_masks

    def resize_masks_to_depth_size(self, labeled_masks, depth_image):
        """Resize the masks to match the depth image size."""
        height, width = depth_image.shape
        resized_masks = {}
        #for label, mask in labeled_masks.items():
            # Resize each mask to the depth image size (480x640)
            #resized_masks[label] = cv2.resize(mask, (640, 480), interpolation=cv2.INTER_NEAREST)
        return resized_masks

    def find_3d_positions(self, labeled_masks, depth_image):
        """Find the 3D positions of points in the segmented masks with labels."""
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
                        if depth_value == 0:
                            continue
                        z = depth_value
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        labeled_points_3d.append((x, y, z, label_id))

        return labeled_points_3d

    def publish_points(self, labeled_points_3d):
        """Publish the labeled 3D points."""
        point_array = Float32MultiArray()
        for point in labeled_points_3d:
            x, y, z, label_id = point
            point_array.data.extend([x, y, z, label_id])
        self.point_pub.publish(point_array)
        self.logger.info(f"Published {len(labeled_points_3d)} labeled 3D points")

    def publish_timestamp(self, timestamp):
        """Publish the image timestamp."""
        header = Header()
        header.stamp = timestamp
        self.timestamp_pub.publish(header)
        self.logger.info(f"Published timestamp: {timestamp}")


####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("Segmentation", "NODE_LOG_LEVEL")

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
    
    # Instance the main class
    image_segmentation_node = ImageSegmentationNode()

    # Begin looping the node
    rclpy.spin(image_segmentation_node)
    

if __name__ == "__main__":
    main()
    
    
    
    
    
    