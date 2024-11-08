#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import torch
import clip
from PIL import Image as pil_Image

class TiagoCustomController(Node):
    def __init__(self):
        super().__init__('tiago_custom_controller')
        
        self.bridge = CvBridge()

        # Timer to periodically publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Publisher for base control (velocity commands)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for arm control (joint positions)
        self.arm_pub = self.create_publisher(Float64, '/arm_1_joint/command', 10)
        
        # Subscription for the RGB image of the headcam
        self.rgb_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.rgb_callback, 10)

        # Subscription for the RGB image of the headcam
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/depth_registered/image_raw', self.depth_callback, 10)

        # Initial commands
        self.velocity_cmd = Twist()
        self.joint_position = Float64()


        ### CLIP ###

        # Set device for computation (GPU if available, otherwise CPU)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load the CLIP model and preprocessing function
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

        # Provide your labels (update this list with the labels you want to test against)
        self.labels = ["Man", "Woman", "Coffee maker", "Bench", "Kitchen sink", "Pringles can", "Box", "Other"]  # <-- replace with your custom labels
        self.text = clip.tokenize(self.labels).to(self.device)

        self.counter = 0

    def timer_callback(self):
        # Example control logic for base movement
        self.velocity_cmd.linear.x = 0.0  # Move forward
        self.velocity_cmd.angular.z = 0.5  # Turn slightly
        self.base_pub.publish(self.velocity_cmd)

        # Example control logic for arm joint position
        self.joint_position.data = 1.0  # Set desired joint position
        self.arm_pub.publish(self.joint_position)
        
        self.get_logger().debug('Published velocity and joint position commands')


    def rgb_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image (numpy array)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        cv.imshow("RGB", image)
        cv.waitKey(1)

        # Convert the OpenCV BGR image (numpy array) to a PIL Image
        pil_image = pil_Image.fromarray(cv.cvtColor(image, cv.COLOR_BGR2RGB))
        
        # Preprocess the image for CLIP model
        preprocessed_image = self.preprocess(pil_image).unsqueeze(0).to(self.device)
        
        # Perform the encoding and similarity computation
        if self.counter % 60 == 0:
            with torch.no_grad():
                image_features = self.model.encode_image(preprocessed_image)
                text_features = self.model.encode_text(self.text)
                
                logits_per_image, logits_per_text = self.model(preprocessed_image, self.text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()
                
            # Output the probabilities for each label
            self.get_logger().info(f"Label probs: {dict(zip(self.labels, probs[0]))}")
        self.counter += 1

    def depth_callback(self, msg):
        # Colorizes the data captured by the headmounted depth sensor and displays it, as it becomes available from the topic '/head_front_camera/depth_registered/image_raw'
        try:
            # Convert ROS Image message to OpenCV format (float32 depth values in meters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Set the min and max depth values for normalization (in meters)
            min_depth, max_depth = 0.5, 5.0  # Adjust based on your sensor's effective range

            # Clip the depth image within the min and max depth range
            depth_image = np.clip(depth_image, min_depth, max_depth)

            # Normalize depth to 0-255 for visualization
            depth_normalized = cv.normalize(depth_image, None, 0, 255, cv.NORM_MINMAX)
            depth_normalized = depth_normalized.astype(np.uint8)

            # Apply color map to create a heatmap effect
            heatmap = cv.applyColorMap(depth_normalized, cv.COLORMAP_JET)

            # Display the colorized heatmap image
            cv.imshow("Depth Heatmap", heatmap)
            cv.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    tiago_custom_controller = TiagoCustomController()
    rclpy.spin(tiago_custom_controller)
    tiago_custom_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()