#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TiagoCustomController(Node):
    def __init__(self):
        super().__init__('tiago_custom_controller')
        
        # Publisher for base control (velocity commands)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for arm control (joint positions)
        self.arm_pub = self.create_publisher(Float64, '/arm_1_joint/command', 10)
        
        # Timer to periodically publish commands
        self.timer = self.create_timer(0.1, self.control_callback)
        
        # Initial commands
        self.velocity_cmd = Twist()
        self.joint_position = Float64()

    def control_callback(self):
        # Example control logic for base movement
        self.velocity_cmd.linear.x = 0.1  # Move forward
        self.velocity_cmd.angular.z = 0.2  # Turn slightly
        self.base_pub.publish(self.velocity_cmd)

        # Example control logic for arm joint position
        self.joint_position.data = 1.0  # Set desired joint position
        self.arm_pub.publish(self.joint_position)
        
        self.get_logger().info('Published velocity and joint position commands')

def main(args=None):
    rclpy.init(args=args)
    tiago_custom_controller = TiagoCustomController()
    rclpy.spin(tiago_custom_controller)
    tiago_custom_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

