from rob7_760_2024.LIB import JSON_Handler

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import transformations

import math
from itertools import product
import time
import ast

class MainNode(Node):
    """
    This is the Main node of the ROS2 network.
    """

    def __init__(self, timer_period, goal_distance_threshold, init_sleep_duration):
        # Initializing parsed variables.
        self.TIMER_PERIOD = timer_period
        self.GOAL_DISTANCE_THRESHOLD = goal_distance_threshold
        self.INIT_SLEEP_DURATION = init_sleep_duration

        # Initializing the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'main_node')
        self.logger = self.get_logger()

        # Example logging to show node is active
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")

        self.trigger = False

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

        self.robot_and_goal_localized = False
        self.old_labels_to_visit = None
        self.new_labels_to_visit = None
        self.goal_position = None
        self.robot_dist_to_goal = None
        self.last_label_positions = None  # Initialize last_label_positions

        # Initialize robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        # Initializing a timer that periodically calls the timer_callback function.
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        ##########################
        ### Publishers ###########
        ##########################
        self.combine_pointcloud_bool_publisher = self.create_publisher(Bool, '/combine_pointcloud_bool', 10)
        self.combine_pointcloud_bool_msg = Bool()

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_pose_msg = PoseStamped()

        ##########################
        ### Subscribers ##########
        ##########################
        self.object_list_subscription = self.create_subscription(String, '/object_list', self.object_list_topic_callback, 10)
        self.PoseWithCovarianceStamped_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.PoseWithCovarianceStamped_callback, 10)

        # Publisher for the filtered points as PointCloud2
        self.centroids_subscriber = self.create_subscription(PointCloud2, '/centroids', self.centroids_callback, 10)
    
        self.trigger_subscriber = self.create_subscription(Bool, '/trigger', self.trigger_callback, 10)
        
        self.logger.fatal("Waiting for trigger.")


    #############################
    ### Callback Functions ######
    #############################

    def trigger_callback(self, msg):
        self.logger.debug(f"Received data '{msg.data}'")
        
        if msg.data == True:
            self.trigger = True

    def object_list_topic_callback(self, msg):
        self.logger.debug(f"Received data '{msg.data}'")
        raw_data = msg.data
        self.logger.debug(f"raw_data: {raw_data}")
        
        # Update old labels
        self.old_labels_to_visit = self.new_labels_to_visit
        
        try:
            if raw_data.startswith('[') and raw_data.endswith(']'):
                self.new_labels_to_visit = ast.literal_eval(raw_data)
            else:
                self.new_labels_to_visit = [label.strip() for label in raw_data.split(',')]
            self.logger.debug(f"Parsed labels: {self.new_labels_to_visit}")
        except (ValueError, SyntaxError) as e:
            self.logger.error(f"Error parsing labels: {e}")

    def PoseWithCovarianceStamped_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        self.logger.debug(f'Robot position - x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}')

    def centroids_callback(self, msg):
        self.centroids = list(pc2.read_points(
            msg, field_names=["x", "y", "z", "label"], skip_nans=True))
        self.logger.debug(f"Received {len(self.centroids)} labeled centroids: '{self.centroids}' -----------------------------------------------------------------------------------------------------------------------------------------------\n--------------------------------------------------------------------------------------------------------------\n---------------------------------------------------------------------------------------------------------\n--------------------------------------------------------------------------------------------------------------------------")


    ###########################
    ### Utility Functions #####
    ###########################

    def filter_items_by_label(self, items, label):
        return [item[:3] for item in items if item[3] == label]

    def euclidean_distance(self, coords1, coords2):
        return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(coords1, coords2)))

    def find_min_distance_path(self, items, labels):
        grouped_positions = [self.filter_items_by_label(items, label) for label in labels]
        
        min_distance = float('inf')
        best_combination = None
        
        for combination in product(*grouped_positions):
            total_distance = sum(
                self.euclidean_distance(combination[i], combination[i + 1])
                for i in range(len(combination) - 1)
            )
            if total_distance < min_distance:
                min_distance = total_distance
                best_combination = combination
        
        return min_distance, best_combination

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to Quaternion using tf2_ros.transformations
        q = transformations.quaternion_from_euler(roll, pitch, yaw)
        return q

    def publish_pose(self):
        yaw_degrees = math.atan2(self.goal_position[1] - self.robot_y, self.goal_position[0] - self.robot_x)
        yaw_radians = math.radians(yaw_degrees)
        
        quaternion = self.euler_to_quaternion(self, 0, 0, yaw_radians)
        
        self.goal_pose_msg.header.frame_id = 'map'
        self.goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.goal_pose_msg.pose.position.x = self.goal_position[0]
        self.goal_pose_msg.pose.position.y = self.goal_position[1]
        self.goal_pose_msg.pose.position.z = self.goal_position[2]

        self.goal_pose_msg.pose.orientation.x = quaternion[0]
        self.goal_pose_msg.pose.orientation.y = quaternion[1]
        self.goal_pose_msg.pose.orientation.z = quaternion[2]
        self.goal_pose_msg.pose.orientation.w = quaternion[3]
        
        self.goal_pose_publisher.publish(self.goal_pose_msg)
        self.logger.debug(f"""Published goal_pose_msg to /goal_pose topic:\n
                            x: {self.goal_pose_msg.pose.position.x}\n
                            y: {self.goal_pose_msg.pose.position.y}\n
                            z: {self.goal_pose_msg.pose.position.z}\n\n
                            yaw_degrees: {yaw_degrees}\n
                            yaw_radians: {yaw_radians}\n\n
                            x quaternion: {self.goal_pose_msg.pose.orientation.x}\n
                            y quaternion: {self.goal_pose_msg.pose.orientation.y}\n
                            z quaternion: {self.goal_pose_msg.pose.orientation.z}\n
                            w quaternion: {self.goal_pose_msg.pose.orientation.w}""")
        
    ##########################
    ### Timer Callback #######
    ##########################

    def timer_callback(self):
        if self.trigger:
            if None in [self.robot_x, self.robot_y, self.robot_z]:
                self.logger.error("Robot position is not available yet.")
                return
            
            if self.new_labels_to_visit is not None and self.new_labels_to_visit != self.old_labels_to_visit:
                reverse_mapping = {v: k for k, v in self.label_mapping.items()}
                labeled_values = [
                    (*value[:3], reverse_mapping.get(value[3], None)) for value in self.centroids
                ]

                self.logger.debug(f"Labeled values: {labeled_values}")

                missing_labels = [label for label in self.new_labels_to_visit if label not in self.label_mapping]
                if missing_labels:
                    self.logger.error(f"Missing labels in label mapping: {missing_labels}")
                    return

                shortest_distance, best_path = self.find_min_distance_path(labeled_values, self.new_labels_to_visit)

                if best_path is None:
                    self.logger.warning("No valid path found.")
                else:
                    self.logger.debug(f"Minimum total distance: {shortest_distance}")
                    self.logger.debug(f"Labels to visit: {self.new_labels_to_visit}")
                    self.logger.debug(f"Best positions for items: {best_path}")

                    self.last_label_positions = best_path[-1]
                    self.goal_position = best_path[-1]

                    self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y, self.robot_z], self.last_label_positions)
                    self.logger.info(f"Robot distance to goal '{self.goal_position}': {self.robot_dist_to_goal}")
                    
                    self.publish_pose()

                    self.robot_and_goal_localized = True


            # if self.robot_and_goal_localized:
            #     self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y, self.robot_z], self.last_label_positions)
            #     self.logger.info(f"Robot distance to goal '{self.goal_position}': {self.robot_dist_to_goal}")
            #     if self.robot_dist_to_goal is not None and self.robot_dist_to_goal < self.GOAL_DISTANCE_THRESHOLD:
            #         self.logger.info(f"Robot within goal position distance threshold: {self.robot_dist_to_goal}/{self.GOAL_DISTANCE_THRESHOLD}")
            #         # Simulate manipulation task or set new goal
            #         # Reset goal or set to home position if needed


def main():
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"
    
    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    TIMER_PERIOD = json_handler.get_subkey_value("MainNode", "TIMER_PERIOD")
    GOAL_DISTANCE_THRESHOLD = json_handler.get_subkey_value("MainNode", "GOAL_DISTANCE_THRESHOLD")
    INIT_SLEEP_DURATION = json_handler.get_subkey_value("MainNode", "INIT_SLEEP_DURATION")
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("MainNode", "NODE_LOG_LEVEL")

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
    rclpy.logging.set_logger_level("main_node", eval(NODE_LOG_LEVEL))
    
    # Instance the Main class
    main_node = MainNode(TIMER_PERIOD, GOAL_DISTANCE_THRESHOLD, INIT_SLEEP_DURATION)
    
    # Begin looping the node
    rclpy.spin(main_node)

if __name__ == "__main__":
    main()
