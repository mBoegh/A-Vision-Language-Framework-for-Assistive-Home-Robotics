from rob7_760_2024.LIB import JSON_Handler
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from itertools import product
import time
import ast

class Main(Node):
    """
    This is the Main node of the ROS2 network.
    """

    def __init__(self, timer_period, goal_distance_threshold, init_sleep_duration):

        # Initialising parsed variables.
        self.TIMER_PERIOD = timer_period
        self.GOAL_DISTANCE_THRESHOLD = goal_distance_threshold
        self.INIT_SLEEP_DURATION = init_sleep_duration

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'.
        Node.__init__(self, 'main')
        self.logger = self.get_logger()

        # Example logging to show node is active
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")

        # Example N-dimensional data
        self.values = [
            [3.0, 1.0, 1.0, 2.0],  # chair
            [5.0, 2.0, 3.0, 4.0],  # cup
            [5.0, 4.0, 6.0, 3.0],  # cup
            [6.0, 7.0, 9.0, 5.0],  # sink
            [7.0, 8.0, 8.0, 9.0],  # spoon
            [7.0, 20.0, 16.0, 3.0],  # spoon
            [9.0, 3.0, 3.0, 7.0],  # refrigerator
            [6.0, 2.0, 5.0, 4.0],  # sink
            [8.0, 3.0, 4.0, 6.0],  # vase
        ]

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

        # # Initialize robot position
        # self.robot_x = None
        # self.robot_y = None
        # self.robot_z = None

        # Initialize robot position
        self.robot_x = 5.0
        self.robot_y = 21.0
        self.robot_z = 10.0


        # Initialising a timer that periodically calls the timer_callback function.
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        ##########################
        ### Publishers ###########
        ##########################
        
        self.trigger_publisher = self.create_publisher(Bool, '/trigger', 10)
        self.trigger_bool_msg = Bool()
        
        self.combine_pointcloud_bool_publisher = self.create_publisher(Bool, '/combine_pointcloud_bool', 10)
        self.combine_pointcloud_bool_msg = Bool()

        ##########################
        ### Subscribers ##########
        ##########################
        # Object list subscriber
        self.object_list_subscription = self.create_subscription(String, '/object_list', self.object_list_topic_callback, 10)
        
        # Point cloud subscriber
        self.point_cloud_subscription = self.create_subscription(PointCloud2, '/transformed_points', self.transformed_points_topic_callback, 10)
        
        # Localization pose subscriber
        self.PoseWithCovarianceStamped_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.PoseWithCovarianceStamped_callback, 10)

        self.logger.debug(f"Sleeping for duration '{self.INIT_SLEEP_DURATION}'")
        time.sleep(self.INIT_SLEEP_DURATION)
        self.logger.debug("Woke up.")
        self.trigger_bool_msg.data = True
        self.logger.debug(f"Loaded trigger_bool_msg with data: '{self.trigger_bool_msg.data}'")
        self.trigger_publisher.publish(self.trigger_bool_msg)
        self.logger.debug(f"Published trigger_bool_msg using trigger_bool_publisher.")


    #############################
    ### Callback Functions ######
    #############################

    def object_list_topic_callback(self, msg):
        """
        Callback function called whenever a message is received on the '/object_list' topic.
        """
        self.logger.debug(f"Received data '{msg.data}'")
        
        raw_data = msg.data
        
        self.logger.debug(f"raw_data: {raw_data}")
        
        # Update old labels
        self.old_labels_to_visit = self.new_labels_to_visit
        
        try:
            # Check the format of msg.data
            if raw_data.startswith('[') and raw_data.endswith(']'):
                # Evaluate it as a literal list
                self.new_labels_to_visit = ast.literal_eval(raw_data)
            else:
                # Manually split the data if it's just a comma-separated string
                self.new_labels_to_visit = [label.strip() for label in raw_data.split(',')]
            
            # Debugging to show the parsed labels
            self.logger.debug(f"Parsed labels: {self.new_labels_to_visit}")
            
        except (ValueError, SyntaxError) as e:
            self.logger.error(f"Error parsing labels: {e}")

    def transformed_points_topic_callback(self, msg):
        """
        Callback function called whenever a message is received on the '/transformed_points' topic.
        """
        self.logger.debug(f"Received data '{msg.data}'")

    def PoseWithCovarianceStamped_callback(self, msg):
        '''
        Callback function called whenever a message is received on the '/localization_pose' topic.
        '''
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        self.get_logger().debug(f'Robot position - x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}')

    ###########################
    ### Utility Functions #####
    ###########################

    # Filter items by specific labels
    def filter_items_by_label(self, items, label):
        return [item[1:] for item in items if item[0] == label]

    # Compute Euclidean distance in N-dimensional space
    def euclidean_distance(self, coords1, coords2):
        return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(coords1, coords2)))

    # Find the minimum total distance for a sequence of items
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

    ##########################
    ### Timer Callback #######
    ##########################

    def timer_callback(self):
        '''
        The callback function called during each period of the timer.
        '''
        
        if None in [self.robot_x, self.robot_y, self.robot_z]:
            self.logger.error("Robot position is not available yet.")
            return
        
        if self.new_labels_to_visit is not None and self.new_labels_to_visit != self.old_labels_to_visit:
            reverse_mapping = {v: k for k, v in self.label_mapping.items()}
            labeled_values = [
                (reverse_mapping.get(value[0], None), *value[1:])
                for value in self.values
            ]

            self.logger.debug(f"Labeled values: {labeled_values}")

            missing_labels = [label for label in self.new_labels_to_visit if label not in self.label_mapping]  # Fix here
            if missing_labels:
                self.logger.error(f"Missing labels in label mapping: {missing_labels}")
                return

            shortest_distance, best_path = self.find_min_distance_path(labeled_values, self.new_labels_to_visit)  # Fix here

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
                
                self.robot_and_goal_localized = True

            

        if self.robot_and_goal_localized:
            self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y, self.robot_z], self.last_label_positions)
            self.logger.info(f"Robot distance to goal '{self.goal_position}': {self.robot_dist_to_goal}")
            if self.robot_dist_to_goal is not None and self.robot_dist_to_goal < self.GOAL_DISTANCE_THRESHOLD:
                self.logger.info(f"Robot within goal position distance threshold: {self.robot_dist_to_goal}/{self.GOAL_DISTANCE_THRESHOLD}")
                # Simulate manipulation task or set new goal
                # Reset goal or set to home position if needed


    ##########################
    ### Main Function #######
    ##########################

def main():
    
    json_file_path = ".//rob7_760_2024//settings.json"
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    TIMER_PERIOD = json_handler.get_subkey_value("Main", "TIMER_PERIOD")
    GOAL_DISTANCE_THRESHOLD = json_handler.get_subkey_value("Main", "GOAL_DISTANCE_THRESHOLD")
    INIT_SLEEP_DURATION = json_handler.get_subkey_value("Main", "INIT_SLEEP_DURATION")
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("Main", "NODE_LOG_LEVEL")

    rclpy.init()
    rclpy.logging.set_logger_level("main", eval(NODE_LOG_LEVEL))
    
    main = Main(TIMER_PERIOD, GOAL_DISTANCE_THRESHOLD, INIT_SLEEP_DURATION)
    rclpy.spin(main)
    

if __name__ == "__main__":
    main()
