from rob7_760_2024.LIB import JSON_Handler
     
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import math
from itertools import product
import time


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

        # This is the ROS2 Humble logging system, which is build on the Logging module for Python.
        # It displays messages with developer specified importance.
        # Here all the levels of importance are used to indicate that the script is running.
        self.logger.debug("Hello world!")
        self.logger.info("Hello world!")
        self.logger.warning("Hello world!")
        self.logger.error("Hello world!")
        self.logger.fatal("Hello world!")

       # Example input with N-dimensional data
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
        
        self.old_labels_to_visit = None
        self.new_labels_to_visit = None

        self.goal_position = None

        self.robot_x = None
        self.robot_y = None
        self.robot_z = None

        self.start_position = None

        # Initialising a timer. The timer periodically calls the timer_callback function. This is essentially a while loop with a set frequency.
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')



    #########################
    ### Example Publisher ###
    #########################
    
        # # Initialising a publisher to the topic 'example'.
        # # On this topic is expected data of type std_msgs.msg.* which is imported as *.
        # # The '10' argument is some Quality of Service parameter (QoS).
        # self.example_publisher = self.create_publisher(String, 'example', 10)
        # self.example_publisher_publisher  # prevent unused variable warning

        # self.example_msg = String()

    ######################
    ### End of example ###
    ######################
    


##########################
### Example Subscriber ###
##########################

    # # Initialising a subscriber to the topic 'example'.
    # # On this topic is expected data of type std_msgs.msg.* which is imported as *.
    # # The subscriber calls a defined callback function upon message recieval from the topic.
    # # The '10' argument is some Quality of Service parameter (QoS).
    # self.example_subscription = self.create_subscription(String, 'example', self.example_topic_callback, 10)
    # self.example_subscription  # prevent unused variable warning

# def example_topic_callback(self, msg):
#     """
#     Callback function called whenever a message is recieved on the subscription 'example_subscription'
#     """
#     self.logger.debug(f"Recieved data '{msg.data}'")

######################
### End of example ###
######################


############################
######## PUBLISHERS ########
############################

        self.trigger_publisher = self.create_publisher(Bool, '/trigger', 10)
        self.trigger_publisher

        self.trigger_msg = Bool()
        

        self.combine_pointcloud_bool_publisher = self.create_publisher(Bool, '/combine_pointcloud_bool', 10)
        self.combine_pointcloud_bool_publisher

        self.combine_pointcloud_bool_msg = Bool()

#############################
######## SUBSCRIBERS ########
#############################

        ######## LLM ########

        # Initialising a subscriber to the topic '/object_list'.
        # On this topic is expected data of type std_msgs.msg.String which is imported as String.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.object_list_subscription = self.create_subscription(String, '/object_list', self.object_list_topic_callback, 10)
        self.object_list_subscription  # prevent unused variable warning


        ######## MAP ########

        # Initialising a subscriber to the topic '/transformed_points'.
        # On this topic is expected data of type sensor_msgs.msg.PointCloud2 which is imported as PointCloud2.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.point_cloud_subscription = self.create_subscription(PointCloud2, '/transformed_points', self.transformed_points_topic_callback, 10)
        self.point_cloud_subscription  # prevent unused variable warning

        # Initialising a subscriber to the topic '/localization_pose'.
        # On this topic is expected data of type geometry_msgs.msg.PoseWithCovarianceStamped which is imported as PoseWithCovarianceStamped.
        # The subscriber calls a defined callback function upon message recieval from the topic.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.PoseWithCovarianceStamped_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.PoseWithCovarianceStamped_callback, 10)
        self.PoseWithCovarianceStamped_subscription  # prevent unused variable warning

        time.sleep(self.INIT_SLEEP_DURATION)

        self.trigger_msg.data = True
        self.trigger_publisher.publish(self.trigger_msg)


    def object_list_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription '/object_list'.
        """

        self.logger.debug(f"Recieved data '{msg.data}'")

        self.old_labels_to_visit = self.new_labels_to_visit
        self.new_labels_to_visit = msg.data


    def transformed_points_topic_callback(self, msg):
        """
        Callback function called whenever a message is recieved on the subscription '/transformed_points'.
        """

        self.logger.debug(f"Recieved data '{msg.data}'")


    def PoseWithCovarianceStamped_callback(self, msg):
        '''
        Callback function called whenever a message is received on the subscription '/localization_pose'.
        '''

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

        self.start_position = [self.robot_x, self.robot_y, self.robot_z]

        # Log the values (or process them as needed)
        self.logger.debug(f'Robot position - x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}')

    # Filter items by specific labels
    def filter_items_by_label(self, items, label):
        return [item[1:] for item in items if item[0] == label]

    # Compute Euclidean distance in N-dimensional space
    def euclidean_distance(self, coords1, coords2):
        return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(coords1, coords2)))

    # Find the minimum total distance for a sequence of items
    def find_min_distance_path(self, items, labels):
        # Group all positions for each label
        grouped_positions = [self.filter_items_by_label(items, label) for label in labels]
        
        # Generate all combinations of positions (one for each label)
        min_distance = float('inf')
        best_combination = None
        
        for combination in product(*grouped_positions):
            # Compute total distance for this combination
            total_distance = sum(
                self.euclidean_distance(combination[i], combination[i + 1])
                for i in range(len(combination) - 1)
            )
            if total_distance < min_distance:
                min_distance = total_distance
                best_combination = combination
        
        return min_distance, best_combination


    def send_nav_goal(self, x, y, z=0.0, yaw=0.0):
        """
        Send a navigation goal to the Nav2 stack.
        :param x: X-coordinate of the goal.
        :param y: Y-coordinate of the goal.
        :param z: Z-coordinate of the goal (optional, default is 0.0).
        :param yaw: Orientation (yaw) of the goal (optional, default is 0.0).
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Nav2 action server not available!")
            return

        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = "map"  # Assuming goal is in the 'map' frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)  # Convert yaw to quaternion
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg.pose = goal_pose

        # Send the goal to Nav2
        self.logger.info(f"Sending navigation goal to Nav2: ({x}, {y}, {z}) with yaw {yaw}")
        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """
        Callback for Nav2 goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error("Nav2 goal was rejected!")
            return

        self.logger.info("Nav2 goal accepted. Waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        """
        Callback for Nav2 goal result.
        """
        result = future.result().result
        if result == 0:  # Success
            self.logger.info("Nav2 goal reached successfully!")
        else:
            self.logger.warning(f"Nav2 goal failed with result code: {result}")


    def timer_callback(self):
        '''
        The callback function called during each period of the timer.
        '''

        if not self.new_labels_to_visit == None and not self.new_labels_to_visit == self.old_labels_to_visit:
            # Reverse lookup dictionary
            reverse_mapping = {v: k for k, v in self.label_mapping.items()}

            # Parse data with labels for N-dimensional coordinates
            labeled_values = [
                (reverse_mapping.get(value[0], None), *value[1:])
                for value in self.values
            ]

            # Log the labeled values to confirm the mapping
            self.logger.debug(f"Labeled values: {labeled_values}")

            # Ensure labels to visit exist in the mapping
            missing_labels = [label for label in self.labels_to_visit if label not in self.label_mapping]
            if missing_labels:
                self.logger.error(f"Missing labels in label mapping: {missing_labels}")
                return

            # Compute the shortest path
            shortest_distance, best_path = self.find_min_distance_path(labeled_values, self.labels_to_visit)

            if best_path is None:
                self.logger.warning("No valid path found. Please check the input data or labels.")
            else:
                self.logger.debug(f"Minimum total distance: {shortest_distance}")
                self.logger.debug(f"Labels to visit: {self.labels_to_visit}")
                self.logger.debug(f"Best positions for items: {best_path}")

                # compute robot distance to goal position
                self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y, self.robot_z], [self.last_label_positions])
                self.logger.info(f"Robot distance to goal '{self.last_label}': {self.robot_dist_to_goal}")

                # when no goal has been set, set the goal to be the last object of best_path. This object is the true goal, as segmented by the LLM
                if self.goal_position == None:
                    self.goal_position = best_path[-1]

                    x, y, z = self.goal_position  # Assuming 3D position
                    self.send_nav_goal(x, y, z)

        try:    
            if self.robot_dist_to_goal < self.GOAL_DISTANCE_THRESHOLD:
                self.logger.info(f"Robot within goal position distance threshold: {self.goal_position}\nwith '{self.robot_dist_to_goal}' distance to true goal position. ")

                ## simulate manipulation task, ie. wait #######################################################################################################
                start_x, start_y, start_z = self.start_position
                self.send_nav_goal(start_x, start_y, start_z)
                
        except Exception as e:
            self.logger.warning(f"Not able to send navigation goal with error: '{e}'")



####################
######  MAIN  ######
####################


def main():
    
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    TIMER_PERIOD = json_handler.get_subkey_value("Main", "TIMER_PERIOD")
    GOAL_DISTANCE_THRESHOLD = json_handler.get_subkey_value("Main", "GOAL_DISTANCE_THRESHOLD")
    INIT_SLEEP_DURATION = json_handler.get_subkey_value("Main", "INIT_SLEEP_DURATION")
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("Main", "NODE_LOG_LEVEL")

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
    rclpy.logging.set_logger_level("main", eval(NODE_LOG_LEVEL))
    
    # Instance the main class
    main = Main(TIMER_PERIOD, GOAL_DISTANCE_THRESHOLD, INIT_SLEEP_DURATION)

    # Begin looping the node
    rclpy.spin(main)
    

if __name__ == "__main__":
    main()