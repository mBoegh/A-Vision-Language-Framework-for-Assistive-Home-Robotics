from rob7_760_2024.LIB import JSON_Handler

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2

import numpy  as np
import math
from itertools import product
import ast

class MainNode(Node):
    """
    This is the Main node of the ROS2 network.
    """

    def __init__(self, timer_period, goal_distance_threshold, filename, centroids_filtering_distance, compute_dist_to_goal_timer_period, task_timer_period):
        # Initializing parsed variables.
        self.TIMER_PERIOD = timer_period
        self.GOAL_DISTANCE_THRESHOLD = goal_distance_threshold
        self.FILENAME = filename
        self.CENTROIDS_FILTERING_DISTANCE = centroids_filtering_distance
        self.COMPUTE_DIST_TO_GOAL_TIMER_PERIOD = compute_dist_to_goal_timer_period
        self.TASK_TIMER_PERIOD = task_timer_period

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

        self.label_dictionary = {
            1.0: 'person',
            2.0: 'couch',
            3.0: 'chair',
            4.0: 'tv',
            5.0: 'cup',
            6.0: 'sink',
            7.0: 'spoon',
            8.0: 'vase',
            9.0: 'refrigerator',
            10.0: 'table',
            11.0: 'sports ball',
            12.0: 'cell phone',
            13.0: 'bench',
            14.0: 'bed'
        }

        self.task_timer_exceeded = False
        self.already_executed_flag = False
        self.goal_reached_flag = False
        self.orientated_correctly_flag = False
        self.centroids_new = None
        self.centroids = np.load(self.FILENAME, allow_pickle=False)
        self.old_labels_to_visit = None
        self.new_labels_to_visit = None
        self.goal_position = None
        self.robot_dist_to_goal = None

        # Define a dtype dynamically based on the field names
        self.dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.uint32),
        ])

        # Initialize robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        # Initializing a timer that periodically calls the timer_callback function.
        self.main_timer = self.create_timer(self.TIMER_PERIOD, self.main_timer_callback)
        self.task_timer = self.create_timer(self.TASK_TIMER_PERIOD, self.task_timer_callback)
        self.task_timer.cancel()
        
        
        #initialise centroids and reverse mapping
        
        #self.centroids_initialized = self.load_pointcloud_from_bin() 
        
        #        self.reverse_mapping = {v: k for k, v in self.label_dictionary.items()}
         #       self.labeled_values = [
          #          (*value[:3], self.reverse_mapping.get(value[3], None)) for value in self.centroids_initialized
           #     ]        
           
           
           
           
        ##########################
        ### Publishers ###########
        ##########################
        self.combine_pointcloud_bool_publisher = self.create_publisher(Bool, '/combine_pointcloud_bool', 10)
        self.combine_pointcloud_bool_msg = Bool()

        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_pose_msg = PoseStamped()
        
        self.robot_reached_goal_publisher = self.create_publisher(Bool, '/robot_reached_goal', 10)
        self.robot_reached_goal_msg = Bool()
        self.robot_reached_goal_msg.data = True

        ##########################
        ### Subscribers ##########
        ##########################
        self.object_list_subscription = self.create_subscription(String, '/object_list', self.object_list_topic_callback, 10)
        self.PoseWithCovarianceStamped_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.PoseWithCovarianceStamped_callback, 10)

        # Publisher for the filtered points as PointCloud2
        self.centroids_subscriber = self.create_subscription(PointCloud2, '/centroids', self.centroids_callback, 10)
    
        self.trigger_subscriber = self.create_subscription(Bool, '/trigger1', self.trigger_callback, 10)
                
        self.logger.fatal("Waiting for trigger.")


    #############################
    ### Callback Functions ######
    #############################

    def trigger_callback(self, msg):
        self.logger.fatal(f"Received data '{msg.data}'")
        
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

            if not self.old_labels_to_visit == self.new_labels_to_visit and self.already_executed_flag and self.orientated_correctly_flag:
                self.already_executed_flag = False
                self.orientated_correctly_flag = False
            
        except (ValueError, SyntaxError) as e:
            self.logger.error(f"Error parsing labels: {e}")

    def PoseWithCovarianceStamped_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        self.robot_angle = msg.pose.pose.orientation
        self.logger.debug(f'Robot position - x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}, angle :{self.robot_angle}')

    def centroids_callback(self, msg):
        # Read the points from the PointCloud2 message and convert each point from a tuple to a list
        self.centroids_new = [list(value) for value in pc2.read_points(
            msg, field_names=["x", "y", "z", "label"], skip_nans=True)]

        # Log the received centroids
        self.logger.debug(f"Received {len(self.centroids_new)} labeled centroids: '{self.centroids_new}'")


    ###########################
    ### Utility Functions #####
    ###########################

    def filter_items_by_label(self, items, label):
        return [item[:3] for item in items if item[3] == label]

    def euclidean_distance(self, coords1, coords2):
        dist = math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(coords1, coords2)))
        #self.logger.debug(f"Distance to goal: '{dist}'")
        return dist
    
    def find_min_distance_path(self, centroids, objects_to_visit):
        grouped_positions = [self.filter_items_by_label(centroids, object) for object in objects_to_visit]
                
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

        # Compute half angles
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        # Calculate quaternion
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        self.logger.debug("7")
        return (x, y, z, w)
   

    def publish_pose(self):
        yaw_radians = math.atan2(self.goal_position[1] - self.robot_y, self.goal_position[0] - self.robot_x)
        
        self.logger.debug(f"yaw_radians: '{yaw_radians}'")
        
        quaternion = self.euler_to_quaternion(0, 0, yaw_radians)

        self.goal_pose_msg.header.frame_id = 'map'
        self.goal_pose_msg.header.stamp = self.get_clock().now().to_msg()

        self.goal_pose_msg.pose.position.x = float(self.goal_position[0])
        self.goal_pose_msg.pose.position.y = float(self.goal_position[1])
        self.goal_pose_msg.pose.position.z = float(self.goal_position[2])

        self.goal_pose_msg.pose.orientation.x = quaternion[0]
        self.goal_pose_msg.pose.orientation.y = quaternion[1]
        self.goal_pose_msg.pose.orientation.z = quaternion[2]
        self.goal_pose_msg.pose.orientation.w = quaternion[3]

        self.goal_pose_publisher.publish(self.goal_pose_msg)
        self.logger.debug(f"""Published goal_pose_msg to /goal_pose topic:\n
                            x: {self.goal_pose_msg.pose.position.x}\n
                            y: {self.goal_pose_msg.pose.position.y}\n
                            z: {self.goal_pose_msg.pose.position.z}\n\n
                            yaw_radians: {yaw_radians}\n\n
                            x quaternion: {self.goal_pose_msg.pose.orientation.x}\n
                            y quaternion: {self.goal_pose_msg.pose.orientation.y}\n
                            z quaternion: {self.goal_pose_msg.pose.orientation.z}\n
                            w quaternion: {self.goal_pose_msg.pose.orientation.w}""")
        
        self.logger.debug("Reset task timer.")
        
        
    def load_pointcloud_from_bin(self):

        # Load the binary file into a numpy array with the defined dtype
        points_array = np.fromfile('centroids.bin', dtype=self.dtype)

        # Now `points_array` contains the point cloud data with correct types for each field
        print(f"Loaded {len(points_array)} points from {self.FILENAME}")

        return points_array
        
        
    def save_pointcloud_to_bin(self):
        
        # Create the numpy structured array
        points_array = np.array(self.centroids_initialized, dtype=self.dtype)

        # Save to binary file
        points_array.tofile('centroids.bin')
        print(f"Point cloud saved to {self.FILENAME}")
        
        import numpy as np

    def update_centroids(self, distance_threshold=0.1):
        if self.centroids_new:
            # Create a copy of the initialized centroids to avoid modifying the list during iteration
            updated_centroids = np.copy(self.centroids)
            self.logger.fatal("Updating centroids")

            # Ensure centroids_new is a numpy array for easier handling
            self.centroids_new = np.array(self.centroids_new)

            for new_centroid in self.centroids_new:
                found_near = False

                for i, init_centroid in enumerate(updated_centroids):
                    x_nc, y_nc, z_nc = new_centroid[0:3]
                    x_ic, y_ic, z_ic = init_centroid[0:3]
                    # Calculate the Euclidean distance only based on (x, y, z) coordinates
                    distance = self.euclidean_distance([x_nc, y_nc, z_nc], [x_ic, y_ic, z_ic])
                    
                    if distance <= distance_threshold:
                        # If the new centroid is near the initialized one, substitute it
                        updated_centroids[i] = new_centroid
                        found_near = True
                        break  # No need to check further if we have already found a match

                if not found_near:
                    # If no near centroid was found, add the new centroid to the list using np.vstack
                    updated_centroids = np.vstack([updated_centroids, new_centroid])

            # Publish the message for the robot goal
            self.robot_reached_goal_publisher.publish(self.robot_reached_goal_msg)
            
            # Update the centroids and save them to an npy file
            self.centroids = updated_centroids
            np.save("updated_centroids.npy", self.centroids)  # Save the centroids to a file
            self.logger.info(f"Updated centroids: {self.centroids}")


        
    ##########################
    ### Timer Callback #######
    ##########################


    def task_timer_callback(self):        
        self.goal_pose_msg.header.frame_id = 'map'
        self.goal_pose_msg.header.stamp = self.get_clock().now().to_msg()

        self.goal_pose_msg.pose.position.x = 0.0
        self.goal_pose_msg.pose.position.y = 0.0
        self.goal_pose_msg.pose.position.z = 0.0

        self.goal_pose_msg.pose.orientation.x = 0.0
        self.goal_pose_msg.pose.orientation.y = 0.0
        self.goal_pose_msg.pose.orientation.z = 0.0
        self.goal_pose_msg.pose.orientation.w = 1.0

        self.goal_pose_publisher.publish(self.goal_pose_msg)
        self.logger.debug(f"""Published goal_pose_msg to /goal_pose topic:\n
                            x: {self.goal_pose_msg.pose.position.x}\n
                            y: {self.goal_pose_msg.pose.position.y}\n
                            z: {self.goal_pose_msg.pose.position.z}\n\n
                            yaw_radians: {0}\n\n
                            x quaternion: {self.goal_pose_msg.pose.orientation.x}\n
                            y quaternion: {self.goal_pose_msg.pose.orientation.y}\n
                            z quaternion: {self.goal_pose_msg.pose.orientation.z}\n
                            w quaternion: {self.goal_pose_msg.pose.orientation.w}""")
        
        self.task_timer.cancel()        
        
        self.already_executed_flag = True
        self.orientated_correctly_flag = True
        
        self.goal_position = [0.0, 0.0, 0.0]
        
        
    def main_timer_callback(self):
        if self.trigger:
            a = False   
            if None in [self.robot_x, self.robot_y, self.robot_z]:
                self.logger.error("Robot position is not available yet.")
                return
            
            if self.new_labels_to_visit is not None and not self.new_labels_to_visit == self.old_labels_to_visit and not self.already_executed_flag:    
                
                # Create a list of [(x,y,z,label_name), (x,y,z,label_name), ...] from the centroids. Notice that is is using the label_dictionary so label_id is now a label_name
                self.centroid_list = [(*value[:3], self.label_dictionary.get(value[3], None)) for value in self.centroids]

                self.logger.debug(f"centroid_list: {self.centroid_list}")

                # missing_labels = [label for label in self.new_labels_to_visit if label not in self.label_dictionary]
                
                # if missing_labels:
                #     self.logger.error(f"Missing labels in label mapping: {missing_labels}")
                #     return

                shortest_distance, best_path = self.find_min_distance_path(self.centroid_list, self.new_labels_to_visit)
                
                if best_path is None:
                    self.logger.warning("No valid path found.")
                    a = True
                else:
                    self.logger.debug(f"Minimum total distance: {shortest_distance}")
                    self.logger.debug(f"Labels to visit: {self.new_labels_to_visit}")
                    self.logger.debug(f"Best positions for items: {best_path}")

                    self.goal_position = best_path[-1]

                    self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y, self.robot_z], self.goal_position)
                    self.logger.info(f"Robot distance to goal '{self.goal_position}': {self.robot_dist_to_goal}")
                    
                    self.publish_pose()
                    self.task_timer.reset()            

                self.already_executed_flag = True
                self.logger.debug(f"already_executed_flag: '{self.already_executed_flag}'")
            if not a:
                if self.robot_x and self.robot_y and self.robot_z and self.goal_position:
                    self.robot_dist_to_goal = self.euclidean_distance([self.robot_x, self.robot_y], self.goal_position[0:2])
                    self.logger.debug(f"Robot dist to goal: {self.robot_dist_to_goal} / {self.GOAL_DISTANCE_THRESHOLD}", throttle_duration_sec=1)
                
                if not self.robot_dist_to_goal == None and self.robot_dist_to_goal < self.GOAL_DISTANCE_THRESHOLD and not self.orientated_correctly_flag:
                    
                    self.logger.fatal("Robot reached goal.")
                                    
                    self.logger.debug(f"Published robot_reached_goal_msg: '{self.robot_reached_goal_msg.data}'")
                    
                    self.publish_pose()
                    self.task_timer.cancel()
                    
                    self.orientated_correctly_flag = True
                
                    #update 
                    self.update_centroids(self.CENTROIDS_FILTERING_DISTANCE)

                    np.save(self.FILENAME, self.centroids)
            else:
                a = False
        #turn on for initialization      
        #np.save(self.FILENAME, self.centroids)
                                                        

def main():
    # Path for 'settings.json' file
    json_file_path = ".//rob7_760_2024//settings.json"
    
    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    json_handler = JSON_Handler(json_file_path)
    
    # Get settings from 'settings.json' file
    TIMER_PERIOD = json_handler.get_subkey_value("MainNode", "TIMER_PERIOD")
    GOAL_DISTANCE_THRESHOLD = json_handler.get_subkey_value("MainNode", "GOAL_DISTANCE_THRESHOLD")
    NODE_LOG_LEVEL = "rclpy.logging.LoggingSeverity." + json_handler.get_subkey_value("MainNode", "NODE_LOG_LEVEL")
    FILENAME = json_handler.get_subkey_value("MainNode", "FILENAME")
    CENTROIDS_FILTERING_DISTANCE = json_handler.get_subkey_value("MainNode", "CENTROIDS_FILTERING_DISTANCE")
    COMPUTE_DIST_TO_GOAL_TIMER_PERIOD = json_handler.get_subkey_value("MainNode", "COMPUTE_DIST_TO_GOAL_TIMER_PERIOD")
    TASK_TIMER_PERIOD = json_handler.get_subkey_value("MainNode", "TASK_TIMER_PERIOD")

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
    main_node = MainNode(TIMER_PERIOD, GOAL_DISTANCE_THRESHOLD, FILENAME, CENTROIDS_FILTERING_DISTANCE, COMPUTE_DIST_TO_GOAL_TIMER_PERIOD, TASK_TIMER_PERIOD)
    
    # Begin looping the node
        # Begin looping the node
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        main_node.logger.info("Shutting down GetCentroidsNode.")
       
        
        
        main_node.logger.info(f"Saving centroids to '{FILENAME}'.")

       
        #does it pass automaticaly the self?
        #main_node.update_centroids() 
        #main_node.save_pointcloud_to_bin()
        
    finally:
        main_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
