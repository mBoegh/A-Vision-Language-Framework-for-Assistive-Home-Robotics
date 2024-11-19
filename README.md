# rob7_760_2024
Aalborg University Robotics student project by group 760 - 2024

# Installation
If you follow the guide *to the teeth* then it *should* be installed in these 6 steps.

## 1. ROS2 Humble Hawksbill Installation
### Requires Ubuntu 22.04 or similar.
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash
```

## 2. Install Simulation Enviroment
``` bash
cd

sudo apt-get update

sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions

mkdir -p ~/tiago_public_ws/src
cd ~/tiago_public_ws
vcs import --input https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/humble-devel/tiago_public.repos src

sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src

source /opt/ros/humble/setup.bash
colcon build --symlink-install

source ~/tiago_public_ws/install/setup.bash

cd ~/tiago_public_ws/src/
```

## 3. Clone this repository
```bash
gh repo clone mBoegh/rob7_760_2024
```
## 4. Build and source
```bash
cd ~/tiago_public_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/tiago_public_ws/install/setup.bash
```

# Running the simulation
In all new terminals you must source ROS2 and the package you wish to use like so:
```bash
cd ~/tiago_public_ws
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
```

## In one terminal - Launch Simulation
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pal_office [arm_type:=no-arm]
```

## In another terminal - Run the custom controller
```bash
ros2 run rob7_760_2024 custom_controller
```

# Customizing the custom_controller.py
## You can find the controller here:
```bash
cd ~/tiago_public_ws/src/rob7_760_2024/rob7_760_2024/
```

## When you make changes to the controller run this
```bash
cd ~/tiago_public_ws
source /opt/ros/humble/setup.bash
colcon build --package-select rob7_760_2024
source ~/tiago_public_ws/install/setup.bash
```
## in the tiago_gazebo -> mdoel -> .config file :
```bash
<?xml version="1.0" ?>
<database>
  <name>REEMC and Home Environment Models</name>
  <license>Creative Commons Attribution 3.0 Unported</license>
  <models>
    
    <!-- home models-->
    <uri>file://ground_plane</uri>
    <uri>file://cabinet</uri>
    <uri>file://cabinet_0</uri>
    <uri>file://cabinet_1</uri>
    <uri>file://cabinet_2</uri>
    <uri>file://cabinet_3</uri>
    <uri>file://cabinet_4</uri>
    <uri>file://bookshelf</uri>
    <uri>file://bookshelf_0</uri>
    <uri>file://bookshelf_1</uri>
    <uri>file://bookshelf_2</uri>
    <uri>file://bookshelf_3</uri>
    <uri>file://bookshelf_4</uri>
    <uri>file://table_0</uri>
    <uri>file://table_1</uri>
    <uri>file://table_2</uri>
    <uri>file://arm_chair</uri>
    <uri>file://arm_chair_0</uri>
    <uri>file://biscuits_pack</uri>
    <uri>file://bowl</uri>
    <uri>file://bowl_0</uri>
    <uri>file://cafe_table</uri>
    <uri>file://closet</uri>
    <uri>file://cocacola</uri>
    <uri>file://cocacola_0</uri>
    <uri>file://cocacola_1</uri>
    <uri>file://coke_can_slim</uri>
    <uri>file://dining_chair</uri>
    <uri>file://floor_lamp</uri>
    <uri>file://home_walls</uri>
    <uri>file://kitchen_chair</uri>
    <uri>file://kitchen_chair_0</uri>
    <uri>file://kitchen_table</uri>
    <uri>file://pringles2</uri>
    <uri>file://reviews_table</uri>
    <uri>file://sideboard</uri>
    <uri>file://sofa</uri>
    <uri>file://sprite</uri>
    <uri>file://tv</uri>
    <uri>file://wardrobe</uri>
    <uri>file://wardrobe_0</uri>
    <!-- New models added -->
    <uri>file://arm_chair</uri>
    <uri>file://aruco_board</uri>
    <uri>file://aruco_cube</uri>
    <uri>file://aruco_marker_0</uri>
    <uri>file://aruco_marker_20</uri>
    <uri>file://aruco_marker_250_6x6_1</uri>
    <uri>file://aruco_marker_250_6x6_2</uri>
    <uri>file://aruco_marker_250_6x6_3</uri>
    <uri>file://aruco_marker_250_6x6_4</uri>
    <uri>file://aruco_marker_6cm_238</uri>
    <uri>file://aruco_marker_6cm_26</uri>
    <uri>file://aruco_marker_6cm_582</uri>
    <uri>file://aruco_marker_6cm_63</uri>
    <uri>file://aruco_marker_9cm_238</uri>
    <uri>file://aruco_marker_9cm_26</uri>
    <uri>file://aruco_marker_9cm_582</uri>
    <uri>file://aruco_marker_9cm_63</uri>
    <uri>file://aruco_marker_original_238</uri>
    <uri>file://aruco_marker_original_26</uri>
    <uri>file://aruco_marker_original_582</uri>
    <uri>file://aruco_marker_original_63</uri>
    <uri>file://bauman</uri>
    <uri>file://bench</uri>
    <uri>file://bifrutas_tropical_can</uri>
    <uri>file://box_with_handles</uri>
    <uri>file://brick_box_3x1x3</uri>
    <uri>file://citizen_extras_female_02</uri>
    <uri>file://citizen_extras_female_03</uri>
    <uri>file://citizen_extras_male_03</uri>
    <uri>file://dock1</uri>
    <uri>file://door_obstacle</uri>
    <uri>file://goetz_sofa</uri>
    <uri>file://green_ball</uri>
    <uri>file://green_cube</uri>
    <uri>file://green_rectangle</uri>
    <uri>file://hospital_flat_map</uri>
    <uri>file://hospital_map</uri>
    <uri>file://ideal_sun</uri>
    <uri>file://ikea_ektorp_chair</uri>
    <uri>file://ikea_folke_chair</uri>
    <uri>file://ikea_harry_chair</uri>
    <uri>file://ikea_stefan_chair</uri>
    <uri>file://ikea_urban_chair</uri>
    <uri>file://lamp</uri>
    <uri>file://macrolink_small_table</uri>
    <uri>file://macrolink_table</uri>
    <uri>file://marker26_8cm</uri>
    <uri>file://pal_poster</uri>
    <uri>file://pal_textured_object</uri>
    <uri>file://reemc_bookshelf</uri>
    <uri>file://reemc_cabinet</uri>
    <uri>file://reemc_table</uri>
    <uri>file://reemc_textured_object</uri>
    <uri>file://reemc_textured_object_nograv</uri>
    <uri>file://reemc_walls</uri>
    <uri>file://robocup_walls_70cm_doors</uri>
    <uri>file://rockin_camp</uri>
    <uri>file://slippery_patch</uri>
    <uri>file://small_cylinder</uri>
    <uri>file://sun</uri>
    <uri>file://table_0m8</uri>
    <uri>file://table_1m</uri>
    <uri>file://test2</uri>
    <uri>file://testing_room</uri>
    <uri>file://wall_with_reem_photo</uri>
    <uri>file://willowgarage</uri>
  </models>
</database>
```
## install movit_planners_chomp
```bash
apt search ros-humble-moveit-planners-chomp
sudo apt install ros-humble-moveit-planners-chomp
```

## tiago_robot->tiago_description->robots->tiago.urdf.xacro
```bash
<!-- ARGUMENTS -->

  <!-- pmb2, omni_base -->
  <xacro:arg name="base_type" default="pmb2"/>

  <!-- no-laser, hokuyo, sick-551, sick-561, sick-571-->
  <xacro:arg name="laser_model" default="no-laser"/>

  <!-- false, true -->
  <xacro:arg name="has_screen" default="false"/>

  <!-- no-arm, tiago-arm -->
  <xacro:arg name="arm_type" default="no-arm"/>

  <!-- no-ft-sensor, schunk-ft -->
  <xacro:arg name="ft_sensor" default="no-ft-sensor"/>

  <!-- wrist-2010, wrist-2017 -->
  <xacro:arg name="wrist_model" default="wrist-2017"/>

  <!-- no-end-effector, pal-gripper, pal-hey5, custom, robotiq-2f-85,robotiq-2f-140-->
  <xacro:arg name="end_effector" default="no-end-effector"/>

  <!-- no-camera, orbbec-astra, orbbec-astra-pro, asus-xtion -->
  <xacro:arg name="camera_model" default="orbbec-astra-pro"/>

  <!-- false, true -->
  <xacro:arg name="has_thermal_camera" default="false"/>

  <!-- false, true -->
  <xacro:arg name="no_safety_eps" default="false"/>

  <!-- Calibration -->
  <xacro:arg name="description_calibration_dir" default="$(find tiago_description)/urdf/calibration"/>
  <xacro:arg name="extrinsic_calibration_dir" default="$(find tiago_description)/urdf/calibration"/>

  <!-- Execution env config -->
  <xacro:arg name="use_sim_time" default="true"/>
  <xacro:arg name="is_multiple" default="false"/>
  <xacro:arg name="namespace" default=""/>
  <xacro:arg name="is_public_sim" default="true"/>
```
## rob760_2024->launch->rtabmap_tiago.launch.py

``` bash
    param = [{
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'rgb_frame_id': 'head_front_camera_rgb_frame',  # RGB camera frame
        'depth_frame_id': 'head_front_camera_depth_frame',  # Depth camera frame
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'use_sim_time':True,
        'RGBD/MaxDepth': 8.0,
        'RGBD/MinDepth': 0.6,
    }]

    remappings = [
        # camera param
        ('rgb/image', '/head_front_camera/rgb/image_raw'),
        ('depth/image', '/head_front_camera/depth_registered/image_raw'),
        ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),

        # imu to improve odometry
        ('imu', '/imu_sensor_broadcaster/imu'),
    ]
```

## launch_pal->launch_pal->robot_arguments->common.py
```bash
use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        choices=['True', 'False'],
        description='Use simulation time.')
    namespace: DeclareLaunchArgument = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Define namespace of the robot.')
    robot_name: DeclareLaunchArgument = DeclareLaunchArgument(
        "robot_name",
        default_value="pmb2",
        description="Name of the robot. ",
        choices=[
            "pmb2",
            "tiago",
            "tiago_dual",
            "pmb3",
            "ari",
            "omni_base",
            "tiago_pro",
            "talos",
            "kangaroo",
        ],
    )
    navigation: DeclareLaunchArgument = DeclareLaunchArgument(
        name='navigation',
        default_value='False',
        choices=['True', 'False'],
        description='Specify if launching Navigation2.')
    advanced_navigation: DeclareLaunchArgument = DeclareLaunchArgument(
        name='advanced_navigation',
        default_value='False',
        choices=['True', 'False'],
        description='Specify if launching Advanced Navigation.')
    moveit: DeclareLaunchArgument = DeclareLaunchArgument(
        name='moveit',
        default_value='True',
        choices=['True', 'False'],
        description='Specify if launching MoveIt 2.')
    slam: DeclareLaunchArgument = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether or not you are using SLAM",
    )
    world_name: DeclareLaunchArgument = DeclareLaunchArgument(
        name='world_name',
        default_value='pal_office',
        description="Specify world name, will be converted to full path.")
    is_public_sim: DeclareLaunchArgument = DeclareLaunchArgument(
        name='is_public_sim',
        default_value='False',
        choices=['True', 'False'],
        description="Enable public simulation.")
    use_sensor_manager: DeclareLaunchArgument = DeclareLaunchArgument(
        name='use_sensor_manager',
        default_value='False',
        choices=['True', 'False'],
        description='Use moveit_sensor_manager for octomap')
    tuck_arm: DeclareLaunchArgument = DeclareLaunchArgument(
        name='tuck_arm',
        default_value='True',
        choices=['True', 'False'],
        description='Launches tuck arm node')
    x: DeclareLaunchArgument = DeclareLaunchArgument(
        name="x",
        description="X pose of the robot",
        default_value="0.0")
    y: DeclareLaunchArgument = DeclareLaunchArgument(
        name="y",
        description="Y pose of the robot",
        default_value="0.0")
    z: DeclareLaunchArgument = DeclareLaunchArgument(
        name="z",
        description="Z pose of the robot",
        default_value="0.0")
    roll: DeclareLaunchArgument = DeclareLaunchArgument(
        name="roll",
        description="Roll pose of the robot",
        default_value="0.0")
    pitch: DeclareLaunchArgument = DeclareLaunchArgument(
        name="pitch",
        description="Pitch pose of the robot",
        default_value="0.0")
    yaw: DeclareLaunchArgument = DeclareLaunchArgument(
        name="yaw",
        description="Yaw pose of the robot",
        default_value="0.0")
```
```bash
/arm_controller/controller_state
/arm_controller/joint_trajectory
/arm_controller/state
/arm_controller/transition_event
/attached_collision_object
/base_imu
/clock
/cloud_ground
/cloud_map
/cloud_obstacles
/cmd_vel
/collision_object
/diagnostics
/display_contacts
/display_planned_path
/dynamic_joint_states
/elevation_map
/filtered_cloud
/ft_sensor_controller/transition_event
/ft_sensor_controller/wrench
/global_path
/global_path_nodes
/global_pose
/goal
/goal_node
/goal_out
/goal_reached
/gps/fix
/grid_prob_map
/gripper_controller/controller_state
/gripper_controller/joint_trajectory
/gripper_controller/state
/gripper_controller/transition_event
/ground_truth_odom
/head_controller/controller_state
/head_controller/joint_trajectory
/head_controller/state
/head_controller/transition_event
/head_front_camera/depth_registered/camera_info
/head_front_camera/depth_registered/image_raw
/head_front_camera/depth_registered/points
/head_front_camera/rgb/camera_info
/head_front_camera/rgb/image_raw
/imu_sensor_broadcaster/imu
/imu_sensor_broadcaster/transition_event
/info
/initialpose
/input_joy/cmd_vel
/joint_state_broadcaster/transition_event
/joint_states
/joy_priority
/joy_vel
/key_vel
/labels
/landmark_detection
/landmark_detections
/landmarks
/local_grid_empty
/local_grid_ground
/local_grid_obstacle
/local_path
/local_path_nodes
/localization_pose
/map
/mapData
/mapGraph
/mapOdomCache
/mapPath
/marker
/marker_vel
/mobile_base_controller/cmd_vel_out
/mobile_base_controller/cmd_vel_unstamped
/mobile_base_controller/odom
/mobile_base_controller/transition_event
/monitored_planning_scene
/motion_plan_request
/octomap_binary
/octomap_empty_space
/octomap_full
/octomap_global_frontier_space
/octomap_grid
/octomap_ground
/octomap_obstacles
/octomap_occupied_space
/odom
/odom_info
/odom_info_lite
/odom_last_frame
/odom_local_map
/odom_local_scan_map
/odom_rgbd_image
/odom_sensor_data/compressed
/odom_sensor_data/features
/odom_sensor_data/raw
/parameter_events
/pause_navigation
/performance_metrics
/phone_vel
/planning_scene
/planning_scene_world
/play_motion2/transition_event
/robot_description
/robot_description_semantic
/rosout
/rtabmap/republish_node_data
/rviz_joy_vel
/scan_raw
/servoing_cmd_vel
/sonar_base
/stop_closing_loop
/tab_vel
/text_marker
/tf
/tf_static
/throttle_filtering_points/filtered_points
/torso_controller/controller_state
/torso_controller/joint_trajectory
/torso_controller/state
/torso_controller/transition_event
/trajectory_execution_event
/user_data_async
```
