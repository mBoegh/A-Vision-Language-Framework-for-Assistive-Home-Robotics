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
