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
