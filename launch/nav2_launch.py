import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to your custom parameters YAML file (adjust the path as needed)
    param_file = os.path.join(get_package_share_directory('rob7_760_2024'), 'config', 'nav2.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('param_file', default_value=param_file, description='path to the param file'),

        # Start the Navigation2 stack (e.g., controller server, planner, etc.)
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='navigation',
            output='screen',
            parameters=[param_file]
        ),
    ])
