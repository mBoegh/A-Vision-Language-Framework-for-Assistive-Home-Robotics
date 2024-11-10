from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():

    parameters = [{
        'frame_id': 'base_link',  # Change this to your robot's frame_id
        'subscribe_depth': True,
        'subscribe_odom_info': True,  # Subscribe to odometry
        'approx_sync':True,
        'wait_imu_to_init':True, 
        'qos' : 1, 
        'rviz': True,   
        # RTAB-Map parameters (adjust them based on your needs)
        'Odom/Strategy': '0',  # Use odometry for SLAM
        'Odom/ResetCountdown': '15',  # Time before resetting odometry
        'Odom/GuessSmoothingDelay': '0',  # Delay before smoothing guess
        'Rtabmap/StartNewMapOnLoopClosure': 'true',  # Start new map on loop closure
        'RGBD/CreateOccupancyGrid': 'false',  # Optionally create occupancy grid
        'Rtabmap/CreateIntermediateNodes': 'true',  # Intermediate nodes for better mapping
        'RGBD/LinearUpdate': '0',  # Update rate in meters
        'RGBD/AngularUpdate': '0',  # Update rate in radians
        'sync_queue_size': 20,  # Increased queue size
        'topic_queue_size': 10  # Increased topic queue size
    
    }]

    remappings = [
        ('rgb/image', '/head_front_camera/rgb/image_raw'),
        ('depth/image', '/head_front_camera/depth_registered/image_raw'),
        ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),
        ('odom', '/mobile_base_controller/odom'),
        ('imu', '/imu_sensor_broadcaster/imu')
    ]
    
    return LaunchDescription([

    SetParameter(name='use_sim_time', value=True),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),
    
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),    

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        )
    ])
