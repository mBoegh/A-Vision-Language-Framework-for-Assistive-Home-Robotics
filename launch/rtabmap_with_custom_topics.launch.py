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
        'wait_for_transform': 0.200000,
        'qos' : 1, 
        'rviz': True,   
        # Frame IDs
        'rgb_frame_id': 'head_front_camera_rgb_frame',  # RGB camera frame
        'depth_frame_id': 'head_front_camera_depth_frame',  # Depth camera frame
        'imu_frame_id': 'base_imu_link',  # IMU frame
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
