from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():

    param_slam = [{
        'subscribe_depth': True,
        'subscribe_rgb': True,
        #'subscribe_odom_info': True,
        'wait_for_transform': 0.2,
        'approx_sync': True,
        #'rgb_frame_id': 'head_front_camera_rgb_frame',  # RGB camera frame
        #'depth_frame_id': 'head_front_camera_depth_frame',  # Depth camera frame
        #'imu_frame_id': 'base_imu_link',  # IMU frame        
        #'odom_frame_id': 'odom',
        #'frame_id': 'base_link',
        'use_sim_time':True,
        'RGBD/MaxDepth': 7.0,
        'RGBD/MinDepth': 0.5,
        'RGBD/UseDepthForRegistration': True,
    }]

    param_odom = [{
        'frame_id': 'base_footprint',
        'Odom/MaxFeatures': 5000,
        'Odom/Strategy': 9,
        'Mem/CompressionParallelized': True,
        'odom_frame_id': 'odom',
        'publish_tf_odom':'/odom',
        'wait_for_transform': 0.2,
        'wait_imu_to_init':True,
        'qos': 2, 
        'approx_sync': True,
        'use_sim_time':True,
    }]
    param_viz = [{
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': True,
        'wait_for_transform': 0.2,
        'approx_sync': True,
        'use_sim_time':True,
    }]
    remappings = [
        # camera param
        ('rgb/image', '/head_front_camera/rgb/image_raw'),
        ('depth/image', '/head_front_camera/depth_registered/image_raw'),
        ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),
        
        # odometry
        ('odom', '/mobile_base_controller/odom'),
        
        # imu to improve odometry
        ('imu', '/imu_sensor_broadcaster/imu'),
    ]


    return LaunchDescription([

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=param_odom,
            remappings=remappings),
    
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=param_slam,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=param_viz,
            remappings=remappings),    

    ])
