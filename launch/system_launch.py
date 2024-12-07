from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():

    param = [{
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'rgb_frame_id': 'head_front_camera_rgb_optical_frame',  # RGB camera frame
        #'depth_frame_id': 'head_front_camera_depth_frame',  # Depth camera frame
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'use_sim_time': True,
        'RGBD/MaxDepth': 8.0,
        'RGBD/MinDepth': 0.6,
        'imu_frame_id':'base_imu_link',
        
        'database_path':'~/.ros/rtabmap.db',
        'Mem/IncrementalMemory': 'true',
        #'RGBD/OptimizeFromGraphEnd': 'true',
        'Grid/MaxObstacleHeight':'1.5',
        #'Rtabmap/LoopClosureDetection':'true',
        #'Mem/NotLinkedNodesKept': 'false',  # Automatically remove unlinked nodes
        #'Mem/STMSize': '10',  # Short-term memory size
        #'RGBD/MaxNodesRemoval': 2,  # Allow removal of up to 1 node at a time
        
        # use this after initialization
        'Rtabmap/Localization': True,   
    }]

    remappings = [
        # camera param
        ('rgb/image', '/head_front_camera/rgb/image_raw'),
        ('depth/image', '/head_front_camera/depth_registered/image_raw'),
        ('rgb/camera_info', '/head_front_camera/rgb/camera_info'),

        # imu to improve odometry
        ('imu', '/imu_sensor_broadcaster/imu'),
        
        ]


    return LaunchDescription([

        Node(
            package='rob7_760_2024', executable='MainNode', output='screen',
            ),
        
        Node(
            package='rob7_760_2024', executable='LlmNode', output='screen',
            ),
        
        Node(
            package='rob7_760_2024', executable='SemanticPointcloudNode', output='screen',
            ),
        
        Node(
            package='rob7_760_2024', executable='GetCentroidsNode', output='screen',
            ),

        Node(
            package='rob7_760_2024', executable='ImageSegmentationNode', output='screen',
            ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=param,
            remappings=remappings),
    
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=param,
            remappings=remappings,
            ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=param,
            remappings=remappings),    

    ])