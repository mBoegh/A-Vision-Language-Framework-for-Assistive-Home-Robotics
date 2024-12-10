from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
            )

    ])