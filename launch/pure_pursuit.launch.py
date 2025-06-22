from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pure_pursuit_ros2'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pure_pursuit_ros2',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[config],
            output='screen'
        )
    ])
