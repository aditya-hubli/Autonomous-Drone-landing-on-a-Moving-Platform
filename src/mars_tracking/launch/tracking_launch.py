from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_tracking',
            executable='platform_tracker',
            name='platform_tracker',
            output='screen',
        ),
    ])
