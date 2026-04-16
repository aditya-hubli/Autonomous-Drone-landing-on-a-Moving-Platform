from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_platform',
            executable='platform_mover',
            name='platform_mover',
            output='screen',
            parameters=[{
                'max_speed': 0.5,
                'boundary': 2.5,
            }],
        ),
    ])
