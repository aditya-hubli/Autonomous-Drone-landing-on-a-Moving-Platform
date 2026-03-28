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
                'motion_type': 'random_walk',
                'max_speed': 0.4,
            }],
        ),
    ])
