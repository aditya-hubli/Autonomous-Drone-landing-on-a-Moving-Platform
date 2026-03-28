from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_drone_control',
            executable='drone_controller',
            name='drone_controller',
            output='screen',
        ),
    ])
