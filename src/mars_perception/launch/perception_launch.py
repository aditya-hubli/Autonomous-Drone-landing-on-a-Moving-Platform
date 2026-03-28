from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_perception',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
        ),
    ])
