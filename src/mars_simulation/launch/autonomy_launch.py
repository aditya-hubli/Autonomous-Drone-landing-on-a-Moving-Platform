"""Launch all MARS autonomy nodes: platform mover, perception, tracking, drone control."""

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    # Static transform from base_link to camera_link for the downward-facing camera
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        output='screen',
        arguments=['0', '0', '-0.06', '0', '0.7071068', '0', '0.7071068', 'drone/base_link', 'drone/camera_link'],
    )

    # Platform mover — start immediately
    platform_mover = Node(
        package='mars_platform',
        executable='platform_mover',
        name='platform_mover',
        output='screen',
        parameters=[{
            'max_speed': 0.6,
            'boundary': 2.5,
        }],
    )

    # Platform tracker — 2s delay
    platform_tracker = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='>>> Starting platform tracker...'),
            Node(
                package='mars_tracking',
                executable='platform_tracker',
                name='platform_tracker',
                output='screen',
            ),
        ],
    )

    # Drone controller — 3s delay
    drone_controller = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='>>> Starting drone controller — mission begins!'),
            Node(
                package='mars_drone_control',
                executable='drone_controller',
                name='drone_controller',
                output='screen',
                parameters=[{
                    'overview_height': 5.0,
                    'descend_speed': 0.4,
                    'land_height': 0.55,
                    'xy_tolerance': 0.2,
                }],
            ),
        ],
    )

    return LaunchDescription([
        LogInfo(msg='>>> Starting platform mover...'),
        platform_mover,
        camera_tf,
        platform_tracker,
        drone_controller,
    ])
