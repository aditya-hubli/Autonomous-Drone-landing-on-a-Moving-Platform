"""Launch all MARS autonomy nodes: platform mover, perception, tracking, drone control."""

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    # Platform mover — start immediately
    platform_mover = Node(
        package='mars_platform',
        executable='platform_mover',
        name='platform_mover',
        output='screen',
        parameters=[{
            'motion_type': 'random_walk',
            'max_speed': 0.4,
        }],
    )

    # ArUco detector — 1s delay
    aruco_detector = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg='>>> Starting ArUco detector...'),
            Node(
                package='mars_perception',
                executable='aruco_detector',
                name='aruco_detector',
                output='screen',
            ),
        ],
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
                    'hover_height': 3.0,
                    'approach_height': 2.0,
                    'descend_speed': 0.3,
                    'land_height': 0.3,
                }],
            ),
        ],
    )

    return LaunchDescription([
        LogInfo(msg='>>> Starting platform mover...'),
        platform_mover,
        aruco_detector,
        platform_tracker,
        drone_controller,
    ])
