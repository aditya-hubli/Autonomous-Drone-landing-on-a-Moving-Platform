"""Spawn drone and platform into a running Gazebo instance."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mars_sim = get_package_share_directory('mars_simulation')
    models_path = os.path.join(pkg_mars_sim, 'models')

    drone_sdf = os.path.join(models_path, 'mars_drone', 'model.sdf')
    platform_sdf = os.path.join(models_path, 'landing_platform', 'model.sdf')

    # Spawn drone
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        output='screen',
        arguments=[
            '-entity', 'drone',
            '-file', drone_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '3.0',
        ],
    )

    # Spawn platform after drone
    spawn_platform = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='>>> Spawning landing platform...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_platform',
                output='screen',
                arguments=[
                    '-entity', 'platform',
                    '-file', platform_sdf,
                    '-x', '2.0', '-y', '0.0', '-z', '0.025',
                ],
            ),
        ],
    )

    return LaunchDescription([
        LogInfo(msg='>>> Spawning drone...'),
        spawn_drone,
        spawn_platform,
    ])
