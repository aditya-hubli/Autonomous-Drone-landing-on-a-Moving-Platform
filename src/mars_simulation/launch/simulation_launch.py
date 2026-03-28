"""
Simulation-only launch: Gazebo + spawn drone + platform (no autonomy nodes).
Useful for testing the simulation setup.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mars_sim = get_package_share_directory('mars_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_mars_sim, 'worlds', 'mars_world.world')
    models_path = os.path.join(pkg_mars_sim, 'models')
    install_lib = os.path.join(pkg_mars_sim, '..', '..', 'lib')

    drone_sdf = os.path.join(models_path, 'mars_drone', 'model.sdf')
    platform_sdf = os.path.join(models_path, 'landing_platform', 'model.sdf')

    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        install_lib + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    # 1. Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    # 2. Gazebo GUI
    gzclient = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='>>> Starting Gazebo GUI...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            ),
        ],
    )

    # 3. Spawn drone
    spawn_drone = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='>>> Spawning drone...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_drone',
                output='screen',
                arguments=[
                    '-entity', 'drone',
                    '-file', drone_sdf,
                    '-x', '0.0', '-y', '0.0', '-z', '3.0',
                ],
            ),
        ],
    )

    # 4. Spawn platform
    spawn_platform = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='>>> Spawning platform...'),
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
        gazebo_model_path,
        gazebo_plugin_path,
        gzserver,
        gzclient,
        spawn_drone,
        spawn_platform,
    ])
