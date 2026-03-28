"""
Single launch file for the entire MARS project.

Launches Gazebo with factory plugin, spawns models, then starts all autonomy nodes.
Usage: ros2 launch mars_simulation mars_full_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable,
    TimerAction, LogInfo
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mars_sim = get_package_share_directory('mars_simulation')
    models_path = os.path.join(pkg_mars_sim, 'models')
    install_lib = os.path.join(pkg_mars_sim, '..', '..', 'lib')
    world_file = os.path.join(pkg_mars_sim, 'worlds', 'mars_world.world')

    drone_sdf = os.path.join(models_path, 'mars_drone', 'model.sdf')
    platform_sdf = os.path.join(models_path, 'landing_platform', 'model.sdf')

    # ── Environment ──
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        install_lib + ':/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    # ── 0. Kill any stale Gazebo processes ──
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c',
             'killall -9 gzserver gzclient gazebo 2>/dev/null; sleep 1; echo "Cleared old Gazebo"'],
        output='screen',
    )

    # ── 1. Gazebo (2s after cleanup) ──
    gazebo = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='>>> Starting Gazebo...'),
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
                output='screen',
            ),
        ],
    )

    # ── 2. Spawn Platform FIRST (8s) — must exist before drone camera initializes ──
    spawn_platform = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='>>> Spawning landing platform at (2, 0, 0.15)...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_platform',
                output='screen',
                arguments=[
                    '-entity', 'platform',
                    '-file', platform_sdf,
                    '-x', '2.0', '-y', '0.0', '-z', '0.15',
                ],
            ),
        ],
    )

    # ── 3. Spawn Drone AFTER platform (12s) — camera scene includes platform ──
    spawn_drone = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='>>> Spawning drone at (0, 0, 3)...'),
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

    # ── 4. Platform Mover (15s) ──
    platform_mover = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='>>> Starting platform mover (random_walk)...'),
            Node(
                package='mars_platform',
                executable='platform_mover',
                name='platform_mover',
                output='screen',
                parameters=[{
                    'max_speed': 0.5,
                }],
            ),
        ],
    )

    # ── 5. ArUco Detector (17s) ──
    aruco_detector = TimerAction(
        period=17.0,
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

    # ── 6. Platform Tracker (18s) ──
    platform_tracker = TimerAction(
        period=18.0,
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

    # ── 7. Drone Controller (19s) ──
    drone_controller = TimerAction(
        period=19.0,
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
                    'descend_speed': 0.8,
                    'land_height': 0.55,
                }],
            ),
        ],
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_plugin_path,
        kill_gazebo,
        gazebo,
        spawn_platform,
        spawn_drone,
        platform_mover,
        aruco_detector,
        platform_tracker,
        drone_controller,
    ])
