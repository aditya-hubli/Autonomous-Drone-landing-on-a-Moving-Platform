"""
Single launch file for the entire MARS project.

Launches Gazebo with factory plugin, spawns models, starts all autonomy nodes,
and opens RViz with the drone camera feed.
Usage: ros2 launch mars_simulation mars_full_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
    TimerAction, LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mars_sim = get_package_share_directory('mars_simulation')
    models_path = os.path.join(pkg_mars_sim, 'models')
    install_lib = os.path.join(pkg_mars_sim, '..', '..', 'lib')
    world_file = os.path.join(pkg_mars_sim, 'worlds', 'mars_world.world')
    rviz_config = os.path.join(pkg_mars_sim, 'config', 'mars_rviz.rviz')

    drone_sdf = os.path.join(models_path, 'mars_drone', 'model.sdf')
    platform_sdf = os.path.join(models_path, 'landing_platform', 'model.sdf')

    # -- Environment --
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        install_lib + ':/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/drone/camera/image_raw',
        description='Camera image topic for the platform tracker')
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='drone/camera_link',
        description='Frame ID of the camera topic')

    # -- 0. Kill any stale Gazebo processes --
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c',
             'killall -9 gzserver gzclient gazebo 2>/dev/null; sleep 1; echo "Cleared old Gazebo"'],
        output='screen',
    )

    # -- 1. Gazebo (2s after cleanup) --
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

    # -- 2. Spawn Platform (8s) --
    spawn_platform = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='>>> Spawning landing platform at (1, 1, 0.15)...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_platform',
                output='screen',
                arguments=[
                    '-entity', 'platform',
                    '-file', platform_sdf,
                    '-x', '1.0', '-y', '1.0', '-z', '0.15',
                ],
            ),
        ],
    )

    # -- 3. Spawn Drone AFTER platform (12s) --
    spawn_drone = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='>>> Spawning drone at (0, 0, 5) — overview height...'),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_drone',
                output='screen',
                arguments=[
                    '-entity', 'drone',
                    '-file', drone_sdf,
                    '-x', '0.0', '-y', '0.0', '-z', '5.0',
                ],
            ),
        ],
    )

    # -- 4. Static camera transform (15s) --
    camera_tf = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='>>> Starting static camera transform...'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_static_tf',
                output='screen',
                arguments=['0', '0', '-0.06', '0', '0.7071068', '0', '0.7071068', 'drone/base_link', 'drone/camera_link'],
            ),
        ],
    )

    # -- 5. Platform Mover (20s) --
    platform_mover = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='>>> Starting platform mover (XY bounce)...'),
            Node(
                package='mars_platform',
                executable='platform_mover',
                name='platform_mover',
                output='screen',
                parameters=[{
                    'max_speed': 0.6,
                    'boundary': 2.5,
                }],
            ),
        ],
    )

    # -- 5. Platform Tracker (18s) --
    platform_tracker = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='>>> Starting platform tracker...'),
            Node(
                package='mars_tracking',
                executable='platform_tracker',
                name='platform_tracker',
                output='screen',
                parameters=[{
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                }],
            ),
        ],
    )

    # -- 7. Drone Controller (19s) --
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
                    'overview_height': 5.0,
                    'descend_speed': 0.4,
                    'land_height': 0.55,
                    'xy_tolerance': 0.2,
                }],
            ),
        ],
    )

    # -- 8. RViz with camera feed (20s) --
    rviz = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='>>> Starting RViz with drone camera feed...'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
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
        camera_tf,
        platform_mover,
        platform_tracker,
        drone_controller,
        rviz,
        camera_topic_arg,
        camera_frame_arg,
    ])
