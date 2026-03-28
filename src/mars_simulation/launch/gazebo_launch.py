"""Launch only Gazebo with the MARS world (ground + sun). Models spawned separately."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess


def generate_launch_description():
    pkg_mars_sim = get_package_share_directory('mars_simulation')
    models_path = os.path.join(pkg_mars_sim, 'models')
    install_lib = os.path.join(pkg_mars_sim, '..', '..', 'lib')
    world_file = os.path.join(pkg_mars_sim, 'worlds', 'mars_world.world')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            install_lib + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen',
        ),
    ])
