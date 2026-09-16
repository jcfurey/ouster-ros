# SPDX-License-Identifier: BSD-3-Clause

"""View a live OSDome cloud and five calibrated pinhole cameras in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('ouster_ros'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(
                share / 'config' / 'os_pinhole_dome_params.yaml')),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(
                share / 'config' / 'pinhole_dome.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                share / 'launch' / 'pinhole_sensor.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ])
