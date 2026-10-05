# SPDX-License-Identifier: BSD-3-Clause

"""Launch pinhole image panels for an existing Ouster packet stream."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('ouster_ros'))
    default_params = (
        package_share / 'config' / 'os_pinhole_params.yaml')

    namespace = LaunchConfiguration('ouster_ns')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('ouster_ns', default_value='ouster'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params),
            description='parameter file containing the os_pinhole settings',
        ),
        # Match these to the os_cloud/os_image producing the same stream so
        # pinhole images share the clouds' time domain (e.g. replay.composite
        # uses TIME_FROM_ROS_TIME with use_sim_time).
        DeclareLaunchArgument(
            'timestamp_mode', default_value='',
            description='timestamp mode; must match os_cloud for this stream',
        ),
        DeclareLaunchArgument('ptp_utc_tai_offset', default_value='-37.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ouster_ros',
            executable='os_pinhole',
            name='os_pinhole',
            namespace=namespace,
            parameters=[params_file, {
                'timestamp_mode': LaunchConfiguration('timestamp_mode'),
                'ptp_utc_tai_offset': ParameterValue(
                    LaunchConfiguration('ptp_utc_tai_offset'),
                    value_type=float),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            output='screen',
        ),
    ])
