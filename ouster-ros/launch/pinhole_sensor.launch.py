# SPDX-License-Identifier: BSD-3-Clause

"""View a live Ouster cloud and calibrated pinhole cameras in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ouster_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('sensor_hostname'),
        DeclareLaunchArgument('udp_dest', default_value=''),
        DeclareLaunchArgument('lidar_port', default_value='7502'),
        DeclareLaunchArgument('imu_port', default_value='7503'),
        DeclareLaunchArgument('metadata', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value=''),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=str(
                share / 'config' / 'os_pinhole_params.yaml')),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(
                share / 'config' / 'pinhole.rviz')),
        GroupAction([IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(
                share / 'launch' / 'sensor.composite.launch.xml')),
            launch_arguments={
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'udp_dest': LaunchConfiguration('udp_dest'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'imu_port': LaunchConfiguration('imu_port'),
                'metadata': LaunchConfiguration('metadata'),
                'timestamp_mode': LaunchConfiguration('timestamp_mode'),
                'ouster_ns': 'ouster',
                'proc_mask': 'IMU|PCL|RAW',
                'point_type': 'native',
                'persist_config': 'false',
                'viz': 'false',
            }.items(),
        )]),
        Node(
            package='ouster_ros', executable='os_pinhole',
            name='os_pinhole', namespace='ouster', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'timestamp_mode': ParameterValue(
                    LaunchConfiguration('timestamp_mode'), value_type=str),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            namespace='ouster', output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('viz')),
        ),
    ])
