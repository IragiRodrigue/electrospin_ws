"""Collector controller standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('collector_controller').find('collector_controller')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),

        Node(
            package='collector_controller',
            executable='collector_controller_node',
            name='collector_controller',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'collector_controller.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
