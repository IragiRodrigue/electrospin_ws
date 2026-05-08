"""Human tracking standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('human_tracking').find('human_tracking')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('debug_visualization', default_value='true'),

        Node(
            package='human_tracking',
            executable='human_tracking_node',
            name='human_tracking',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'human_tracking.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'camera_index': LaunchConfiguration('camera_index')},
                {'debug_visualization': LaunchConfiguration('debug_visualization')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
