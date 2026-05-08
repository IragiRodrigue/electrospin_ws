"""Dashboard UI standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('dashboard_ui').find('dashboard_ui')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),

        Node(
            package='dashboard_ui',
            executable='dashboard_node',
            name='dashboard',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'dashboard.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
