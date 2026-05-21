"""Dashboard UI standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dashboard_ui')

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
