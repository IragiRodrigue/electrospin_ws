"""Vision system standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_system')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('debug_visualization', default_value='true'),

        Node(
            package='vision_system',
            executable='vision_system_node',
            name='vision_system',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'vision_system.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'debug_visualization': LaunchConfiguration('debug_visualization')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
