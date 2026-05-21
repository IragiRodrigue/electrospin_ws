"""AI controller standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ai_controller')

    return LaunchDescription([
        DeclareLaunchArgument('optimization_mode', default_value='adaptive'),
        DeclareLaunchArgument('quality_target', default_value='0.75'),

        Node(
            package='ai_controller',
            executable='ai_controller_node',
            name='ai_controller',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'ai_controller.yaml'),
                {'optimization_mode': LaunchConfiguration('optimization_mode')},
                {'quality_target': LaunchConfiguration('quality_target')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
