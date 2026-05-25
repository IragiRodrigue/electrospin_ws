"""Robot controller standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_controller')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),

        Node(
            package='robot_controller',
            executable='robot_controller_node',
            name='robot_controller',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'robot_controller.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': LaunchConfiguration('baud_rate')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
