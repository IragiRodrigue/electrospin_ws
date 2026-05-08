"""Robot controller standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('robot_controller').find('robot_controller')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),

        Node(
            package='robot_controller',
            executable='robot_controller_node',
            name='robot_controller',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'robot_controller.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'serial_port': LaunchConfiguration('serial_port')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
