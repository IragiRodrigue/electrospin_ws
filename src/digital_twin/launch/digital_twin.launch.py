"""Digital twin bridge standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('digital_twin').find('digital_twin')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),

        Node(
            package='digital_twin',
            executable='digital_twin_bridge',
            name='digital_twin_bridge',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'digital_twin.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
