"""Syringe pump controller standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('syringe_controller').find('syringe_controller')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),

        Node(
            package='syringe_controller',
            executable='syringe_controller_node',
            name='syringe_pump',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'syringe_controller.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
