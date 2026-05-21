"""Motion mapping standalone launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('motion_mapping')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('teleoperation_arm', default_value='right'),
        DeclareLaunchArgument('scale_factor', default_value='0.5'),

        Node(
            package='motion_mapping',
            executable='motion_mapping_node',
            name='motion_mapping',
            namespace='electrospin',
            parameters=[
                os.path.join(pkg_share, 'config', 'motion_mapping.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'teleoperation_arm': LaunchConfiguration('teleoperation_arm')},
                {'scale_factor': LaunchConfiguration('scale_factor')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
