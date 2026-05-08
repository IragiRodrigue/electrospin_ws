"""Combined teleoperation launch — human tracking + motion mapping."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    human_pkg = FindPackageShare('human_tracking').find('human_tracking')
    motion_pkg = FindPackageShare('motion_mapping').find('motion_mapping')

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('teleoperation_arm', default_value='right'),
        DeclareLaunchArgument('scale_factor', default_value='0.5'),
        DeclareLaunchArgument('debug_visualization', default_value='true'),

        # Human tracking node
        Node(
            package='human_tracking',
            executable='human_tracking_node',
            name='human_tracking',
            namespace='electrospin',
            parameters=[
                os.path.join(human_pkg, 'config', 'human_tracking.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'camera_index': LaunchConfiguration('camera_index')},
                {'debug_visualization': LaunchConfiguration('debug_visualization')},
            ],
            output='screen',
            emulate_tty=True,
        ),

        # Motion mapping node
        Node(
            package='motion_mapping',
            executable='motion_mapping_node',
            name='motion_mapping',
            namespace='electrospin',
            parameters=[
                os.path.join(motion_pkg, 'config', 'motion_mapping.yaml'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')},
                {'teleoperation_arm': LaunchConfiguration('teleoperation_arm')},
                {'scale_factor': LaunchConfiguration('scale_factor')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
