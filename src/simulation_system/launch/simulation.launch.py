"""Simulation system launch — Gazebo + RViz + Robot State Publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import pathlib


def generate_launch_description():
    pkg_share = FindPackageShare('simulation_system').find('simulation_system')
    pkg_dir = get_package_share_directory('simulation_system')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'mycobot_280.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'electrospin_lab.world')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'electrospin.rviz')

    # Read URDF for robot_state_publisher
    try:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
    except FileNotFoundError:
        robot_description = '<robot name="empty"/>'

    return LaunchDescription([
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 30.0},
            ],
            output='screen',
        ),

        # Joint State Publisher (for simulation)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=lambda: LaunchConfiguration('gui') == 'true',
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gz_server.launch.py'
                )
            ]),
            launch_arguments={
                'world': world_file,
                'verbose': 'true',
            }.items(),
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gz_client.launch.py'
                )
            ]),
            launch_arguments={
                'world': world_file,
            }.items(),
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_mycobot',
            arguments=[
                '-entity', 'mycobot_280',
                '-topic', '/robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.42',
            ],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
