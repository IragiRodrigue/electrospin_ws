"""Simulation system launch for Gazebo, RViz, and robot description publishing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def resolve_robot_urdf(pkg_dir: str, variant: str) -> str:
    """Resolve the robot URDF, preferring official Elephant Robotics descriptions."""
    internal_urdf = os.path.join(pkg_dir, "urdf", "mycobot_280.urdf")
    official_variants = {
        "mycobot_280_jn": ("mycobot_280_jn", "mycobot_280_jn.urdf"),
        "mycobot_280_riscv": ("mycobot_280_riscv", "mycobot_280_riscv.urdf"),
        "mycobot_280_x3pi": ("mycobot_280_x3pi", "mycobot_280_x3pi.urdf"),
        "mycobot_280_rdkx5": ("mycobot_280_rdkx5", "mycobot_280_rdkx5.urdf"),
        "mycobot_280_arduino": ("mycobot_280_arduino", "mycobot_280_arduino.urdf"),
    }

    if variant == "internal":
        return internal_urdf

    if variant in official_variants:
        try:
            desc_dir = get_package_share_directory("mycobot_description")
            subdir, filename = official_variants[variant]
            candidate = os.path.join(desc_dir, "urdf", subdir, filename)
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

    return internal_urdf


def resolve_spawn_pose(variant: str) -> dict:
    """Return a visually clean default spawn pose for each robot variant."""
    defaults = {
        "internal": {"x": "0.00", "y": "-0.12", "z": "0.42", "yaw": "1.5708"},
        "mycobot_280_jn": {"x": "0.00", "y": "-0.12", "z": "0.46", "yaw": "1.5708"},
        "mycobot_280_riscv": {"x": "0.00", "y": "-0.12", "z": "0.46", "yaw": "1.5708"},
        "mycobot_280_x3pi": {"x": "0.00", "y": "-0.12", "z": "0.46", "yaw": "1.5708"},
        "mycobot_280_rdkx5": {"x": "0.00", "y": "-0.12", "z": "0.46", "yaw": "1.5708"},
        "mycobot_280_arduino": {"x": "0.00", "y": "-0.12", "z": "0.46", "yaw": "1.5708"},
    }
    return defaults.get(variant, defaults["internal"])


def resolve_gazebo_launch_file(base_dir: str, candidates):
    """Pick the first Gazebo launch filename that exists on this ROS distro."""
    for name in candidates:
        candidate = os.path.join(base_dir, name)
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(
        "Could not find any Gazebo launch file in '{}': {}".format(
            base_dir, ", ".join(candidates)
        )
    )


def runtime_nodes(context, pkg_dir: str, world_file: str, rviz_file: str):
    robot_variant = LaunchConfiguration("robot_model_variant").perform(context)
    urdf_file = resolve_robot_urdf(pkg_dir, robot_variant)
    auto_pose = resolve_spawn_pose(robot_variant)

    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    spawn_x = auto_pose["x"] if spawn_x == "auto" else spawn_x
    spawn_y = auto_pose["y"] if spawn_y == "auto" else spawn_y
    spawn_z = auto_pose["z"] if spawn_z == "auto" else spawn_z
    spawn_yaw = auto_pose["yaw"] if spawn_yaw == "auto" else spawn_yaw
    gazebo_launch_dir = os.path.join(get_package_share_directory("gazebo_ros"), "launch")
    gzserver_launch = resolve_gazebo_launch_file(
        gazebo_launch_dir,
        ["gzserver.launch.py", "gz_server.launch.py"],
    )
    gzclient_launch = resolve_gazebo_launch_file(
        gazebo_launch_dir,
        ["gzclient.launch.py", "gz_client.launch.py"],
    )

    return [
        LogInfo(msg=f"[simulation_system] Robot model variant: {robot_variant}"),
        LogInfo(msg=f"[simulation_system] Robot URDF: {urdf_file}"),
        LogInfo(msg=f"[simulation_system] Spawn pose: x={spawn_x}, y={spawn_y}, z={spawn_z}, yaw={spawn_yaw}"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command([
                        FindExecutable(name="xacro"),
                        " ",
                        urdf_file,
                    ])
                },
                {"publish_frequency": 30.0},
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                gzserver_launch
            ]),
            launch_arguments={
                "world": world_file,
                "verbose": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gzclient_launch]),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_mycobot",
            arguments=[
                "-entity", "mycobot_280",
                "-topic", "/robot_description",
                "-x", spawn_x,
                "-y", spawn_y,
                "-z", spawn_z,
                "-Y", spawn_yaw,
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_file],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory("simulation_system")
    world_file = os.path.join(pkg_dir, "worlds", "electrospin_lab.world")
    rviz_file = os.path.join(pkg_dir, "rviz", "electrospin.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("simulation_mode", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument(
            "robot_model_variant",
            default_value="internal",
            description="Robot model variant: internal, mycobot_280_jn, mycobot_280_riscv, mycobot_280_x3pi, mycobot_280_rdkx5, mycobot_280_arduino",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="auto",
            description="Robot spawn X position in world coordinates, or auto",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="auto",
            description="Robot spawn Y position in world coordinates, or auto",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="auto",
            description="Robot spawn Z position in world coordinates, or auto",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="auto",
            description="Robot spawn yaw in radians, or auto",
        ),
        OpaqueFunction(function=lambda context: runtime_nodes(context, pkg_dir, world_file, rviz_file)),
    ])
