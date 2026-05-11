"""
ElectroSpin Platform — Master Bringup Launch File
===================================================
Launches the complete electrospinning platform in either:
  - Simulation mode (default, safe for testing)
  - Real hardware mode (requires physical devices)

Usage:
  ros2 launch electrospin_bringup electrospin_bringup.launch.py
  ros2 launch electrospin_bringup electrospin_bringup.launch.py simulation_mode:=false
  ros2 launch electrospin_bringup electrospin_bringup.launch.py optimization_mode:=adaptive
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, OpaqueFunction, LogInfo, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "simulation_mode", default_value="true",
            description="Run in simulation (true) or with real hardware (false)"
        ),
        DeclareLaunchArgument(
            "optimization_mode", default_value="adaptive",
            description="AI optimization mode: off|rule|adaptive|rl"
        ),
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="MyCobot serial port"
        ),
        DeclareLaunchArgument(
            "enable_dashboard", default_value="true",
            description="Launch industrial dashboard UI"
        ),
        DeclareLaunchArgument(
            "enable_simulation_env", default_value="true",
            description="Launch Gazebo + RViz simulation environment"
        ),
        DeclareLaunchArgument(
            "enable_voltage_control", default_value="false",
            description="Enable high-voltage controller (SAFETY: disabled by default)"
        ),
        DeclareLaunchArgument(
            "quality_target", default_value="0.75",
            description="AI quality optimization target (0.0–1.0)"
        ),
        DeclareLaunchArgument(
            "namespace", default_value="electrospin",
            description="ROS2 namespace for all nodes"
        ),
    ]

    # ── Shared substitutions ──────────────────────────────────────────────────
    sim_mode     = LaunchConfiguration("simulation_mode")
    opt_mode     = LaunchConfiguration("optimization_mode")
    serial_port  = LaunchConfiguration("serial_port")
    hv_enable    = LaunchConfiguration("enable_voltage_control")
    q_target     = LaunchConfiguration("quality_target")
    ns           = LaunchConfiguration("namespace")

    # ── Common QoS params ─────────────────────────────────────────────────────
    common_params = [
        {"simulation_mode": sim_mode},
    ]

    # ─────────────────────────────────────────────────────────────────────────
    # Core Nodes
    # ─────────────────────────────────────────────────────────────────────────

    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller_node",
        name="robot_controller",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"serial_port": serial_port},
            {"baud_rate": 115200},
            {"control_frequency": 10.0},
            {"enable_trajectory_smoothing": True},
            {"enable_singularity_avoidance": True},
        ],
        output="screen",
        emulate_tty=True,
    )

    collector_controller_node = Node(
        package="collector_controller",
        executable="collector_controller_node",
        name="collector_controller",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"control_frequency_hz": 50.0},
            {"publish_frequency_hz": 20.0},
            {"pid_kp": 0.8},
            {"pid_ki": 0.15},
            {"pid_kd": 0.05},
            {"max_rpm": 3000.0},
            {"ramp_rate_rpm_s": 100.0},
        ],
        output="screen",
        emulate_tty=True,
    )

    syringe_controller_node = Node(
        package="syringe_controller",
        executable="syringe_controller_node",
        name="syringe_pump",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"max_flow_ml_hr": 10.0},
            {"pressure_limit_kpa": 50.0},
            {"syringe_volume_ml": 20.0},
        ],
        output="screen",
        emulate_tty=True,
    )

    vision_system_node = Node(
        package="vision_system",
        executable="vision_system_node",
        name="vision_system",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"processing_fps": 10.0},
            {"use_yolo": False},
            {"temporal_smoothing": True},
            {"debug_visualization": True},
        ],
        output="screen",
        emulate_tty=True,
    )

    # Delayed start for AI controller (wait for sensors to warm up)
    ai_controller_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ai_controller",
                executable="ai_controller_node",
                name="ai_controller",
                namespace=ns,
                parameters=[
                    {"optimization_mode": opt_mode},
                    {"decision_frequency_hz": 2.0},
                    {"quality_target": q_target},
                    {"command_smoothing_alpha": 0.3},
                    {"enable_voltage_control": hv_enable},
                ],
                output="screen",
                emulate_tty=True,
            )
        ]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Dashboard UI
    # ─────────────────────────────────────────────────────────────────────────

    dashboard_node = Node(
        package="dashboard_ui",
        executable="dashboard_node",
        name="dashboard",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"window_title": "ElectroSpin Control Platform"},
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_dashboard")),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Simulation Environment (Gazebo + RViz)
    # ─────────────────────────────────────────────────────────────────────────

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("simulation_system"),
                "launch", "simulation.launch.py"
            ])
        ]),
        launch_arguments={
            "simulation_mode": sim_mode,
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_simulation_env")),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # System Monitor (watchdog + status aggregator)
    # ─────────────────────────────────────────────────────────────────────────

    system_monitor_node = Node(
        package="electrospin_bringup",
        executable="system_monitor_node",
        name="system_monitor",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"watchdog_timeout_s": 5.0},
        ],
        output="screen",
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Command Bridge (decomposes ElectrospinCommand to individual setpoints)
    # ─────────────────────────────────────────────────────────────────────────

    command_bridge_node = Node(
        package="electrospin_bringup",
        executable="command_bridge_node",
        name="command_bridge",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
        ],
        output="screen",
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Digital Twin Bridge (physics simulation for pump, collector, HV, sensors)
    # ─────────────────────────────────────────────────────────────────────────

    digital_twin_node = Node(
        package="digital_twin",
        executable="digital_twin_bridge",
        name="digital_twin_bridge",
        namespace=ns,
        parameters=[
            {"simulation_mode": sim_mode},
            {"update_rate_hz": 50.0},
        ],
        output="screen",
        condition=IfCondition(sim_mode),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Launch Description Assembly
    # ─────────────────────────────────────────────────────────────────────────

    log_start = LogInfo(msg=(
        "\n"
        "╔══════════════════════════════════════════════════════╗\n"
        "║     ElectroSpin Autonomous Platform — Bringup        ║\n"
        "║     ROS2 Nanofiber Fabrication System v1.0           ║\n"
        "╚══════════════════════════════════════════════════════╝\n"
    ))

    return LaunchDescription([
        *args,
        log_start,
        # Core nodes (parallel startup)
        robot_controller_node,
        collector_controller_node,
        syringe_controller_node,
        vision_system_node,
        # Delayed AI (wait for sensors)
        ai_controller_node,
        # UI
        dashboard_node,
        # Simulation
        simulation_launch,
        # System monitor
        system_monitor_node,
        # Command bridge
        command_bridge_node,
        # Digital twin (simulation only)
        digital_twin_node,
    ])
