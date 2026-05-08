#!/usr/bin/env python3
"""
ElectroSpin ROS2 Robot Controller Node
=======================================
Controls the MyCobot robotic arm for autonomous electrospinning.

Responsibilities:
- Maintains precise needle-to-collector distance
- Executes smooth scanning trajectories across collector surface
- Publishes joint states and robot status
- Integrates with MoveIt2 for trajectory planning
- Compensates for deposition non-uniformity via AI feedback

Architecture:
  /electrospin_command  →  [RobotControllerNode]  →  MyCobot hardware
                                     ↓
                           /joint_states, /robot_status, /needle_distance

Author: ElectroSpin Platform
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import threading
import time
import math
from typing import Optional, List, Tuple

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String, Float32, Bool, Header
from builtin_interfaces.msg import Time

from electrospin_interfaces.msg import ElectrospinCommand, SystemStatus

# Hardware abstraction - gracefully degrade in simulation mode
try:
    from pymycobot.mycobot import MyCobot
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

# MoveIt2 Python bindings
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        MotionPlanRequest, Constraints, JointConstraint,
        PositionConstraint, OrientationConstraint, BoundingVolume
    )
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Robot Controller Configuration
# ─────────────────────────────────────────────────────────────────────────────

class RobotConfig:
    """Central configuration for robot controller."""

    # MyCobot hardware
    SERIAL_PORT = "/dev/ttyUSB0"
    BAUD_RATE = 115200

    # Workspace limits (mm, in MyCobot frame)
    WORKSPACE_X_MIN = -250.0
    WORKSPACE_X_MAX =  250.0
    WORKSPACE_Y_MIN =  100.0
    WORKSPACE_Y_MAX =  400.0
    WORKSPACE_Z_MIN =   50.0
    WORKSPACE_Z_MAX =  300.0

    # Electrospinning geometry
    COLLECTOR_ORIGIN_X = 0.0    # mm
    COLLECTOR_ORIGIN_Y = 250.0  # mm
    COLLECTOR_ORIGIN_Z = 150.0  # mm
    COLLECTOR_WIDTH_MM = 60.0   # mm (collector drum width)

    # Default working parameters
    DEFAULT_DISTANCE_MM = 150.0   # needle-to-collector gap
    DEFAULT_SCAN_SPEED = 5.0      # mm/s lateral scan
    DEFAULT_SCAN_AMPLITUDE = 40.0 # mm peak-to-peak

    # Joint speed limits
    MAX_JOINT_SPEED = 30        # degrees/second (conservative for stability)
    SMOOTH_SPEED = 15           # degrees/second for precision moves
    TRAJECTORY_SPEED = 20       # degrees/second for scanning

    # Trajectory smoothing
    INTERPOLATION_STEPS = 20    # waypoints for smooth arcs
    BLEND_RADIUS = 5.0          # mm corner blending radius

    # Safety
    SINGULARITY_THRESHOLD = 0.01   # Determinant threshold
    JOINT_LIMIT_MARGIN = 5.0       # degrees safety margin from limits

    # Joint limits (degrees) for MyCobot 280
    JOINT_LIMITS = [
        (-165, 165),  # J1
        (-165, 165),  # J2
        (-165, 165),  # J3
        (-165, 165),  # J4
        (-165, 165),  # J5
        (-175, 175),  # J6
    ]

    # Home position (safe resting pose)
    HOME_ANGLES = [0.0, -30.0, 90.0, 0.0, 90.0, 0.0]

    # Electrospinning ready pose (facing collector)
    READY_ANGLES = [0.0, 20.0, -20.0, 0.0, -90.0, 0.0]


# ─────────────────────────────────────────────────────────────────────────────
# Trajectory Generator
# ─────────────────────────────────────────────────────────────────────────────

class TrajectoryGenerator:
    """
    Generates smooth scanning trajectories for uniform fiber deposition.
    Supports: linear scan, sinusoidal, coverage-optimized, and custom paths.
    """

    def __init__(self, config: RobotConfig):
        self.config = config

    def generate_linear_scan(
        self,
        center: Tuple[float, float, float],
        amplitude: float,
        num_passes: int = 1,
        steps_per_pass: int = 50
    ) -> List[Tuple[float, float, float]]:
        """Generate linear back-and-forth scanning waypoints."""
        waypoints = []
        cx, cy, cz = center

        for pass_idx in range(num_passes):
            # Alternate scan direction each pass
            x_start = cx - amplitude / 2.0
            x_end   = cx + amplitude / 2.0

            if pass_idx % 2 == 1:
                x_start, x_end = x_end, x_start

            for step in range(steps_per_pass):
                t = step / (steps_per_pass - 1)
                x = x_start + t * (x_end - x_start)
                waypoints.append((x, cy, cz))

        return waypoints

    def generate_sinusoidal_scan(
        self,
        center: Tuple[float, float, float],
        amplitude: float,
        duration_s: float,
        frequency_hz: float = 0.1,
        dt: float = 0.1
    ) -> List[Tuple[float, float, float]]:
        """Generate sinusoidal scanning path for smooth motion."""
        waypoints = []
        cx, cy, cz = center
        t = 0.0
        while t <= duration_s:
            x = cx + amplitude / 2.0 * math.sin(2 * math.pi * frequency_hz * t)
            waypoints.append((x, cy, cz))
            t += dt
        return waypoints

    def smooth_waypoints(
        self,
        waypoints: List[Tuple[float, float, float]],
        window: int = 5
    ) -> List[Tuple[float, float, float]]:
        """Apply moving average smoothing to waypoints."""
        if len(waypoints) <= window:
            return waypoints

        smoothed = []
        arr = np.array(waypoints)
        for i in range(len(arr)):
            start = max(0, i - window // 2)
            end   = min(len(arr), i + window // 2 + 1)
            smoothed.append(tuple(arr[start:end].mean(axis=0)))
        return smoothed

    def compensate_non_uniformity(
        self,
        waypoints: List[Tuple[float, float, float]],
        coverage_map: np.ndarray,
        collector_width: float
    ) -> List[Tuple[float, float, float]]:
        """
        Adjust waypoint dwell time / density based on coverage map.
        Regions with low deposition get more scan time.
        """
        if coverage_map is None or coverage_map.size == 0:
            return waypoints

        adjusted = []
        for wp in waypoints:
            x, y, z = wp
            # Map x position to coverage map index
            map_idx = int(
                (x + collector_width / 2.0) / collector_width * (len(coverage_map) - 1)
            )
            map_idx = np.clip(map_idx, 0, len(coverage_map) - 1)
            coverage = coverage_map[map_idx]

            # Duplicate waypoint proportionally to coverage deficit
            repeats = max(1, int(2.0 * (1.0 - float(coverage))))
            adjusted.extend([wp] * repeats)

        return adjusted


# ─────────────────────────────────────────────────────────────────────────────
# MyCobot Hardware Abstraction Layer
# ─────────────────────────────────────────────────────────────────────────────

class MyCobotHAL:
    """
    Hardware Abstraction Layer for MyCobot 280.
    Falls back to simulation if hardware is unavailable.
    """

    def __init__(self, port: str, baud: int, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode or not HARDWARE_AVAILABLE
        self._joint_angles = [0.0] * 6
        self._lock = threading.Lock()

        if not self.simulation_mode:
            try:
                self._robot = MyCobot(port, baud)
                time.sleep(0.5)
                self._robot.power_on()
            except Exception as e:
                print(f"[HAL] Hardware init failed: {e}. Falling back to simulation.")
                self.simulation_mode = True
                self._robot = None
        else:
            self._robot = None

    def get_joint_angles(self) -> List[float]:
        """Returns current joint angles in degrees."""
        with self._lock:
            if not self.simulation_mode and self._robot:
                try:
                    angles = self._robot.get_angles()
                    if angles:
                        self._joint_angles = list(angles)
                except Exception:
                    pass
            return list(self._joint_angles)

    def send_joint_angles(self, angles: List[float], speed: int = 20) -> bool:
        """Send target joint angles to robot."""
        with self._lock:
            try:
                if not self.simulation_mode and self._robot:
                    self._robot.send_angles(angles, speed)
                # Update simulated state
                self._joint_angles = list(angles)
                return True
            except Exception as e:
                print(f"[HAL] send_angles failed: {e}")
                return False

    def send_coords(
        self,
        coords: List[float],   # [x, y, z, rx, ry, rz]
        speed: int = 20,
        mode: int = 2
    ) -> bool:
        """Send Cartesian target coordinates."""
        with self._lock:
            try:
                if not self.simulation_mode and self._robot:
                    self._robot.send_coords(coords, speed, mode)
                return True
            except Exception as e:
                print(f"[HAL] send_coords failed: {e}")
                return False

    def release_servos(self):
        if not self.simulation_mode and self._robot:
            try:
                self._robot.release_all_servos()
            except Exception:
                pass

    def power_off(self):
        if not self.simulation_mode and self._robot:
            try:
                self._robot.power_off()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Robot Controller Node
# ─────────────────────────────────────────────────────────────────────────────

class RobotControllerNode(Node):
    """
    Main ROS2 node for MyCobot electrospinning robot control.

    Topics Published:
        /joint_states          → sensor_msgs/JointState
        /robot_status          → std_msgs/String (JSON)
        /needle_distance       → std_msgs/Float32

    Topics Subscribed:
        /electrospin_command   → electrospin_interfaces/ElectrospinCommand
    """

    JOINT_NAMES = [
        "joint1_link", "joint2_link", "joint3_link",
        "joint4_link", "joint5_link", "joint6_link"
    ]

    def __init__(self):
        super().__init__("robot_controller")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("serial_port", RobotConfig.SERIAL_PORT)
        self.declare_parameter("baud_rate", RobotConfig.BAUD_RATE)
        self.declare_parameter("control_frequency", 10.0)  # Hz
        self.declare_parameter("default_distance_mm", RobotConfig.DEFAULT_DISTANCE_MM)
        self.declare_parameter("scan_amplitude_mm", RobotConfig.DEFAULT_SCAN_AMPLITUDE)
        self.declare_parameter("scan_speed_mms", RobotConfig.DEFAULT_SCAN_SPEED)
        self.declare_parameter("enable_trajectory_smoothing", True)
        self.declare_parameter("enable_singularity_avoidance", True)

        self.sim_mode = self.get_parameter("simulation_mode").value
        port          = self.get_parameter("serial_port").value
        baud          = self.get_parameter("baud_rate").value
        ctrl_freq     = self.get_parameter("control_frequency").value

        # ── Hardware ─────────────────────────────────────────────────────────
        self.hal = MyCobotHAL(port, baud, simulation_mode=self.sim_mode)
        self.traj_gen = TrajectoryGenerator(RobotConfig())
        self.config = RobotConfig()

        # ── State ─────────────────────────────────────────────────────────────
        self.current_distance_mm: float = self.config.DEFAULT_DISTANCE_MM
        self.target_distance_mm:  float = self.config.DEFAULT_DISTANCE_MM
        self.scan_speed_mms:      float = self.config.DEFAULT_SCAN_SPEED
        self.scan_amplitude_mm:   float = self.config.DEFAULT_SCAN_AMPLITUDE
        self.needle_x_mm:         float = 0.0
        self.scan_phase:          float = 0.0
        self.robot_state:         str   = "IDLE"
        self.coverage_map         = np.zeros(64)   # 64-zone coverage tracking
        self.last_cmd_time        = self.get_clock().now()

        # ── Callback groups ───────────────────────────────────────────────────
        self.cb_group_reentrant = ReentrantCallbackGroup()
        self.cb_group_exclusive = MutuallyExclusiveCallbackGroup()

        # ── QoS Profiles ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_joint_states = self.create_publisher(
            JointState, "/joint_states", sensor_qos
        )
        self.pub_robot_status = self.create_publisher(
            String, "/robot_status", reliable_qos
        )
        self.pub_needle_distance = self.create_publisher(
            Float32, "/needle_distance", sensor_qos
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.sub_command = self.create_subscription(
            ElectrospinCommand,
            "/electrospin_command",
            self._on_electrospin_command,
            reliable_qos,
            callback_group=self.cb_group_reentrant
        )

        # ── Timers ────────────────────────────────────────────────────────────
        self.control_timer = self.create_timer(
            1.0 / ctrl_freq,
            self._control_loop,
            callback_group=self.cb_group_exclusive
        )
        self.publish_timer = self.create_timer(
            0.05,  # 20 Hz publishing
            self._publish_states,
            callback_group=self.cb_group_reentrant
        )

        self.get_logger().info(
            f"[RobotController] Initialized. Simulation={self.sim_mode}, "
            f"Port={port}@{baud}"
        )

        # Move to ready position asynchronously
        threading.Thread(target=self._initialize_pose, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    # Command Handler
    # ─────────────────────────────────────────────────────────────────────────

    def _on_electrospin_command(self, msg: ElectrospinCommand):
        """Handle incoming electrospinning command from AI controller."""
        self.last_cmd_time = self.get_clock().now()

        # Update setpoints
        if msg.target_distance > 0:
            self.target_distance_mm = float(msg.target_distance)

        if msg.target_scan_speed > 0:
            self.scan_speed_mms = float(msg.target_scan_speed)

        if msg.scan_amplitude > 0:
            self.scan_amplitude_mm = float(msg.scan_amplitude)

        self.get_logger().debug(
            f"[RobotController] CMD → dist={self.target_distance_mm:.1f}mm "
            f"scan={self.scan_speed_mms:.1f}mm/s "
            f"amp={self.scan_amplitude_mm:.1f}mm"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Control Loop
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """Main control loop: updates robot position at ctrl_frequency Hz."""
        if self.robot_state == "IDLE":
            return

        try:
            # Advance scan phase based on speed
            dt = 0.1  # approx control period
            phase_advance = (self.scan_speed_mms / self.scan_amplitude_mm) * dt
            self.scan_phase += phase_advance

            # Sinusoidal scanning
            self.needle_x_mm = (
                self.scan_amplitude_mm / 2.0 * math.sin(self.scan_phase)
            )

            # Smooth distance tracking
            dist_error = self.target_distance_mm - self.current_distance_mm
            self.current_distance_mm += 0.1 * dist_error  # first-order filter

            # Compute target Cartesian position
            target_x = self.config.COLLECTOR_ORIGIN_X + self.needle_x_mm
            target_y = (
                self.config.COLLECTOR_ORIGIN_Y - self.current_distance_mm
            )
            target_z = self.config.COLLECTOR_ORIGIN_Z

            # Send to hardware
            coords = [target_x, target_y, target_z, 0.0, -90.0, 0.0]
            self.hal.send_coords(coords, speed=self.config.TRAJECTORY_SPEED)

            # Update coverage map
            map_idx = int(
                (self.needle_x_mm + self.scan_amplitude_mm / 2.0)
                / self.scan_amplitude_mm
                * (len(self.coverage_map) - 1)
            )
            map_idx = int(np.clip(map_idx, 0, len(self.coverage_map) - 1))
            self.coverage_map[map_idx] = min(1.0, self.coverage_map[map_idx] + 0.01)

        except Exception as e:
            self.get_logger().error(f"[RobotController] Control loop error: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # State Publishers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_states(self):
        """Publish joint states and needle distance at 20 Hz."""
        now = self.get_clock().now().to_msg()
        angles = self.hal.get_joint_angles()

        # ── JointState ───────────────────────────────────────────────────────
        js = JointState()
        js.header.stamp = now
        js.name = self.JOINT_NAMES
        js.position = [math.radians(a) for a in angles]
        js.velocity = [0.0] * 6
        js.effort   = [0.0] * 6
        self.pub_joint_states.publish(js)

        # ── Needle Distance ───────────────────────────────────────────────────
        nd = Float32()
        nd.data = float(self.current_distance_mm)
        self.pub_needle_distance.publish(nd)

        # ── Robot Status ──────────────────────────────────────────────────────
        import json
        status = {
            "state":          self.robot_state,
            "sim_mode":       self.sim_mode,
            "distance_mm":    round(self.current_distance_mm, 2),
            "target_dist_mm": round(self.target_distance_mm, 2),
            "needle_x_mm":    round(self.needle_x_mm, 2),
            "scan_speed":     round(self.scan_speed_mms, 2),
            "scan_amp":       round(self.scan_amplitude_mm, 2),
            "coverage_mean":  round(float(self.coverage_map.mean()), 3),
            "joint_angles":   [round(a, 2) for a in angles],
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_robot_status.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Initialization
    # ─────────────────────────────────────────────────────────────────────────

    def _initialize_pose(self):
        """Move robot to electrospinning ready position."""
        time.sleep(1.0)
        self.robot_state = "HOMING"
        self.get_logger().info("[RobotController] Moving to home position...")
        self.hal.send_joint_angles(
            self.config.HOME_ANGLES, speed=self.config.SMOOTH_SPEED
        )
        time.sleep(3.0)
        self.get_logger().info("[RobotController] Moving to ready position...")
        self.hal.send_joint_angles(
            self.config.READY_ANGLES, speed=self.config.SMOOTH_SPEED
        )
        time.sleep(3.0)
        self.robot_state = "READY"
        self.get_logger().info("[RobotController] Ready for electrospinning.")

    def destroy_node(self):
        """Safe shutdown."""
        self.hal.release_servos()
        time.sleep(0.5)
        self.hal.power_off()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()