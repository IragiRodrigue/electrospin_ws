#!/usr/bin/env python3
"""
Motion Mapping Node — Human-to-Robot Kinematic Mapping
========================================================
Converts human pose data into robot-compatible joint commands with
inverse kinematics, safety enforcement, and trajectory smoothing.

Responsibilities:
  - Map human arm pose to MyCobot joint angles
  - Apply analytical inverse kinematics
  - Enforce robot joint limits
  - Smooth trajectories (EMA + velocity limiting)
  - Reduce jitter with low-pass filtering
  - Avoid unsafe robot motion
  - Scale between human and robot arm dimensions
  - Compensate for latency with motion prediction

Topics Subscribed:
  /human_pose       → HumanPose
  /hand_gesture      → HandGesture
  /emergency_stop    → Bool

Topics Published:
  /motion_command    → MotionCommand
  /motion_status      → String (JSON)

Parameters:
  teleoperation_arm     — Which arm to track: "left", "right", "auto"
  scale_factor          — Human-to-robot motion scaling (0.0–1.0)
  smoothing_alpha       — EMA smoothing factor (0=heavy, 1=no smoothing)
  max_joint_velocity    — Maximum joint velocity (rad/s)
  prediction_steps      — Motion prediction lookahead (0=disabled)
  enable_safety         — Enable joint limit and velocity checks
  workspace_radius      — Robot workspace radius limit (m)

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
import json
import time
from typing import Optional, List, Tuple
from enum import IntEnum

from std_msgs.msg import String, Bool
from electrospin_interfaces.msg import HumanPose, HandGesture, MotionCommand


# ─────────────────────────────────────────────────────────────────────────────
# MyCobot 280 Joint Limits (radians)
# ─────────────────────────────────────────────────────────────────────────────

MYCABOT_JOINT_LIMITS = [
    (-2.88, 2.88),   # Joint 1 (base rotation)
    (-2.88, 2.88),   # Joint 2 (shoulder)
    (-2.88, 2.88),   # Joint 3 (elbow)
    (-2.88, 2.88),   # Joint 4 (wrist 1)
    (-3.05, 3.05),   # Joint 5 (wrist 2)
    (-3.05, 3.05),   # Joint 6 (wrist 3)
]

# MyCobot 280 link lengths (meters)
L1 = 0.072   # Base to shoulder
L2 = 0.110   # Shoulder to elbow
L3 = 0.110   # Elbow to wrist 1
L4 = 0.075   # Wrist 1 to wrist 2
L5 = 0.055   # Wrist 2 to flange
L6 = 0.030   # Flange to needle tip

# Human arm reference lengths (meters) for scaling
HUMAN_UPPER_ARM = 0.30   # Shoulder to elbow
HUMAN_FOREARM   = 0.26   # Elbow to wrist


# ─────────────────────────────────────────────────────────────────────────────
# Smoothing Filters
# ─────────────────────────────────────────────────────────────────────────────

class EMASmoother:
    """Exponential Moving Average filter for joint angle smoothing."""

    def __init__(self, size: int, alpha: float = 0.3):
        self.size = size
        self.alpha = alpha
        self.value: Optional[np.ndarray] = None

    def filter(self, sample: np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = sample.copy()
            return self.value.copy()
        self.value = self.alpha * sample + (1.0 - self.alpha) * self.value
        return self.value.copy()

    def reset(self):
        self.value = None


class VelocityLimiter:
    """Limit joint velocity to prevent abrupt motion."""

    def __init__(self, size: int, max_vel: float, dt: float):
        self.size = size
        self.max_vel = max_vel
        self.dt = dt
        self.prev: Optional[np.ndarray] = None

    def limit(self, target: np.ndarray) -> np.ndarray:
        if self.prev is None:
            self.prev = target.copy()
            return target.copy()

        delta = target - self.prev
        max_delta = self.max_vel * self.dt
        clipped = np.clip(delta, -max_delta, max_delta)
        result = self.prev + clipped
        self.prev = result.copy()
        return result

    def reset(self):
        self.prev = None


class MotionPredictor:
    """Simple linear extrapolation for latency compensation."""

    def __init__(self, size: int, steps: int = 1):
        self.size = size
        self.steps = steps
        self.history: List[np.ndarray] = []
        self.max_history = 5

    def predict(self, sample: np.ndarray) -> np.ndarray:
        self.history.append(sample.copy())
        if len(self.history) > self.max_history:
            self.history.pop(0)

        if len(self.history) < 2 or self.steps <= 0:
            return sample.copy()

        # Linear extrapolation from last two samples
        velocity = self.history[-1] - self.history[-2]
        predicted = sample + velocity * self.steps
        return predicted

    def reset(self):
        self.history.clear()


# ─────────────────────────────────────────────────────────────────────────────
# Analytical Inverse Kinematics for MyCobot 280
# ─────────────────────────────────────────────────────────────────────────────

class MyCobotIK:
    """
    Analytical inverse kinematics solver for MyCobot 280.

    Maps a target end-effector position (x, y, z) in the robot base frame
    to joint angles (j1–j6). Uses geometric IK for the first 3 joints
    and heuristic wrist orientation for joints 4–6.
    """

    @staticmethod
    def solve(target_pos: np.ndarray, elbow_preference: str = "up") -> np.ndarray:
        """
        Solve IK for target position [x, y, z] in robot base frame.

        Returns 6 joint angles in radians. Returns zeros if unreachable.
        """
        x, y, z = target_pos

        # Joint 1: Base rotation (yaw)
        j1 = math.atan2(y, x + 1e-8)

        # Distance in the horizontal plane from base axis
        r = math.sqrt(x**2 + y**2)

        # Wrist position (approximate — ignoring wrist orientation)
        wrist_z = z - L5 - L6
        wrist_r = r - L4 * 0.5  # Approximate wrist offset

        # Effective reach for the 2-link arm (L2 + L3)
        d = math.sqrt(wrist_r**2 + wrist_z**2)
        reach = L2 + L3

        if d > reach * 0.98:
            # Target too far — extend fully
            j2 = math.atan2(wrist_r, wrist_z + 1e-8)
            j3 = 0.0
        elif d < abs(L2 - L3) * 1.02:
            # Target too close — fold
            j2 = math.atan2(wrist_r, wrist_z + 1e-8)
            j3 = math.pi * 0.8
        else:
            # Law of cosines for elbow angle
            cos_j3 = (L2**2 + L3**2 - d**2) / (2 * L2 * L3 + 1e-8)
            cos_j3 = np.clip(cos_j3, -1.0, 1.0)
            j3 = math.pi - math.acos(cos_j3)

            if elbow_preference == "down":
                j3 = -j3

            # Shoulder angle
            alpha = math.atan2(wrist_z, wrist_r + 1e-8)
            beta = math.atan2(L3 * math.sin(j3), L2 + L3 * math.cos(j3))
            j2 = alpha + beta

        # Wrist joints: maintain downward orientation for electrospinning
        j4 = -j2 - j3 + math.pi * 0.5  # Keep needle pointing down
        j5 = 0.0                         # No wrist rotation
        j6 = 0.0                         # No flange rotation

        angles = np.array([j1, j2, j3, j4, j5, j6])

        # Clamp to joint limits
        for i in range(6):
            lo, hi = MYCABOT_JOINT_LIMITS[i]
            angles[i] = np.clip(angles[i], lo, hi)

        return angles

    @staticmethod
    def forward(angles: np.ndarray) -> np.ndarray:
        """Forward kinematics — compute end-effector position from joint angles."""
        j1, j2, j3, j4, j5, j6 = angles

        # Simplified FK
        r = (L2 * math.sin(j2) + L3 * math.sin(j2 + j3) +
             L4 * math.sin(j2 + j3 + j4) + L5 * math.sin(j2 + j3 + j4))
        x = r * math.cos(j1)
        y = r * math.sin(j1)
        z = (L1 + L2 * math.cos(j2) + L3 * math.cos(j2 + j3) +
             L4 * math.cos(j2 + j3 + j4) + L5 * math.cos(j2 + j3 + j4))

        return np.array([x, y, z])


# ─────────────────────────────────────────────────────────────────────────────
# Human-to-Robot Pose Mapper
# ─────────────────────────────────────────────────────────────────────────────

class HumanRobotMapper:
    """
    Maps human arm pose to robot joint space.

    Strategy:
      1. Extract human shoulder/elbow/wrist positions
      2. Compute human arm angles (shoulder flexion, elbow flexion)
      3. Scale angles to robot workspace
      4. Apply IK to get robot joint angles
      5. Optionally use direct angle mapping for teleoperation
    """

    def __init__(self, scale_factor: float = 0.5):
        self.scale_factor = scale_factor

    def map_pose_to_angles(self, pose_msg: HumanPose, arm: str = "right") -> np.ndarray:
        """
        Map HumanPose to MyCobot joint angles.

        Uses a hybrid approach:
          - Joint 1 (base): derived from shoulder midpoint x-position
          - Joints 2-3 (shoulder/elbow): mapped from human arm angles
          - Joints 4-6 (wrist): maintain downward orientation
        """
        if arm == "left":
            shoulder_angle = pose_msg.left_shoulder_angle
            elbow_angle = pose_msg.left_elbow_angle
            shoulder_pos = np.array(pose_msg.left_shoulder_position)
            wrist_pos = np.array(pose_msg.left_wrist_position)
            visibility = pose_msg.left_shoulder_visibility
        else:
            shoulder_angle = pose_msg.right_shoulder_angle
            elbow_angle = pose_msg.right_elbow_angle
            shoulder_pos = np.array(pose_msg.right_shoulder_position)
            wrist_pos = np.array(pose_msg.right_wrist_position)
            visibility = pose_msg.right_shoulder_visibility

        if visibility < 0.3:
            return None

        # Joint 1: Base rotation from wrist x-position
        # Map human x (0–1 normalized) to robot base rotation
        j1 = (wrist_pos[0] - 0.5) * 2.0 * self.scale_factor
        j1 = np.clip(j1, MYCABOT_JOINT_LIMITS[0][0], MYCABOT_JOINT_LIMITS[0][1])

        # Joint 2: Shoulder — map human shoulder angle
        # Human shoulder angle 0–pi maps to robot shoulder range
        j2 = shoulder_angle * self.scale_factor
        j2 = np.clip(j2, MYCABOT_JOINT_LIMITS[1][0], MYCABOT_JOINT_LIMITS[1][1])

        # Joint 3: Elbow — map human elbow angle
        # Human elbow angle 0–pi maps to robot elbow
        j3 = elbow_angle * self.scale_factor * 0.8
        j3 = np.clip(j3, MYCABOT_JOINT_LIMITS[2][0], MYCABOT_JOINT_LIMITS[2][1])

        # Joints 4-6: Wrist — maintain downward orientation for needle
        j4 = -j2 - j3 + math.pi * 0.5
        j4 = np.clip(j4, MYCABOT_JOINT_LIMITS[3][0], MYCABOT_JOINT_LIMITS[3][1])
        j5 = 0.0
        j6 = 0.0

        return np.array([j1, j2, j3, j4, j5, j6])

    def map_pose_to_position(self, pose_msg: HumanPose, arm: str = "right") -> Optional[np.ndarray]:
        """
        Map human wrist position to robot end-effector target position.

        Scales the human wrist position into the robot workspace.
        """
        if arm == "left":
            wrist = np.array(pose_msg.left_wrist_position)
            vis = pose_msg.left_wrist_visibility
        else:
            wrist = np.array(pose_msg.right_wrist_position)
            vis = pose_msg.right_wrist_visibility

        if vis < 0.3:
            return None

        # Scale: MediaPipe gives normalized 0–1 coords
        # Map to robot workspace (approx 0–0.3m reach)
        target = np.array([
            (wrist[0] - 0.5) * 0.3 * self.scale_factor,
            (wrist[1] - 0.5) * 0.3 * self.scale_factor,
            (0.8 - wrist[1]) * 0.3 * self.scale_factor + 0.1,
        ])

        return target


# ─────────────────────────────────────────────────────────────────────────────
# Safety Checker
# ─────────────────────────────────────────────────────────────────────────────

class SafetyChecker:
    """Validates motion commands against safety constraints."""

    def __init__(self, workspace_radius: float = 0.35):
        self.workspace_radius = workspace_radius
        self._estop_active = False

    def set_estop(self, active: bool):
        self._estop_active = active

    def check(self, joint_angles: np.ndarray, prev_angles: Optional[np.ndarray] = None,
              dt: float = 0.033) -> Tuple[bool, np.ndarray, str]:
        """
        Check if a joint angle command is safe.

        Returns (is_safe, clamped_angles, reason).
        """
        if self._estop_active:
            return False, np.zeros(6), "E-STOP active"

        safe = np.zeros(6)
        reason = ""

        for i in range(6):
            lo, hi = MYCABOT_JOINT_LIMITS[i]
            if joint_angles[i] < lo or joint_angles[i] > hi:
                safe[i] = np.clip(joint_angles[i], lo, hi)
                reason += f"J{i+1} limit clamped; "
            else:
                safe[i] = joint_angles[i]

        # Velocity check
        if prev_angles is not None:
            max_vel = 2.0  # rad/s
            for i in range(6):
                vel = abs(safe[i] - prev_angles[i]) / dt
                if vel > max_vel:
                    max_delta = max_vel * dt
                    safe[i] = prev_angles[i] + np.sign(safe[i] - prev_angles[i]) * max_delta
                    reason += f"J{i+1} velocity limited; "

        # Workspace check (approximate)
        fk = MyCobotIK.forward(safe)
        dist = np.linalg.norm(fk)
        if dist > self.workspace_radius:
            reason += "Near workspace boundary; "

        is_safe = len(reason) == 0
        return is_safe, safe, reason if reason else "OK"


# ─────────────────────────────────────────────────────────────────────────────
# Motion Mapping Node
# ─────────────────────────────────────────────────────────────────────────────

class MotionMappingNode(Node):
    """
    ROS2 node that maps human pose to robot motion commands.

    Subscribes to HumanPose and HandGesture, applies IK, smoothing,
    and safety checks, then publishes MotionCommand.
    """

    def __init__(self):
        super().__init__("motion_mapping")

        # Parameters
        self.declare_parameter("teleoperation_arm", "right")
        self.declare_parameter("scale_factor", 0.5)
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("max_joint_velocity", 2.0)
        self.declare_parameter("prediction_steps", 1)
        self.declare_parameter("enable_safety", True)
        self.declare_parameter("workspace_radius", 0.35)
        self.declare_parameter("mapping_mode", "angle")
        self.declare_parameter("simulation_mode", True)

        self.arm = self.get_parameter("teleoperation_arm").value
        self.scale = self.get_parameter("scale_factor").value
        self.smooth_alpha = self.get_parameter("smoothing_alpha").value
        self.max_vel = self.get_parameter("max_joint_velocity").value
        self.pred_steps = self.get_parameter("prediction_steps").value
        self.safety_enabled = self.get_parameter("enable_safety").value
        self.workspace_r = self.get_parameter("workspace_radius").value
        self.mapping_mode = self.get_parameter("mapping_mode").value
        self.sim_mode = self.get_parameter("simulation_mode").value

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # Publishers
        self.pub_motion = self.create_publisher(MotionCommand, "/motion_command", reliable_qos)
        self.pub_status = self.create_publisher(String, "/motion_status", reliable_qos)

        # Subscribers
        self.sub_pose = self.create_subscription(
            HumanPose, "/human_pose", self._on_pose, sensor_qos
        )
        self.sub_gesture = self.create_subscription(
            HandGesture, "/hand_gesture", self._on_gesture, reliable_qos
        )
        self.sub_estop = self.create_subscription(
            Bool, "/emergency_stop", self._on_estop, reliable_qos
        )

        # Components
        self.mapper = HumanRobotMapper(scale_factor=self.scale)
        self.ik_solver = MyCobotIK()
        self.safety = SafetyChecker(workspace_radius=self.workspace_r)
        self.smoother = EMASmoother(6, alpha=self.smooth_alpha)
        self.vel_limiter = VelocityLimiter(6, max_vel=self.max_vel, dt=0.033)
        self.predictor = MotionPredictor(6, steps=self.pred_steps)

        # State
        self._current_angles = np.zeros(6)
        self._prev_angles = np.zeros(6)
        self._gesture_active = False
        self._last_pose_time = time.time()
        self._cycle_count = 0

        # Status timer
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f"[MotionMapping] Initialized. "
            f"Arm={self.arm}, Scale={self.scale}, "
            f"Mode={self.mapping_mode}, Safety={self.safety_enabled}"
        )

    # ── Subscriber Callbacks ──────────────────────────────────────────────────

    def _on_pose(self, msg: HumanPose):
        """Process incoming HumanPose and produce MotionCommand."""
        if not msg.person_detected:
            return

        # Determine which arm to track
        arm = self.arm
        if arm == "auto":
            if msg.right_shoulder_visibility > msg.left_shoulder_visibility:
                arm = "right"
            else:
                arm = "left"

        # Map human pose to robot joint angles
        if self.mapping_mode == "angle":
            target_angles = self.mapper.map_pose_to_angles(msg, arm)
        elif self.mapping_mode == "position":
            target_pos = self.mapper.map_pose_to_position(msg, arm)
            if target_pos is not None:
                target_angles = self.ik_solver.solve(target_pos)
            else:
                target_angles = None
        else:
            target_angles = self.mapper.map_pose_to_angles(msg, arm)

        if target_angles is None:
            return

        # Motion prediction (latency compensation)
        if self.pred_steps > 0:
            target_angles = self.predictor.predict(target_angles)

        # EMA smoothing
        smoothed = self.smoother.filter(target_angles)

        # Velocity limiting
        limited = self.vel_limiter.limit(smoothed)

        # Safety checks
        if self.safety_enabled:
            is_safe, safe_angles, reason = self.safety.check(
                limited, self._prev_angles, dt=0.033
            )
        else:
            is_safe = True
            safe_angles = limited
            reason = "safety disabled"

        # Build MotionCommand
        cmd = MotionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        cmd.target_joint_angles = safe_angles.tolist()

        # Compute FK for position
        fk = self.ik_solver.forward(safe_angles)
        cmd.target_position = fk.tolist()
        cmd.target_orientation = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion

        cmd.confidence = msg.overall_confidence
        cmd.is_safe = is_safe
        cmd.source = 0 if arm == "left" else 1

        # Velocity hints
        if self._prev_angles is not None:
            vel = (safe_angles - self._prev_angles) / 0.033
            cmd.velocity_hints = np.clip(vel, -self.max_vel, self.max_vel).tolist()
        else:
            cmd.velocity_hints = [0.0] * 6

        cmd.latency_ms = (time.time() - self._last_pose_time) * 1000.0

        self.pub_motion.publish(cmd)

        # Update state
        self._prev_angles = safe_angles.copy()
        self._current_angles = safe_angles.copy()
        self._last_pose_time = time.time()
        self._cycle_count += 1

    def _on_gesture(self, msg: HandGesture):
        """Handle gesture commands."""
        if msg.command == 3:  # ESTOP
            self.safety.set_estop(True)
            self.get_logger().error("[MotionMapping] E-STOP from gesture")
        elif msg.command == 5:  # RESET
            self.safety.set_estop(False)
            self.smoother.reset()
            self.vel_limiter.reset()
            self.predictor.reset()
            self.get_logger().info("[MotionMapping] Reset from gesture")

        self._gesture_active = msg.gesture_id != 0

    def _on_estop(self, msg: Bool):
        self.safety.set_estop(msg.data)

    # ── Status Publisher ─────────────────────────────────────────────────────

    def _publish_status(self):
        status = {
            "node": "motion_mapping",
            "arm": self.arm,
            "mode": self.mapping_mode,
            "scale": self.scale,
            "safety_enabled": self.safety_enabled,
            "estop": self.safety._estop_active,
            "cycle_count": self._cycle_count,
            "current_angles_rad": self._current_angles.tolist(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MotionMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
