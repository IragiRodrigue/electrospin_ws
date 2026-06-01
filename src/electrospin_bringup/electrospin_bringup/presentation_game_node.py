#!/usr/bin/env python3
"""
Presentation game node for a simple Cham Cham Cham style robot demo.

This node listens to the tracked human pose and converts left/right hand
guidance into safe MotionCommand targets that make the robot "look" left or
right. It is intentionally conservative so it can be used during a live demo.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from electrospin_interfaces.msg import HumanPose, HandGesture, MotionCommand


class PresentationGameNode(Node):
    """Map human hand motion into a simple left/right robot presentation demo."""

    SOURCE_GESTURE = 2

    def __init__(self):
        super().__init__("presentation_game")

        self.declare_parameter("tracked_hand", "right")
        self.declare_parameter("invert_direction", False)
        self.declare_parameter("activation_gesture", "point_or_open")
        self.declare_parameter("deadband_m", 0.05)
        self.declare_parameter("max_lateral_offset_m", 0.07)
        self.declare_parameter("joint6_max_deg", 35.0)
        self.declare_parameter("base_joint_angles_deg", [0.0, 20.0, -20.0, 0.0, -90.0, 0.0])
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("publish_rate_hz", 12.0)
        self.declare_parameter("hold_timeout_s", 0.75)
        self.declare_parameter("direction_smoothing", 0.35)

        self.tracked_hand = str(self.get_parameter("tracked_hand").value).lower()
        self.invert_direction = self._as_bool(self.get_parameter("invert_direction").value)
        self.activation_gesture = str(self.get_parameter("activation_gesture").value).lower()
        self.deadband_m = float(self.get_parameter("deadband_m").value)
        self.max_lateral_offset_m = float(self.get_parameter("max_lateral_offset_m").value)
        self.joint6_max_deg = float(self.get_parameter("joint6_max_deg").value)
        self.base_joint_angles_deg = [
            float(v) for v in self.get_parameter("base_joint_angles_deg").value
        ]
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.hold_timeout_s = float(self.get_parameter("hold_timeout_s").value)
        self.direction_smoothing = float(self.get_parameter("direction_smoothing").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_motion = self.create_publisher(MotionCommand, "/motion_command", reliable_qos)
        self.pub_status = self.create_publisher(String, "/presentation_game_status", reliable_qos)
        self.sub_pose = self.create_subscription(
            HumanPose, "/human_pose", self._on_pose, sensor_qos
        )
        self.sub_gesture = self.create_subscription(
            HandGesture, "/hand_gesture", self._on_gesture, reliable_qos
        )

        self.latest_pose: Optional[HumanPose] = None
        self.latest_gesture: Optional[HandGesture] = None
        self.filtered_direction = 0.0
        self.current_side = "center"
        self.last_active_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._tick)

        self.get_logger().info(
            "[PresentationGame] Ready. "
            f"hand={self.tracked_hand}, invert={self.invert_direction}, "
            f"gesture={self.activation_gesture}"
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def _on_pose(self, msg: HumanPose):
        self.latest_pose = msg

    def _on_gesture(self, msg: HandGesture):
        self.latest_gesture = msg

    def _gesture_allows_tracking(self) -> bool:
        if self.activation_gesture in {"none", "always"}:
            return True

        if self.latest_gesture is None:
            return False

        gesture_name = self.latest_gesture.gesture_name.lower()
        if self.activation_gesture == "point":
            return gesture_name == "point"
        if self.activation_gesture == "open":
            return gesture_name == "open"
        if self.activation_gesture == "point_or_open":
            return gesture_name in {"point", "open"}
        return False

    def _get_hand_x(self, pose: HumanPose) -> Optional[float]:
        if not pose.person_detected or pose.overall_confidence < self.conf_threshold:
            return None

        if self.tracked_hand == "left":
            wrist = pose.left_wrist_position
            shoulder = pose.left_shoulder_position
            visibility = min(pose.left_wrist_visibility, pose.left_shoulder_visibility)
        else:
            wrist = pose.right_wrist_position
            shoulder = pose.right_shoulder_position
            visibility = min(pose.right_wrist_visibility, pose.right_shoulder_visibility)

        if visibility < self.conf_threshold:
            return None

        return float(wrist[0] - shoulder[0])

    def _tick(self):
        if self.latest_pose is None:
            self._publish_status(active=False, side="center", offset_m=0.0, reason="no_pose")
            return

        if not self._gesture_allows_tracking():
            self._publish_status(active=False, side="center", offset_m=0.0, reason="gesture_gate")
            return

        raw_x = self._get_hand_x(self.latest_pose)
        if raw_x is None:
            self._publish_status(active=False, side="center", offset_m=0.0, reason="low_confidence")
            return

        if self.invert_direction:
            raw_x = -raw_x

        bounded = max(-self.max_lateral_offset_m, min(self.max_lateral_offset_m, raw_x))
        if abs(bounded) < self.deadband_m:
            bounded = 0.0

        self.filtered_direction = (
            self.direction_smoothing * bounded
            + (1.0 - self.direction_smoothing) * self.filtered_direction
        )

        if abs(self.filtered_direction) < self.deadband_m * 0.5:
            self.current_side = "center"
        elif self.filtered_direction > 0.0:
            self.current_side = "right"
        else:
            self.current_side = "left"

        self.last_active_time = self.get_clock().now()
        self._publish_motion(self.filtered_direction)
        self._publish_status(
            active=True,
            side=self.current_side,
            offset_m=self.filtered_direction,
            reason="tracking",
        )

    def _publish_motion(self, x_offset_m: float):
        normalized = 0.0
        if self.max_lateral_offset_m > 1e-6:
            normalized = max(-1.0, min(1.0, x_offset_m / self.max_lateral_offset_m))

        target_joint_angles_deg = list(self.base_joint_angles_deg)
        target_joint_angles_deg[5] = self.base_joint_angles_deg[5] + normalized * self.joint6_max_deg

        cmd = MotionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.target_joint_angles = [math.radians(v) for v in target_joint_angles_deg]
        cmd.target_position = [0.0, 0.0, 0.0]
        cmd.target_orientation = [0.0, 0.0, 0.0, 1.0]
        cmd.confidence = 0.95
        cmd.is_safe = True
        cmd.source = self.SOURCE_GESTURE
        cmd.velocity_hints = [0.0] * 6
        cmd.latency_ms = 1000.0 / max(self.publish_rate_hz, 1.0)
        self.pub_motion.publish(cmd)

    def _publish_status(self, active: bool, side: str, offset_m: float, reason: str):
        age_s = (
            self.get_clock().now().nanoseconds - self.last_active_time.nanoseconds
        ) / 1e9
        holding = active or age_s < self.hold_timeout_s
        status = {
            "active": active,
            "holding": holding,
            "tracked_hand": self.tracked_hand,
            "side": side,
            "offset_m": round(float(offset_m), 4),
            "joint6_deg": round(
                self.base_joint_angles_deg[5]
                + (
                    max(-1.0, min(1.0, offset_m / self.max_lateral_offset_m))
                    * self.joint6_max_deg
                    if self.max_lateral_offset_m > 1e-6 else 0.0
                ),
                2,
            ),
            "reason": reason,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PresentationGameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
