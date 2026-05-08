#!/usr/bin/env python3
"""
Human Tracking Node — MediaPipe Pose & Hand Tracking
=====================================================
Real-time human upper-body tracking for robot teleoperation.

Uses MediaPipe Pose and Hands solutions to:
  - Detect shoulder, elbow, wrist 3D positions
  - Estimate arm orientation vectors
  - Compute joint angles (shoulder flexion, elbow flexion)
  - Track 21 hand landmarks per hand
  - Recognize static gestures
  - Map gestures to robot commands

Topics Published:
  /human_pose       → HumanPose
  /hand_gesture     → HandGesture
  /tracking/image   → sensor_msgs/Image (annotated debug feed)

Parameters:
  camera_index          — OpenCV camera device index (default 0)
  tracking_fps          — Target processing rate (default 30)
  model_complexity      — MediaPipe pose model: 0=lite, 1=full, 2=heavy
  min_detection_conf    — Minimum detection confidence
  min_tracking_conf     — Minimum tracking confidence
  enable_hands          — Enable hand landmark tracking
  enable_pose           — Enable body pose tracking
  debug_visualization   — Publish annotated camera frames
  gesture_debounce_s    — Gesture debounce interval

Author: ElectroSpin Platform
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import time
import math
from enum import IntEnum
from typing import Optional, List, Tuple, Dict

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from electrospin_interfaces.msg import HumanPose, HandGesture

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    CVBRIDGE_AVAILABLE = True
except ImportError:
    CVBRIDGE_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Gesture Definitions
# ─────────────────────────────────────────────────────────────────────────────

class GestureID(IntEnum):
    NONE       = 0
    FIST       = 1
    OPEN       = 2
    POINT      = 3
    THUMBS_UP  = 4
    PEACE      = 5
    GRAB       = 6
    RELEASE    = 7


class RobotCommand(IntEnum):
    NONE       = 0
    START      = 1
    STOP       = 2
    ESTOP      = 3
    MODE_SWITCH = 4
    RESET      = 5


GESTURE_NAMES = {
    GestureID.NONE: "none",
    GestureID.FIST: "fist",
    GestureID.OPEN: "open",
    GestureID.POINT: "point",
    GestureID.THUMBS_UP: "thumbs_up",
    GestureID.PEACE: "peace",
    GestureID.GRAB: "grab",
    GestureID.RELEASE: "release",
}

# Gesture → Robot command mapping
GESTURE_COMMAND_MAP = {
    GestureID.FIST:      RobotCommand.ESTOP,
    GestureID.THUMBS_UP: RobotCommand.START,
    GestureID.OPEN:      RobotCommand.STOP,
    GestureID.PEACE:     RobotCommand.MODE_SWITCH,
    GestureID.POINT:     RobotCommand.NONE,
    GestureID.GRAB:      RobotCommand.NONE,
    GestureID.RELEASE:   RobotCommand.NONE,
    GestureID.NONE:      RobotCommand.NONE,
}


# ─────────────────────────────────────────────────────────────────────────────
# MediaPipe Landmark Indices
# ─────────────────────────────────────────────────────────────────────────────

# Pose landmarks (upper body subset)
POSE_L_SHOULDER = 11
POSE_R_SHOULDER = 12
POSE_L_ELBOW   = 13
POSE_R_ELBOW   = 14
POSE_L_WRIST   = 15
POSE_R_WRIST   = 16

# Hand landmarks
HAND_WRIST        = 0
HAND_THUMB_CMC    = 1
HAND_THUMB_MCP    = 2
HAND_THUMB_IP     = 3
HAND_THUMB_TIP    = 4
HAND_INDEX_MCP    = 5
HAND_INDEX_PIP    = 6
HAND_INDEX_DIP    = 7
HAND_INDEX_TIP    = 8
HAND_MIDDLE_MCP   = 9
HAND_MIDDLE_PIP   = 10
HAND_MIDDLE_DIP   = 11
HAND_MIDDLE_TIP   = 12
HAND_RING_MCP     = 13
HAND_RING_PIP     = 14
HAND_RING_DIP     = 15
HAND_RING_TIP     = 16
HAND_PINKY_MCP    = 17
HAND_PINKY_PIP    = 18
HAND_PINKY_DIP    = 19
HAND_PINKY_TIP    = 20

# Finger tip indices for gesture detection
FINGER_TIPS = [HAND_THUMB_TIP, HAND_INDEX_TIP, HAND_MIDDLE_TIP,
               HAND_RING_TIP, HAND_PINKY_TIP]
FINGER_PIPS = [HAND_THUMB_IP, HAND_INDEX_PIP, HAND_MIDDLE_PIP,
               HAND_RING_PIP, HAND_PINKY_PIP]


# ─────────────────────────────────────────────────────────────────────────────
# Gesture Classifier
# ─────────────────────────────────────────────────────────────────────────────

class GestureClassifier:
    """Classify hand gestures from MediaPipe hand landmarks."""

    @staticmethod
    def _finger_extended(landmarks, tip_idx, pip_idx, is_thumb=False) -> bool:
        """Check if a finger is extended."""
        if is_thumb:
            # Thumb: compare x-distance from wrist
            dx_tip = abs(landmarks[tip_idx].x - landmarks[HAND_WRIST].x)
            dx_ip = abs(landmarks[pip_idx].x - landmarks[HAND_WRIST].x)
            return dx_tip > dx_ip * 1.2
        else:
            # Other fingers: tip above PIP (y-axis points down in image)
            return landmarks[tip_idx].y < landmarks[pip_idx].y

    @staticmethod
    def classify(landmarks) -> Tuple[GestureID, float, List[int]]:
        """
        Classify gesture from 21 hand landmarks.
        Returns (gesture_id, confidence, finger_states).
        """
        if landmarks is None or len(landmarks) < 21:
            return GestureID.NONE, 0.0, [0, 0, 0, 0, 0]

        # Compute finger states
        finger_states = []
        finger_states.append(
            0 if GestureClassifier._finger_extended(
                landmarks, HAND_THUMB_TIP, HAND_THUMB_IP, is_thumb=True
            ) else 1
        )
        for tip, pip in zip(FINGER_TIPS[1:], FINGER_PIPS[1:]):
            finger_states.append(
                0 if GestureClassifier._finger_extended(landmarks, tip, pip) else 1
            )

        extended_count = sum(1 for s in finger_states if s == 0)
        flexed_count = 5 - extended_count

        # Pinch distance
        thumb_tip = landmarks[HAND_THUMB_TIP]
        index_tip = landmarks[HAND_INDEX_TIP]
        pinch_dist = math.sqrt(
            (thumb_tip.x - index_tip.x) ** 2 +
            (thumb_tip.y - index_tip.y) ** 2
        )

        # Classification rules
        confidence = 0.7

        if flexed_count >= 4 and finger_states[0] == 1:
            return GestureID.FIST, 0.9, finger_states

        if extended_count >= 4 and finger_states[0] == 0:
            return GestureID.OPEN, 0.85, finger_states

        if (finger_states[1] == 0 and finger_states[2] == 0 and
                flexed_count >= 2):
            return GestureID.PEACE, 0.85, finger_states

        if (finger_states[0] == 0 and finger_states[1] == 0 and
                flexed_count >= 3):
            return GestureID.THUMBS_UP, 0.8, finger_states

        if (finger_states[1] == 0 and flexed_count >= 3 and
                finger_states[0] == 1):
            return GestureID.POINT, 0.8, finger_states

        if pinch_dist < 0.05 and finger_states[1] == 1:
            return GestureID.GRAB, 0.7, finger_states

        if pinch_dist > 0.08 and finger_states[1] == 0:
            return GestureID.RELEASE, 0.65, finger_states

        return GestureID.NONE, confidence * 0.5, finger_states


# ─────────────────────────────────────────────────────────────────────────────
# Human Tracking Node
# ─────────────────────────────────────────────────────────────────────────────

class HumanTrackingNode(Node):
    """
    ROS2 node for real-time human pose and hand tracking using MediaPipe.

    Captures webcam frames, runs MediaPipe Pose + Hands, publishes
    HumanPose and HandGesture messages, and optionally streams annotated
    debug images.
    """

    def __init__(self):
        super().__init__("human_tracking")

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("tracking_fps", 30)
        self.declare_parameter("model_complexity", 1)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("enable_hands", True)
        self.declare_parameter("enable_pose", True)
        self.declare_parameter("debug_visualization", True)
        self.declare_parameter("gesture_debounce_s", 0.5)
        self.declare_parameter("simulation_mode", True)

        self.camera_idx = self.get_parameter("camera_index").value
        self.target_fps = self.get_parameter("tracking_fps").value
        self.model_complexity = self.get_parameter("model_complexity").value
        self.min_det_conf = self.get_parameter("min_detection_confidence").value
        self.min_trk_conf = self.get_parameter("min_tracking_confidence").value
        self.enable_hands = self.get_parameter("enable_hands").value
        self.enable_pose = self.get_parameter("enable_pose").value
        self.debug_viz = self.get_parameter("debug_visualization").value
        self.gesture_debounce = self.get_parameter("gesture_debounce_s").value
        self.sim_mode = self.get_parameter("simulation_mode").value

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=2
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5
        )

        # Publishers
        self.pub_pose = self.create_publisher(HumanPose, "/human_pose", reliable_qos)
        self.pub_gesture = self.create_publisher(HandGesture, "/hand_gesture", reliable_qos)
        self.pub_debug = self.create_publisher(Image, "/tracking/image", sensor_qos)

        # CV Bridge
        self._bridge = CvBridge() if CVBRIDGE_AVAILABLE else None

        # MediaPipe setup
        self._mp_pose = None
        self._mp_hands = None
        self._mp_draw = None
        self._pose_processor = None
        self._hands_processor = None

        if MEDIAPIPE_AVAILABLE:
            self._mp_pose = mp.solutions.pose
            self._mp_hands = mp.solutions.hands
            self._mp_draw = mp.solutions.drawing_utils
            self._mp_pose_style = mp.solutions.drawing_styles

            if self.enable_pose:
                self._pose_processor = self._mp_pose.Pose(
                    static_image_mode=False,
                    model_complexity=self.model_complexity,
                    smooth_landmarks=True,
                    enable_segmentation=False,
                    min_detection_confidence=self.min_det_conf,
                    min_tracking_confidence=self.min_trk_conf,
                )

            if self.enable_hands:
                self._hands_processor = self._mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=2,
                    model_complexity=1,
                    min_detection_confidence=self.min_det_conf,
                    min_tracking_confidence=self.min_trk_conf,
                )

        # Gesture classifier
        self._gesture_classifier = GestureClassifier()

        # Gesture debounce state
        self._last_gesture = GestureID.NONE
        self._last_gesture_time = 0.0

        # Camera
        self._cap = None
        if CV_AVAILABLE and not self.sim_mode:
            self._cap = cv2.VideoCapture(self.camera_idx)
            if not self._cap.isOpened():
                self.get_logger().warn(
                    f"Cannot open camera {self.camera_idx}, falling back to simulation"
                )
                self._cap = None
                self.sim_mode = True

        # Frame counter for simulation mode
        self._sim_frame = 0

        # Processing timer
        interval = 1.0 / max(1, self.target_fps)
        self._timer = self.create_timer(interval, self._tracking_cycle)

        self.get_logger().info(
            f"[HumanTracking] Initialized. "
            f"Pose={self.enable_pose}, Hands={self.enable_hands}, "
            f"Sim={self.sim_mode}, FPS={self.target_fps}"
        )

    def __del__(self):
        if self._cap is not None:
            self._cap.release()
        if self._pose_processor is not None:
            self._pose_processor.close()
        if self._hands_processor is not None:
            self._hands_processor.close()

    # ── Frame Acquisition ────────────────────────────────────────────────────

    def _get_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from camera or generate simulation frame."""
        if self.sim_mode:
            return self._generate_sim_frame()

        if self._cap is not None and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                return frame
        return None

    def _generate_sim_frame(self) -> np.ndarray:
        """Generate a synthetic frame for simulation testing."""
        self._sim_frame += 1
        h, w = 480, 640
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:] = (30, 30, 40)

        # Simulated arm motion (sinusoidal)
        t = self._sim_frame / 30.0
        cx = int(w * 0.5 + 80 * math.sin(t * 0.8))
        cy = int(h * 0.4 + 40 * math.cos(t * 1.2))

        # Draw simulated skeleton
        shoulder_l = (cx - 60, cy - 20)
        shoulder_r = (cx + 60, cy - 20)
        elbow_l = (cx - 90, cy + 50 + int(30 * math.sin(t * 2.0)))
        elbow_r = (cx + 90, cy + 50 + int(30 * math.cos(t * 2.0)))
        wrist_l = (cx - 70, cy + 120 + int(20 * math.sin(t * 2.5)))
        wrist_r = (cx + 70, cy + 120 + int(20 * math.cos(t * 2.5)))

        cv2.line(frame, shoulder_l, shoulder_r, (100, 200, 100), 2)
        cv2.line(frame, shoulder_l, elbow_l, (100, 200, 100), 2)
        cv2.line(frame, shoulder_r, elbow_r, (100, 200, 100), 2)
        cv2.line(frame, elbow_l, wrist_l, (100, 200, 100), 2)
        cv2.line(frame, elbow_r, wrist_r, (100, 200, 100), 2)

        for pt in [shoulder_l, shoulder_r, elbow_l, elbow_r, wrist_l, wrist_r]:
            cv2.circle(frame, pt, 5, (58, 166, 255), -1)

        # Label
        cv2.putText(frame, "SIMULATION MODE", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (58, 166, 255), 2)

        return frame

    # ── Main Tracking Cycle ──────────────────────────────────────────────────

    def _tracking_cycle(self):
        """Process one frame: capture, detect, publish."""
        frame = self._get_frame()
        if frame is None:
            return

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if CV_AVAILABLE else frame
        h, w = frame.shape[:2] if CV_AVAILABLE else (480, 640)

        # Process pose
        pose_landmarks = None
        if self._pose_processor is not None:
            results = self._pose_processor.process(image_rgb)
            if results.pose_landmarks:
                pose_landmarks = results.pose_landmarks.landmark

        # Process hands
        hand_results = None
        if self._hands_processor is not None:
            hand_results = self._hands_processor.process(image_rgb)

        # Build and publish HumanPose
        pose_msg = self._build_pose_msg(pose_landmarks, hand_results, w, h)
        self.pub_pose.publish(pose_msg)

        # Build and publish HandGesture
        gesture_msg = self._build_gesture_msg(hand_results)
        self.pub_gesture.publish(gesture_msg)

        # Debug visualization
        if self.debug_viz and CV_AVAILABLE and self._bridge is not None:
            debug_frame = self._annotate_frame(frame, pose_landmarks, hand_results)
            try:
                img_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera"
                self.pub_debug.publish(img_msg)
            except Exception:
                pass

    # ── Pose Message Builder ────────────────────────────────────────────────

    def _build_pose_msg(self, pose_landmarks, hand_results, img_w, img_h) -> HumanPose:
        msg = HumanPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        # Defaults
        msg.person_detected = False
        msg.num_persons = 0
        msg.overall_confidence = 0.0
        msg.left_hand_confidence = 0.0
        msg.right_hand_confidence = 0.0

        # Zero-init arrays
        msg.left_hand_landmarks = [0.0] * 63
        msg.right_hand_landmarks = [0.0] * 63

        if pose_landmarks is not None:
            msg.person_detected = True
            msg.num_persons = 1

            # Extract upper-body landmarks
            ls = pose_landmarks[POSE_L_SHOULDER]
            rs = pose_landmarks[POSE_R_SHOULDER]
            le = pose_landmarks[POSE_L_ELBOW]
            re = pose_landmarks[POSE_R_ELBOW]
            lw = pose_landmarks[POSE_L_WRIST]
            rw = pose_landmarks[POSE_R_WRIST]

            # Convert normalized coords to metric (approximate)
            # MediaPipe gives 0–1 normalized; we approximate depth from z
            def to_meters(lm):
                return [lm.x, lm.y, lm.z]

            msg.left_shoulder_position = to_meters(ls)
            msg.right_shoulder_position = to_meters(rs)
            msg.left_elbow_position = to_meters(le)
            msg.right_elbow_position = to_meters(re)
            msg.left_wrist_position = to_meters(lw)
            msg.right_wrist_position = to_meters(rw)

            msg.left_shoulder_visibility = ls.visibility
            msg.right_shoulder_visibility = rs.visibility
            msg.left_elbow_visibility = le.visibility
            msg.right_elbow_visibility = re.visibility

            # Compute arm direction vectors
            def unit_vec(a, b):
                dx = b[0] - a[0]
                dy = b[1] - a[1]
                dz = b[2] - a[2]
                mag = math.sqrt(dx*dx + dy*dy + dz*dz) + 1e-8
                return [dx/mag, dy/mag, dz/mag]

            msg.left_upper_arm_dir = unit_vec(
                msg.left_shoulder_position, msg.left_elbow_position
            )
            msg.left_forearm_dir = unit_vec(
                msg.left_elbow_position, msg.left_wrist_position
            )
            msg.right_upper_arm_dir = unit_vec(
                msg.right_shoulder_position, msg.right_elbow_position
            )
            msg.right_forearm_dir = unit_vec(
                msg.right_elbow_position, msg.right_wrist_position
            )

            # Compute joint angles
            msg.left_shoulder_angle = self._angle_between(
                [0, -1, 0], msg.left_upper_arm_dir
            )
            msg.left_elbow_angle = self._angle_between(
                msg.left_upper_arm_dir, msg.left_forearm_dir
            )
            msg.right_shoulder_angle = self._angle_between(
                [0, -1, 0], msg.right_upper_arm_dir
            )
            msg.right_elbow_angle = self._angle_between(
                msg.right_upper_arm_dir, msg.right_forearm_dir
            )

            # Overall confidence
            vis = [ls.visibility, rs.visibility, le.visibility, re.visibility,
                   lw.visibility, rw.visibility]
            msg.overall_confidence = sum(vis) / len(vis)

        # Hand landmarks
        if hand_results is not None and hand_results.multi_hand_landmarks:
            for idx, hand_lm in enumerate(hand_results.multi_hand_landmarks):
                handedness = hand_results.multi_handedness[idx]
                label = handedness.classification[0].label
                conf = handedness.classification[0].score

                flat = []
                for lm in hand_lm.landmark:
                    flat.extend([lm.x, lm.y, lm.z])

                if label == "Left":
                    msg.left_hand_landmarks = flat
                    msg.left_hand_confidence = conf
                else:
                    msg.right_hand_landmarks = flat
                    msg.right_hand_confidence = conf

        return msg

    # ── Gesture Message Builder ──────────────────────────────────────────────

    def _build_gesture_msg(self, hand_results) -> HandGesture:
        msg = HandGesture()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.gesture_id = GestureID.NONE
        msg.gesture_name = "none"
        msg.confidence = 0.0
        msg.left_hand = False
        msg.right_hand = False
        msg.command = RobotCommand.NONE
        msg.left_finger_states = [0] * 5
        msg.right_finger_states = [0] * 5
        msg.left_pinch_distance = 1.0
        msg.right_pinch_distance = 1.0

        if hand_results is None or not hand_results.multi_hand_landmarks:
            return msg

        best_gesture = GestureID.NONE
        best_conf = 0.0
        best_cmd = RobotCommand.NONE

        for idx, hand_lm in enumerate(hand_results.multi_hand_landmarks):
            handedness = hand_results.multi_handedness[idx]
            label = handedness.classification[0].label

            gesture_id, conf, finger_states = self._gesture_classifier.classify(
                hand_lm.landmark
            )

            # Pinch distance
            thumb = hand_lm.landmark[HAND_THUMB_TIP]
            index = hand_lm.landmark[HAND_INDEX_TIP]
            pinch = math.sqrt(
                (thumb.x - index.x)**2 + (thumb.y - index.y)**2
            )

            if label == "Left":
                msg.left_hand = True
                msg.left_finger_states = finger_states
                msg.left_pinch_distance = pinch
            else:
                msg.right_hand = True
                msg.right_finger_states = finger_states
                msg.right_pinch_distance = pinch

            # Keep highest-confidence gesture
            if conf > best_conf:
                best_gesture = gesture_id
                best_conf = conf
                best_cmd = GESTURE_COMMAND_MAP.get(gesture_id, RobotCommand.NONE)

        # Apply debounce
        now = time.time()
        if best_gesture != self._last_gesture:
            if now - self._last_gesture_time > self.gesture_debounce:
                self._last_gesture = best_gesture
                self._last_gesture_time = now
        else:
            best_gesture = self._last_gesture

        msg.gesture_id = int(best_gesture)
        msg.gesture_name = GESTURE_NAMES.get(best_gesture, "none")
        msg.confidence = best_conf
        msg.command = int(best_cmd)

        return msg

    # ── Utilities ────────────────────────────────────────────────────────────

    @staticmethod
    def _angle_between(v1, v2) -> float:
        """Angle in radians between two 3D vectors."""
        dot = sum(a * b for a, b in zip(v1, v2))
        mag1 = math.sqrt(sum(x*x for x in v1)) + 1e-8
        mag2 = math.sqrt(sum(x*x for x in v2)) + 1e-8
        cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
        return math.acos(cos_angle)

    def _annotate_frame(self, frame, pose_landmarks, hand_results) -> np.ndarray:
        """Draw tracking overlays on the frame."""
        annotated = frame.copy()

        if pose_landmarks is not None and self._mp_draw is not None:
            # Draw pose connections (upper body only)
            upper_body_connections = [
                (POSE_L_SHOULDER, POSE_R_SHOULDER),
                (POSE_L_SHOULDER, POSE_L_ELBOW),
                (POSE_L_ELBOW, POSE_L_WRIST),
                (POSE_R_SHOULDER, POSE_R_ELBOW),
                (POSE_R_ELBOW, POSE_R_WRIST),
            ]
            for start, end in upper_body_connections:
                p1 = pose_landmarks[start]
                p2 = pose_landmarks[end]
                h, w = frame.shape[:2]
                pt1 = (int(p1.x * w), int(p1.y * h))
                pt2 = (int(p2.x * w), int(p2.y * h))
                cv2.line(annotated, pt1, pt2, (58, 166, 255), 2)
                cv2.circle(annotated, pt1, 4, (63, 185, 80), -1)
                cv2.circle(annotated, pt2, 4, (63, 185, 80), -1)

        if hand_results is not None and hand_results.multi_hand_landmarks:
            for hand_lm in hand_results.multi_hand_landmarks:
                if self._mp_draw is not None:
                    self._mp_draw.draw_landmarks(
                        annotated, hand_lm, self._mp_hands.HAND_CONNECTIONS,
                        self._mp_pose_style.get_default_hand_connections_style(),
                        self._mp_pose_style.get_default_hand_landmarks_style(),
                    )

        # Status overlay
        status = "TRACKING" if pose_landmarks else "NO PERSON"
        color = (63, 185, 80) if pose_landmarks else (248, 81, 73)
        cv2.putText(annotated, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return annotated


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackingNode()

    if not MEDIAPIPE_AVAILABLE:
        node.get_logger().warn(
            "[HumanTracking] mediapipe not installed. "
            "Install with: pip install mediapipe"
        )

    if not CV_AVAILABLE:
        node.get_logger().error(
            "[HumanTracking] opencv not available. Cannot capture frames."
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
