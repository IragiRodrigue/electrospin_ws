#!/usr/bin/env python3
"""
Collector tracker based on a single ArUco marker observed by the process camera.

The node subscribes to /camera/image_raw, estimates the tag pose, converts it to a
collector center pose using configurable offsets, and publishes the result for the
robot controller. This keeps the first camera-based integration simple and robust.
"""

import json
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


class CollectorTrackerNode(Node):
    def __init__(self):
        super().__init__("collector_tracker")

        self.declare_parameter("enabled", False)
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("collector_frame", "collector_frame")
        self.declare_parameter("collector_tag_frame", "collector_tag")
        self.declare_parameter("marker_dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_length_m", 0.03)
        self.declare_parameter("detection_timeout_s", 1.0)
        self.declare_parameter("min_marker_area_px", 400.0)
        self.declare_parameter("pose_smoothing_alpha", 0.35)
        self.declare_parameter("debug_visualization", True)

        self.declare_parameter("camera_hfov_deg", 69.0)
        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)
        self.declare_parameter("camera_distortion", [0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("camera_in_robot_x_m", 0.0)
        self.declare_parameter("camera_in_robot_y_m", 0.0)
        self.declare_parameter("camera_in_robot_z_m", 0.0)
        self.declare_parameter("camera_in_robot_roll_rad", 0.0)
        self.declare_parameter("camera_in_robot_pitch_rad", 0.0)
        self.declare_parameter("camera_in_robot_yaw_rad", 0.0)

        self.declare_parameter("collector_from_tag_x_m", 0.0)
        self.declare_parameter("collector_from_tag_y_m", 0.0)
        self.declare_parameter("collector_from_tag_z_m", 0.0)
        self.declare_parameter("collector_from_tag_roll_rad", 0.0)
        self.declare_parameter("collector_from_tag_pitch_rad", 0.0)
        self.declare_parameter("collector_from_tag_yaw_rad", 0.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.collector_frame = str(self.get_parameter("collector_frame").value)
        self.collector_tag_frame = str(self.get_parameter("collector_tag_frame").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_length_m = float(self.get_parameter("marker_length_m").value)
        self.detection_timeout_s = float(self.get_parameter("detection_timeout_s").value)
        self.min_marker_area_px = float(self.get_parameter("min_marker_area_px").value)
        self.pose_smoothing_alpha = float(self.get_parameter("pose_smoothing_alpha").value)
        self.debug_visualization = bool(self.get_parameter("debug_visualization").value)

        self.camera_hfov_deg = float(self.get_parameter("camera_hfov_deg").value)
        self.camera_fx = float(self.get_parameter("camera_fx").value)
        self.camera_fy = float(self.get_parameter("camera_fy").value)
        self.camera_cx = float(self.get_parameter("camera_cx").value)
        self.camera_cy = float(self.get_parameter("camera_cy").value)
        self.camera_distortion = np.array(
            [float(v) for v in self.get_parameter("camera_distortion").value],
            dtype=np.float32,
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_detection_time = None
        self.last_pose_m = None
        self.last_rotation = None
        self.smoothed_position = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_pose = self.create_publisher(PoseStamped, "/collector_pose", reliable_qos)
        self.pub_status = self.create_publisher(String, "/collector_tracking_status", reliable_qos)
        self.pub_debug = self.create_publisher(Image, "/collector_tracking_debug", sensor_qos)
        self.sub_image = self.create_subscription(Image, "/camera/image_raw", self._on_image, sensor_qos)
        self.status_timer = self.create_timer(0.25, self._publish_status)

        self.marker_dictionary_name = str(self.get_parameter("marker_dictionary").value)
        self.aruco_dictionary = self._resolve_dictionary(self.marker_dictionary_name)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_parameters = cv2.aruco.DetectorParameters()
        else:
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        camera_rotation = rpy_to_matrix(
            float(self.get_parameter("camera_in_robot_roll_rad").value),
            float(self.get_parameter("camera_in_robot_pitch_rad").value),
            float(self.get_parameter("camera_in_robot_yaw_rad").value),
        )
        camera_translation = np.array(
            [
                float(self.get_parameter("camera_in_robot_x_m").value),
                float(self.get_parameter("camera_in_robot_y_m").value),
                float(self.get_parameter("camera_in_robot_z_m").value),
            ],
            dtype=float,
        )
        self.robot_from_camera = make_transform(camera_rotation, camera_translation)

        collector_rotation = rpy_to_matrix(
            float(self.get_parameter("collector_from_tag_roll_rad").value),
            float(self.get_parameter("collector_from_tag_pitch_rad").value),
            float(self.get_parameter("collector_from_tag_yaw_rad").value),
        )
        collector_translation = np.array(
            [
                float(self.get_parameter("collector_from_tag_x_m").value),
                float(self.get_parameter("collector_from_tag_y_m").value),
                float(self.get_parameter("collector_from_tag_z_m").value),
            ],
            dtype=float,
        )
        self.tag_to_collector = make_transform(collector_rotation, collector_translation)

        self.get_logger().info(
            "[CollectorTracker] Initialized. "
            f"Enabled={self.enabled}, MarkerID={self.marker_id}, Frame={self.collector_frame}"
        )

    def _resolve_dictionary(self, name: str):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV ArUco module is not available in this environment.")
        dictionary_id = getattr(cv2.aruco, name, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown ArUco dictionary: {name}")
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _camera_matrix(self, image_width: int, image_height: int) -> np.ndarray:
        fx = self.camera_fx
        fy = self.camera_fy
        cx = self.camera_cx
        cy = self.camera_cy

        if fx <= 0.0 or fy <= 0.0:
            hfov_rad = math.radians(max(self.camera_hfov_deg, 1.0))
            fx = image_width / (2.0 * math.tan(hfov_rad / 2.0))
            fy = fx

        if cx <= 0.0:
            cx = image_width / 2.0
        if cy <= 0.0:
            cy = image_height / 2.0

        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)

    def _marker_area(self, corners: np.ndarray) -> float:
        pts = corners.reshape(-1, 2).astype(np.float32)
        return float(cv2.contourArea(pts))

    def _on_image(self, msg: Image):
        if not self.enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"[CollectorTracker] Failed to decode image: {exc}")
            return

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame,
                self.aruco_dictionary,
                parameters=self.aruco_parameters,
            )
        debug_frame = frame.copy() if self.debug_visualization else None

        if ids is None or len(ids) == 0:
            if debug_frame is not None:
                cv2.putText(
                    debug_frame,
                    "Collector tag not detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self._publish_debug_image(debug_frame, msg.header)
            return

        ids = ids.flatten()
        target_index = None
        for index, detected_id in enumerate(ids):
            if int(detected_id) == self.marker_id and self._marker_area(corners[index]) >= self.min_marker_area_px:
                target_index = index
                break

        if target_index is None:
            if debug_frame is not None:
                cv2.putText(
                    debug_frame,
                    f"Marker {self.marker_id} not usable",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                self._publish_debug_image(debug_frame, msg.header)
            return

        image_height, image_width = frame.shape[:2]
        camera_matrix = self._camera_matrix(image_width, image_height)
        marker_corners = [corners[target_index].astype(np.float32)]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners,
            self.marker_length_m,
            camera_matrix,
            self.camera_distortion,
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        rotation_camera_tag, _ = cv2.Rodrigues(rvec)
        camera_from_tag = make_transform(rotation_camera_tag, tvec)
        robot_from_tag = self.robot_from_camera @ camera_from_tag
        robot_from_collector = robot_from_tag @ self.tag_to_collector

        position_m = robot_from_collector[:3, 3].copy()
        rotation_robot_collector = robot_from_collector[:3, :3].copy()

        if self.smoothed_position is None:
            self.smoothed_position = position_m
        else:
            alpha = min(max(self.pose_smoothing_alpha, 0.0), 1.0)
            self.smoothed_position = (1.0 - alpha) * self.smoothed_position + alpha * position_m
            position_m = self.smoothed_position

        self.last_detection_time = self.get_clock().now()
        self.last_pose_m = position_m
        self.last_rotation = rotation_robot_collector

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.robot_base_frame
        pose_msg.pose.position.x = float(position_m[0])
        pose_msg.pose.position.y = float(position_m[1])
        pose_msg.pose.position.z = float(position_m[2])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_robot_collector)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pub_pose.publish(pose_msg)

        tag_qx, tag_qy, tag_qz, tag_qw = rotation_matrix_to_quaternion(robot_from_tag[:3, :3])
        tag_tf_msg = TransformStamped()
        tag_tf_msg.header = pose_msg.header
        tag_tf_msg.child_frame_id = self.collector_tag_frame
        tag_tf_msg.transform.translation.x = float(robot_from_tag[0, 3])
        tag_tf_msg.transform.translation.y = float(robot_from_tag[1, 3])
        tag_tf_msg.transform.translation.z = float(robot_from_tag[2, 3])
        tag_tf_msg.transform.rotation.x = float(tag_qx)
        tag_tf_msg.transform.rotation.y = float(tag_qy)
        tag_tf_msg.transform.rotation.z = float(tag_qz)
        tag_tf_msg.transform.rotation.w = float(tag_qw)
        self.tf_broadcaster.sendTransform(tag_tf_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = self.collector_frame
        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        if debug_frame is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, marker_corners, np.array([[self.marker_id]], dtype=np.int32))
            cv2.drawFrameAxes(
                debug_frame,
                camera_matrix,
                self.camera_distortion,
                rvec,
                tvec,
                self.marker_length_m * 0.5,
            )
            position_mm = position_m * 1000.0
            cv2.putText(
                debug_frame,
                (
                    f"collector xyz = "
                    f"{position_mm[0]:.0f}, {position_mm[1]:.0f}, {position_mm[2]:.0f} mm"
                ),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            self._publish_debug_image(debug_frame, msg.header)

    def _publish_debug_image(self, frame: np.ndarray, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = header
            self.pub_debug.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f"[CollectorTracker] Failed to publish debug image: {exc}")

    def _publish_status(self):
        now = self.get_clock().now()
        if self.last_detection_time is None:
            age_s = None
            tracking_ok = False
        else:
            age_s = (now.nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            tracking_ok = age_s <= self.detection_timeout_s

        payload = {
            "enabled": self.enabled,
            "tracking_ok": tracking_ok,
            "marker_id": self.marker_id,
            "age_s": None if age_s is None else round(age_s, 3),
            "pose_m": None if self.last_pose_m is None else [round(float(v), 4) for v in self.last_pose_m],
            "source_frame": self.camera_frame,
            "target_frame": self.robot_base_frame,
        }
        status_msg = String()
        status_msg.data = json.dumps(payload)
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollectorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
