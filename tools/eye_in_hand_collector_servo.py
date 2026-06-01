#!/usr/bin/env python3
"""
Eye-in-hand collector localization and target pose computation for MyCobot.

This script assumes:
- the camera is mounted on joint 6 / tool side
- a real ArUco marker is rigidly attached near the collector
- the marker size is known
- the transforms tool->camera, tag->collector, and tool->needle are approximated

It can run in:
- detect-only mode
- target-compute mode
- direct robot command mode via pymycobot send_coords()
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np

try:
    from pymycobot.mycobot import MyCobot
    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_json(path: Path, payload: Dict):
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def resolve_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is not available. Install opencv-contrib-python.")
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def build_detector(dictionary_name: str):
    dictionary = resolve_dictionary(dictionary_name)
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()
    detector = None
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters, detector


def detect_markers(frame: np.ndarray, dictionary, parameters, detector):
    if detector is not None:
        return detector.detectMarkers(frame)
    return cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)


def build_camera_matrix(width: int, height: int, cfg: Dict) -> np.ndarray:
    fx = float(cfg.get("camera_fx", 0.0))
    fy = float(cfg.get("camera_fy", 0.0))
    cx = float(cfg.get("camera_cx", 0.0))
    cy = float(cfg.get("camera_cy", 0.0))
    hfov_deg = float(cfg.get("camera_hfov_deg", 69.0))
    if fx <= 0.0 or fy <= 0.0:
        hfov_rad = math.radians(max(hfov_deg, 1.0))
        fx = width / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx
    if cx <= 0.0:
        cx = width / 2.0
    if cy <= 0.0:
        cy = height / 2.0
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)


def coords_deg_to_transform(coords: Tuple[float, float, float, float, float, float]) -> np.ndarray:
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = coords
    rotation = rpy_to_matrix(
        math.radians(rx_deg),
        math.radians(ry_deg),
        math.radians(rz_deg),
    )
    translation = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0], dtype=float)
    return make_transform(rotation, translation)


def transform_to_coords_deg(transform: np.ndarray) -> Tuple[float, float, float, float, float, float]:
    position = transform[:3, 3] * 1000.0
    roll, pitch, yaw = matrix_to_rpy(transform[:3, :3])
    return (
        float(position[0]),
        float(position[1]),
        float(position[2]),
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def build_transform_from_cfg(pos_key: str, rpy_key: str, cfg: Dict) -> np.ndarray:
    position = np.array(cfg[pos_key], dtype=float)
    rpy = np.array(cfg[rpy_key], dtype=float)
    return make_transform(rpy_to_matrix(rpy[0], rpy[1], rpy[2]), position)


def marker_area(corners: np.ndarray) -> float:
    return float(cv2.contourArea(corners.reshape(-1, 2).astype(np.float32)))


def main():
    parser = argparse.ArgumentParser(description="Eye-in-hand collector servo for MyCobot")
    parser.add_argument("--config", default="tools/eye_in_hand_collector_servo_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--control-robot", action="store_true")
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-last-json", default="")
    parser.add_argument("--apply-on-key", default="g", help="Key to send target coords to robot")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config file not found: {cfg_path}")
    cfg = load_json(cfg_path)

    if args.camera_index is not None:
        cfg["camera_index"] = args.camera_index
    if args.serial_port is not None:
        cfg["serial_port"] = args.serial_port
    if args.baud_rate is not None:
        cfg["baud_rate"] = args.baud_rate

    dictionary, parameters, detector = build_detector(cfg.get("marker_dictionary", "DICT_4X4_50"))
    distortion = np.array(cfg.get("camera_distortion", [0, 0, 0, 0, 0]), dtype=np.float32)
    marker_id = int(cfg["marker_id"])
    marker_length_m = float(cfg["marker_length_m"])
    min_area = float(cfg.get("min_marker_area_px", 400.0))

    tool_from_camera = build_transform_from_cfg("tool_from_camera_position_m", "tool_from_camera_rpy_rad", cfg)
    tag_to_collector = build_transform_from_cfg("collector_from_tag_position_m", "collector_from_tag_rpy_rad", cfg)
    tool_to_needle = build_transform_from_cfg("needle_from_tool_position_m", "needle_from_tool_rpy_rad", cfg)
    collector_to_needle_target = build_transform_from_cfg(
        "needle_target_from_collector_position_m",
        "needle_target_from_collector_rpy_rad",
        cfg,
    )

    robot = None
    if args.control_robot:
        if not PYMCOBOT_AVAILABLE:
            raise RuntimeError("pymycobot is not installed. Install it or disable --control-robot.")
        robot = MyCobot(cfg.get("serial_port", "/dev/ttyTHS1"), int(cfg.get("baud_rate", 1000000)))
        time.sleep(1.0)

    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.get('camera_index', 0)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("frame_width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("frame_height", 720)))
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    last_payload = None
    print("Eye-in-hand collector servo started.")
    print(f"Need a REAL full ArUco marker, fully visible. Press '{args.apply_on_key}' to send target coords.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            camera_matrix = build_camera_matrix(frame.shape[1], frame.shape[0], cfg)
            corners, ids, _ = detect_markers(frame, dictionary, parameters, detector)

            status_line = "marker not detected"
            target_coords = None

            if ids is not None and len(ids) > 0:
                ids = ids.flatten()
                selected_index = None
                for i, detected_id in enumerate(ids):
                    if int(detected_id) == marker_id and marker_area(corners[i]) >= min_area:
                        selected_index = i
                        break

                if selected_index is not None:
                    selected_corners = [corners[selected_index].astype(np.float32)]
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        selected_corners,
                        marker_length_m,
                        camera_matrix,
                        distortion,
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]
                    rotation_camera_tag, _ = cv2.Rodrigues(rvec)
                    camera_to_tag = make_transform(rotation_camera_tag, tvec)

                    cv2.aruco.drawDetectedMarkers(debug_frame, selected_corners, np.array([[marker_id]], dtype=np.int32))
                    cv2.drawFrameAxes(
                        debug_frame,
                        camera_matrix,
                        distortion,
                        rvec,
                        tvec,
                        marker_length_m * 0.5,
                    )

                    if robot is not None:
                        current_coords = robot.get_coords()
                    else:
                        current_coords = cfg.get("simulated_robot_coords_mm_deg", [160.0, 0.0, 180.0, 0.0, -90.0, 0.0])
                    if current_coords is None or len(current_coords) != 6:
                        raise RuntimeError(f"Invalid robot coords: {current_coords}")

                    base_to_tool = coords_deg_to_transform(tuple(float(v) for v in current_coords))
                    base_to_tag = base_to_tool @ tool_from_camera @ camera_to_tag
                    base_to_collector = base_to_tag @ tag_to_collector
                    base_to_needle_target = base_to_collector @ collector_to_needle_target
                    base_to_target_tool = base_to_needle_target @ np.linalg.inv(tool_to_needle)
                    target_coords = transform_to_coords_deg(base_to_target_tool)

                    collector_center_mm = base_to_collector[:3, 3] * 1000.0
                    needle_target_mm = base_to_needle_target[:3, 3] * 1000.0
                    tag_camera_mm = camera_to_tag[:3, 3] * 1000.0

                    status_line = (
                        f"collector=({collector_center_mm[0]:.0f},{collector_center_mm[1]:.0f},{collector_center_mm[2]:.0f})mm "
                        f"target=({target_coords[0]:.0f},{target_coords[1]:.0f},{target_coords[2]:.0f})"
                    )

                    cv2.putText(
                        debug_frame,
                        f"tag camera xyz = {tag_camera_mm[0]:.0f}, {tag_camera_mm[1]:.0f}, {tag_camera_mm[2]:.0f} mm",
                        (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (255, 220, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        (
                            f"collector xyz = "
                            f"{collector_center_mm[0]:.0f}, {collector_center_mm[1]:.0f}, {collector_center_mm[2]:.0f} mm"
                        ),
                        (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        (
                            f"target needle xyz = "
                            f"{needle_target_mm[0]:.0f}, {needle_target_mm[1]:.0f}, {needle_target_mm[2]:.0f} mm"
                        ),
                        (12, 92),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    last_payload = {
                        "current_robot_coords_mm_deg": [round(float(v), 4) for v in current_coords],
                        "target_robot_coords_mm_deg": [round(float(v), 4) for v in target_coords],
                        "tag_in_camera_mm": [round(float(v), 4) for v in tag_camera_mm],
                        "collector_center_in_base_mm": [round(float(v), 4) for v in collector_center_mm],
                        "needle_target_in_base_mm": [round(float(v), 4) for v in needle_target_mm],
                    }
                else:
                    status_line = f"marker {marker_id} found but too small / invalid"

            cv2.putText(
                debug_frame,
                status_line,
                (12, frame.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if target_coords is not None else (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

            if not args.headless:
                cv2.imshow("eye_in_hand_collector_servo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord(args.apply_on_key) and robot is not None and target_coords is not None:
                robot.send_coords(list(target_coords), args.speed, 0)
                print(f"Sent target coords: {[round(v, 2) for v in target_coords]}")
            if key == ord("s") and args.save_last_json and last_payload is not None:
                save_json(Path(args.save_last_json), last_payload)
                print(f"Saved result to {args.save_last_json}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
