#!/usr/bin/env python3
"""
Standalone collector tracking demo using a single ArUco marker.

This script is intentionally independent from ROS 2 so the camera-based
collector localization can be validated first with plain Python + OpenCV.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def load_config(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


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


def build_camera_matrix(frame_width: int, frame_height: int, cfg: Dict) -> np.ndarray:
    fx = float(cfg.get("camera_fx", 0.0))
    fy = float(cfg.get("camera_fy", 0.0))
    cx = float(cfg.get("camera_cx", 0.0))
    cy = float(cfg.get("camera_cy", 0.0))
    hfov_deg = float(cfg.get("camera_hfov_deg", 69.0))

    if fx <= 0.0 or fy <= 0.0:
        hfov_rad = math.radians(max(hfov_deg, 1.0))
        fx = frame_width / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx

    if cx <= 0.0:
        cx = frame_width / 2.0
    if cy <= 0.0:
        cy = frame_height / 2.0

    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)


def build_transform_from_pose(position_key: str, rotation_key: str, cfg: Dict) -> np.ndarray:
    translation = np.array(cfg[position_key], dtype=float)
    rotation = np.array(cfg[rotation_key], dtype=float)
    return make_transform(rpy_to_matrix(rotation[0], rotation[1], rotation[2]), translation)


def marker_area(corners: np.ndarray) -> float:
    return float(cv2.contourArea(corners.reshape(-1, 2).astype(np.float32)))


def pretty_xyz_mm(transform: np.ndarray) -> Tuple[float, float, float]:
    position_mm = transform[:3, 3] * 1000.0
    return float(position_mm[0]), float(position_mm[1]), float(position_mm[2])


def main():
    parser = argparse.ArgumentParser(description="Standalone collector camera demo")
    parser.add_argument(
        "--config",
        default="tools/collector_camera_demo_config.example.json",
        help="Path to JSON config file",
    )
    parser.add_argument("--camera-index", type=int, default=None, help="Override camera index")
    parser.add_argument("--marker-id", type=int, default=None, help="Override marker id")
    parser.add_argument("--marker-length", type=float, default=None, help="Override marker length in meters")
    parser.add_argument("--show-tag-frame", action="store_true", help="Display tag pose in addition to collector pose")
    parser.add_argument("--headless", action="store_true", help="Do not open the OpenCV preview window")
    parser.add_argument("--save-last-pose", default="", help="Optional JSON output path for the last detected collector pose")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    cfg = load_config(config_path)
    if args.camera_index is not None:
        cfg["camera_index"] = args.camera_index
    if args.marker_id is not None:
        cfg["marker_id"] = args.marker_id
    if args.marker_length is not None:
        cfg["marker_length_m"] = args.marker_length

    dictionary, parameters, detector = build_detector(cfg.get("marker_dictionary", "DICT_4X4_50"))
    camera_distortion = np.array(cfg.get("camera_distortion", [0.0, 0.0, 0.0, 0.0, 0.0]), dtype=np.float32)
    robot_from_camera = build_transform_from_pose("camera_in_robot_position_m", "camera_in_robot_rpy_rad", cfg)
    tag_to_collector = build_transform_from_pose("collector_from_tag_position_m", "collector_from_tag_rpy_rad", cfg)

    marker_id = int(cfg["marker_id"])
    marker_length_m = float(cfg["marker_length_m"])
    min_marker_area_px = float(cfg.get("min_marker_area_px", 400.0))

    cap = cv2.VideoCapture(int(cfg["camera_index"]))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg['camera_index']}")

    frame_width = int(cfg.get("frame_width", 1280))
    frame_height = int(cfg.get("frame_height", 720))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    last_pose = None
    last_report_time = 0.0

    print("Collector camera demo started.")
    print("Keys: q=quit, s=save last pose JSON (if --save-last-pose was provided)")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                time.sleep(0.1)
                continue

            camera_matrix = build_camera_matrix(frame.shape[1], frame.shape[0], cfg)
            corners, ids, _ = detect_markers(frame, dictionary, parameters, detector)
            debug_frame = frame.copy()

            if ids is not None and len(ids) > 0:
                ids = ids.flatten()
                selected = None
                for index, detected_id in enumerate(ids):
                    if int(detected_id) == marker_id and marker_area(corners[index]) >= min_marker_area_px:
                        selected = index
                        break

                if selected is not None:
                    selected_corners = [corners[selected].astype(np.float32)]
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        selected_corners,
                        marker_length_m,
                        camera_matrix,
                        camera_distortion,
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]
                    rotation_camera_tag, _ = cv2.Rodrigues(rvec)
                    camera_from_tag = make_transform(rotation_camera_tag, tvec)
                    robot_from_tag = robot_from_camera @ camera_from_tag
                    robot_from_collector = robot_from_tag @ tag_to_collector
                    last_pose = {
                        "tag_xyz_mm": pretty_xyz_mm(robot_from_tag),
                        "collector_xyz_mm": pretty_xyz_mm(robot_from_collector),
                    }

                    now = time.time()
                    if now - last_report_time > 0.5:
                        tag_xyz = last_pose["tag_xyz_mm"]
                        collector_xyz = last_pose["collector_xyz_mm"]
                        print(
                            f"tag xyz mm = ({tag_xyz[0]:7.1f}, {tag_xyz[1]:7.1f}, {tag_xyz[2]:7.1f}) | "
                            f"collector xyz mm = ({collector_xyz[0]:7.1f}, {collector_xyz[1]:7.1f}, {collector_xyz[2]:7.1f})"
                        )
                        last_report_time = now

                    cv2.aruco.drawDetectedMarkers(debug_frame, selected_corners, np.array([[marker_id]], dtype=np.int32))
                    cv2.drawFrameAxes(
                        debug_frame,
                        camera_matrix,
                        camera_distortion,
                        rvec,
                        tvec,
                        marker_length_m * 0.5,
                    )

                    collector_xyz = last_pose["collector_xyz_mm"]
                    cv2.putText(
                        debug_frame,
                        f"collector xyz = {collector_xyz[0]:.0f}, {collector_xyz[1]:.0f}, {collector_xyz[2]:.0f} mm",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    if args.show_tag_frame:
                        tag_xyz = last_pose["tag_xyz_mm"]
                        cv2.putText(
                            debug_frame,
                            f"tag xyz = {tag_xyz[0]:.0f}, {tag_xyz[1]:.0f}, {tag_xyz[2]:.0f} mm",
                            (20, 75),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 220, 0),
                            2,
                            cv2.LINE_AA,
                        )
                else:
                    cv2.putText(
                        debug_frame,
                        f"Marker {marker_id} not usable",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 165, 255),
                        2,
                        cv2.LINE_AA,
                    )
            else:
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

            if not args.headless:
                cv2.imshow("collector_camera_demo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("s") and args.save_last_pose and last_pose is not None:
                output_path = Path(args.save_last_pose)
                output_path.write_text(json.dumps(last_pose, indent=2), encoding="utf-8")
                print(f"Saved pose to {output_path}")
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
