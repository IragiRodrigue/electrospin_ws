#!/usr/bin/env python3
"""
Interactive calibration helper for the eye-in-hand collector setup.

This tool lets you tune the key geometric offsets live while watching the camera:
- tool_from_camera_position_m
- collector_from_tag_position_m
- needle_target_from_collector_position_m

It uses the same marker detection pipeline as eye_in_hand_collector_servo.py,
but focuses on calibration rather than movement.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict

import cv2
import numpy as np

from eye_in_hand_collector_servo import (
    build_camera_matrix,
    build_detector,
    coords_deg_to_transform,
    detect_markers,
    load_json,
    marker_area,
    save_json,
    transform_to_coords_deg,
    make_transform,
    rpy_to_matrix,
)

try:
    from pymycobot.mycobot import MyCobot
    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


def build_transform(position, rpy):
    return make_transform(rpy_to_matrix(rpy[0], rpy[1], rpy[2]), np.array(position, dtype=float))


def overlay_lines(frame, lines):
    y = 24
    for text, color in lines:
        cv2.putText(
            frame,
            text,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        y += 24


def normalize_key(key: int) -> int:
    if ord("A") <= key <= ord("Z"):
        return key + 32
    return key


def main():
    parser = argparse.ArgumentParser(description="Interactive eye-in-hand calibration helper")
    parser.add_argument("--config", default="tools/eye_in_hand_collector_servo_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--use-robot-coords", action="store_true", help="Read current coords from the real robot")
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--save-on-exit", action="store_true")
    parser.add_argument("--autosave", action="store_true", help="Save config automatically after each offset change")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    cfg: Dict = load_json(config_path)
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
    min_area = float(cfg.get("min_marker_area_px", 800.0))

    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.get('camera_index', 0)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("frame_width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("frame_height", 720)))
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    robot = None
    if args.use_robot_coords:
        if not PYMCOBOT_AVAILABLE:
            raise RuntimeError("pymycobot is not installed. Disable --use-robot-coords or install pymycobot.")
        robot = MyCobot(cfg.get("serial_port", "/dev/ttyTHS1"), int(cfg.get("baud_rate", 1000000)))
        time.sleep(1.0)

    edit_mode = "collector"
    step = 0.005
    last_payload = None
    status_message = "ready"

    print("Interactive eye-in-hand calibration started.")
    print("Modes: c=collector offset, n=needle target offset, t=tool-camera offset")
    print("Axes: a/d=x-,x+  w/s=y+,y-  r/f=z+,z-  [ and ] step-/step+")
    print("p=save now, Ctrl+S=save now, q=quit")
    print(f"Config file: {config_path.resolve()}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            camera_matrix = build_camera_matrix(frame.shape[1], frame.shape[0], cfg)
            corners, ids, _ = detect_markers(frame, dictionary, parameters, detector)

            lines = [
                (f"mode={edit_mode} step={step:.4f} m", (255, 220, 0)),
                (f"config={config_path.name}", (255, 255, 255)),
                (f"tool_from_camera={cfg['tool_from_camera_position_m']}", (200, 255, 200)),
                (f"collector_from_tag={cfg['collector_from_tag_position_m']}", (200, 255, 200)),
                (f"needle_target_from_collector={cfg['needle_target_from_collector_position_m']}", (200, 255, 200)),
                (f"status={status_message}", (200, 220, 255)),
            ]

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
                    cv2.drawFrameAxes(debug_frame, camera_matrix, distortion, rvec, tvec, marker_length_m * 0.5)

                    if robot is not None:
                        current_coords = robot.get_coords()
                    else:
                        current_coords = cfg.get("simulated_robot_coords_mm_deg", [160.0, 0.0, 180.0, 0.0, -90.0, 0.0])

                    base_to_tool = coords_deg_to_transform(tuple(float(v) for v in current_coords))
                    tool_from_camera = build_transform(cfg["tool_from_camera_position_m"], cfg["tool_from_camera_rpy_rad"])
                    tag_to_collector = build_transform(cfg["collector_from_tag_position_m"], cfg["collector_from_tag_rpy_rad"])
                    collector_to_needle_target = build_transform(
                        cfg["needle_target_from_collector_position_m"],
                        cfg["needle_target_from_collector_rpy_rad"],
                    )

                    base_to_tag = base_to_tool @ tool_from_camera @ camera_to_tag
                    base_to_collector = base_to_tag @ tag_to_collector
                    base_to_needle_target = base_to_collector @ collector_to_needle_target
                    target_coords = transform_to_coords_deg(base_to_needle_target)

                    collector_mm = base_to_collector[:3, 3] * 1000.0
                    target_mm = base_to_needle_target[:3, 3] * 1000.0
                    tag_mm = camera_to_tag[:3, 3] * 1000.0

                    lines.extend([
                        (f"tag camera mm = {[round(float(v), 1) for v in tag_mm]}", (0, 255, 255)),
                        (f"collector base mm = {[round(float(v), 1) for v in collector_mm]}", (0, 255, 0)),
                        (f"needle target mm = {[round(float(v), 1) for v in target_mm]}", (255, 255, 0)),
                        (f"target coords mm/deg = {[round(float(v), 1) for v in target_coords]}", (255, 180, 0)),
                    ])
                    last_payload = cfg.copy()
                else:
                    lines.append((f"marker {marker_id} visible but invalid", (0, 165, 255)))
            else:
                lines.append(("marker not detected", (0, 0, 255)))

            overlay_lines(debug_frame, lines)
            cv2.imshow("eye_in_hand_calibrator", debug_frame)
            raw_key = cv2.waitKey(1) & 0xFF
            key = normalize_key(raw_key)

            if key == ord("q"):
                break
            if key == ord("c"):
                edit_mode = "collector"
            elif key == ord("n"):
                edit_mode = "needle"
            elif key == ord("t"):
                edit_mode = "tool"
            elif key == ord("["):
                step = max(0.001, step * 0.5)
            elif key == ord("]"):
                step = min(0.05, step * 2.0)
            elif key in (ord("p"), 19):
                save_json(config_path, cfg)
                status_message = f"saved to {config_path.name}"
                print(f"Saved config to {config_path}")
            elif key in (ord("a"), ord("d"), ord("w"), ord("s"), ord("r"), ord("f")):
                if edit_mode == "collector":
                    vector = cfg["collector_from_tag_position_m"]
                elif edit_mode == "needle":
                    vector = cfg["needle_target_from_collector_position_m"]
                else:
                    vector = cfg["tool_from_camera_position_m"]

                if key == ord("a"):
                    vector[0] -= step
                elif key == ord("d"):
                    vector[0] += step
                elif key == ord("w"):
                    vector[1] += step
                elif key == ord("s"):
                    vector[1] -= step
                elif key == ord("r"):
                    vector[2] += step
                elif key == ord("f"):
                    vector[2] -= step
                status_message = f"updated {edit_mode} offset"
                if args.autosave:
                    save_json(config_path, cfg)
                    status_message = f"autosaved {config_path.name}"
                    print(f"Saved config to {config_path}")
            elif key == ord("h"):
                print("Modes: c=collector, n=needle, t=tool | axes: a/d x, w/s y, r/f z | p=save")
    finally:
        if args.save_on_exit:
            save_json(config_path, cfg)
            print(f"Saved config to {config_path}")
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
