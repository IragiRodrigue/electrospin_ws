#!/usr/bin/env python3
"""Safer real-time visual servo for a spherical collector with MyCobot."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from eye_in_hand_collector_servo import (
    build_transform_from_cfg,
    coords_deg_to_transform,
    load_json,
    make_transform,
    save_json,
)
from markerless_collector_pose_optimizer import inside_workspace, normalize
from markerless_collector_tracker import (
    detect_circle_contour,
    detect_circle_hough,
    estimate_depth_and_offset,
    preprocess,
)

try:
    from pymycobot.mycobot import MyCobot

    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


def clamp_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or norm <= 1e-9:
        return vec
    return vec * (max_norm / norm)


def ema_update(previous: Optional[np.ndarray], current: np.ndarray, alpha: float) -> np.ndarray:
    if previous is None:
        return current.copy()
    return (1.0 - alpha) * previous + alpha * current


def refine_sphere_head_candidate(frame: np.ndarray, candidate: Tuple[float, float, float], cfg: Dict) -> Tuple[float, float, float]:
    x, y, radius = candidate
    margin_scale = float(cfg.get("head_refine_margin_scale", 1.5))
    top_scale = float(cfg.get("head_refine_top_scale", 1.35))
    bottom_scale = float(cfg.get("head_refine_bottom_scale", 0.25))
    roi_x0 = max(0, int(x - radius * margin_scale))
    roi_x1 = min(frame.shape[1], int(x + radius * margin_scale))
    roi_y0 = max(0, int(y - radius * top_scale))
    roi_y1 = min(frame.shape[0], int(y + radius * bottom_scale))
    if roi_x1 - roi_x0 < 20 or roi_y1 - roi_y0 < 20:
        return candidate

    roi = frame[roi_y0:roi_y1, roi_x0:roi_x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    _, mask = cv2.threshold(value, int(cfg.get("head_refine_value_threshold", 150)), 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1.0
    min_area = float(cfg.get("head_refine_min_area_px", 500.0))
    min_radius = float(cfg.get("head_refine_min_radius_px", 10.0))
    max_aspect = float(cfg.get("head_refine_max_aspect_ratio", 1.35))

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue
        rx, ry, rw, rh = cv2.boundingRect(contour)
        aspect = max(rw / max(rh, 1.0), rh / max(rw, 1.0))
        if aspect > max_aspect:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        (cx, cy), cr = cv2.minEnclosingCircle(contour)
        if cr < min_radius:
            continue
        score = circularity * area
        if score > best_score:
            best = (float(cx + roi_x0), float(cy + roi_y0), float(cr))
            best_score = score

    return best if best is not None else candidate


def detect_candidate(frame: np.ndarray, cfg: Dict, method: str) -> Tuple[Optional[Tuple[float, float, float]], str]:
    gray = preprocess(frame)
    candidate = None
    detect_mode = "none"
    if method in {"auto", "hough"}:
        candidate = detect_circle_hough(gray, cfg)
        if candidate is not None:
            detect_mode = "hough"
    if candidate is None and method in {"auto", "contour"}:
        candidate = detect_circle_contour(frame, cfg)
        if candidate is not None:
            detect_mode = "contour"
    if candidate is not None and bool(cfg.get("sphere_head_only", True)):
        candidate = refine_sphere_head_candidate(frame, candidate, cfg)
        detect_mode = "head"
    return candidate, detect_mode


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe real-time spherical collector servo for MyCobot")
    parser.add_argument("--config", default="tools/real_time_collector_servo_safe_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--control-robot", action="store_true")
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--speed", type=int, default=12)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-last-json", default="")
    parser.add_argument("--method", default="auto", choices=["auto", "hough", "contour"])
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

    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.get('camera_index', 0)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("frame_width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("frame_height", 720)))
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    robot = None
    if args.control_robot:
        if not PYMCOBOT_AVAILABLE:
            raise RuntimeError("pymycobot is not installed. Install it or disable --control-robot.")
        robot = MyCobot(cfg.get("serial_port", "/dev/ttyTHS1"), int(cfg.get("baud_rate", 1000000)))
        time.sleep(1.0)

    tool_from_camera = build_transform_from_cfg("tool_from_camera_position_m", "tool_from_camera_rpy_rad", cfg)
    tool_to_needle = build_transform_from_cfg("needle_from_tool_position_m", "needle_from_tool_rpy_rad", cfg)
    sphere_radius_m = 0.5 * float(cfg.get("sphere_diameter_m", 0.05))
    target_gap_m = float(cfg.get("target_distance_mm", 150.0)) / 1000.0
    alpha = float(cfg.get("filter_alpha", 0.22))
    max_step_m = float(cfg.get("max_step_m", 0.008))
    max_position_error_m = float(cfg.get("max_position_error_m", 0.06))
    min_command_interval_s = float(cfg.get("min_command_interval_s", 0.65))
    command_deadband_mm = float(cfg.get("command_deadband_mm", 2.5))
    lost_timeout_s = float(cfg.get("lost_timeout_s", 0.8))
    max_depth_jump_m = float(cfg.get("max_depth_jump_m", 0.08))
    max_xy_jump_m = float(cfg.get("max_xy_jump_m", 0.05))
    simulated_coords = tuple(float(v) for v in cfg.get("simulated_robot_coords_mm_deg", [160.0, 0.0, 180.0, 0.0, -90.0, 0.0]))

    auto_follow = False
    filtered_collector_base = None
    filtered_radius_px = None
    last_detection_time = 0.0
    last_payload = None
    last_command_time = 0.0

    print("Safe real-time collector servo started.")
    print("Keys: g=toggle follow, m=single safe correction, s=save latest JSON, q=quit")
    print("This mode keeps the current tool orientation and only applies small filtered translations.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            candidate, detect_mode = detect_candidate(frame, cfg, args.method)
            status = "collector not detected"
            target_coords = None
            apply_now = False
            confidence_ok = False

            if candidate is not None:
                x, y, radius_px = candidate
                payload = estimate_depth_and_offset(x, y, radius_px, frame.shape[1], frame.shape[0], cfg)
                sphere_camera_xyz = np.array(payload["camera_xyz_m"], dtype=float)

                current_coords = tuple(float(v) for v in (robot.get_coords() if robot is not None else simulated_coords))
                if current_coords is None or len(current_coords) != 6:
                    raise RuntimeError(f"Invalid robot coords: {current_coords}")

                base_to_tool = coords_deg_to_transform(current_coords)
                base_to_camera = base_to_tool @ tool_from_camera
                collector_center_base = (base_to_camera @ make_transform(np.eye(3, dtype=float), sphere_camera_xyz))[:3, 3]

                jump_ok = True
                if filtered_collector_base is not None:
                    delta = collector_center_base - filtered_collector_base
                    if abs(float(delta[2])) > max_depth_jump_m or float(np.linalg.norm(delta[:2])) > max_xy_jump_m:
                        jump_ok = False

                if jump_ok:
                    filtered_collector_base = ema_update(filtered_collector_base, collector_center_base, alpha)
                    filtered_radius_px = float(radius_px) if filtered_radius_px is None else (1.0 - alpha) * filtered_radius_px + alpha * float(radius_px)
                    last_detection_time = time.time()
                    confidence_ok = True

                if filtered_collector_base is not None:
                    camera_origin_base = base_to_camera[:3, 3]
                    approach_direction = normalize(camera_origin_base - filtered_collector_base)
                    if float(np.linalg.norm(approach_direction)) > 1e-6:
                        desired_needle_base = filtered_collector_base + approach_direction * (sphere_radius_m + target_gap_m)
                        current_tool_rotation = base_to_tool[:3, :3]
                        desired_tool_translation = desired_needle_base - current_tool_rotation @ tool_to_needle[:3, 3]
                        current_tool_translation = base_to_tool[:3, 3]
                        tool_delta = desired_tool_translation - current_tool_translation
                        tool_delta = clamp_norm(tool_delta, max_step_m)
                        tool_error = desired_tool_translation - current_tool_translation
                        error_norm = float(np.linalg.norm(tool_error))

                        candidate_tool_translation = current_tool_translation + tool_delta
                        candidate_tool_mm = candidate_tool_translation * 1000.0
                        if inside_workspace(candidate_tool_mm, cfg) and error_norm <= max_position_error_m:
                            target_coords = (
                                float(candidate_tool_mm[0]),
                                float(candidate_tool_mm[1]),
                                float(candidate_tool_mm[2]),
                                float(current_coords[3]),
                                float(current_coords[4]),
                                float(current_coords[5]),
                            )
                            status = (
                                f"mode={detect_mode} gap={(np.linalg.norm((base_to_tool @ tool_to_needle)[:3, 3] - filtered_collector_base) - sphere_radius_m) * 1000.0:.1f}mm "
                                f"target={target_gap_m * 1000.0:.1f}mm err={error_norm * 1000.0:.1f}mm"
                            )
                            last_payload = {
                                "sphere_center_px": [round(float(x), 4), round(float(y), 4)],
                                "sphere_radius_px": round(float(radius_px), 4),
                                "filtered_sphere_radius_px": round(float(filtered_radius_px), 4) if filtered_radius_px is not None else None,
                                "sphere_camera_xyz_m": [round(float(v), 6) for v in sphere_camera_xyz],
                                "collector_center_base_m": [round(float(v), 6) for v in filtered_collector_base],
                                "target_robot_coords_mm_deg": [round(float(v), 4) for v in target_coords],
                                "detect_mode": detect_mode,
                                "follow_enabled": auto_follow,
                                "tool_error_mm": round(error_norm * 1000.0, 4),
                            }
                            if auto_follow and robot is not None and confidence_ok and (time.time() - last_command_time) >= min_command_interval_s:
                                if error_norm * 1000.0 >= command_deadband_mm:
                                    apply_now = True
                        else:
                            status = "target rejected by workspace or safety bounds"

                cv2.circle(debug_frame, (int(x), int(y)), int(radius_px), (0, 255, 0), 2)
                cv2.circle(debug_frame, (int(x), int(y)), 4, (255, 0, 0), -1)
            else:
                if auto_follow and (time.time() - last_detection_time) > lost_timeout_s:
                    status = "detection lost: follow paused"

            overlay_color = (0, 255, 0) if target_coords is not None else (0, 0, 255)
            cv2.putText(debug_frame, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.60, overlay_color, 2, cv2.LINE_AA)
            cv2.putText(
                debug_frame,
                f"follow={auto_follow} method={detect_mode} radius={0.0 if filtered_radius_px is None else filtered_radius_px:.1f}px",
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 220, 0),
                2,
                cv2.LINE_AA,
            )
            if filtered_collector_base is not None:
                collector_mm = filtered_collector_base * 1000.0
                cv2.putText(
                    debug_frame,
                    f"collector base mm = {collector_mm[0]:.1f}, {collector_mm[1]:.1f}, {collector_mm[2]:.1f}",
                    (12, 86),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            if target_coords is not None:
                cv2.putText(
                    debug_frame,
                    f"target tool mm = {target_coords[0]:.1f}, {target_coords[1]:.1f}, {target_coords[2]:.1f}",
                    (12, 114),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if not args.headless:
                cv2.imshow("real_time_collector_servo_safe", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("g"):
                auto_follow = not auto_follow
                print(f"Auto-follow set to {auto_follow}")
            if key == ord("m"):
                apply_now = True
            if key == ord("s") and args.save_last_json and last_payload is not None:
                save_json(Path(args.save_last_json), last_payload)
                print(f"Saved result to {args.save_last_json}")

            if apply_now and robot is not None and target_coords is not None and confidence_ok:
                robot.send_coords(list(target_coords), args.speed, 0)
                last_command_time = time.time()
                print(f"Sent coords: {[round(float(v), 2) for v in target_coords]}")
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
