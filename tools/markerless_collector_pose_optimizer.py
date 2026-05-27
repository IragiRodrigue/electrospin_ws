#!/usr/bin/env python3
"""Markerless pose optimizer around a spherical collector for MyCobot 280."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

from eye_in_hand_collector_servo import (
    build_transform_from_cfg,
    coords_deg_to_transform,
    load_json,
    make_transform,
    save_json,
    transform_to_coords_deg,
)
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


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        return vec.copy()
    return vec / norm


def rotation_angle_deg(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    delta = rotation_a.T @ rotation_b
    trace = max(-1.0, min(3.0, float(np.trace(delta))))
    cos_theta = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    return math.degrees(math.acos(cos_theta))


def look_at_rotation(tool_z_axis: np.ndarray, world_up: np.ndarray) -> np.ndarray:
    z_axis = normalize(tool_z_axis)
    x_axis = np.cross(world_up, z_axis)
    if np.linalg.norm(x_axis) <= 1e-6:
        fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(fallback, z_axis))) > 0.95:
            fallback = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = np.cross(fallback, z_axis)
    x_axis = normalize(x_axis)
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack([x_axis, y_axis, z_axis])


def build_local_basis(direction: np.ndarray, world_up: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    tangent_x = np.cross(world_up, direction)
    if np.linalg.norm(tangent_x) <= 1e-6:
        tangent_x = np.cross(np.array([1.0, 0.0, 0.0], dtype=float), direction)
    tangent_x = normalize(tangent_x)
    tangent_y = normalize(np.cross(direction, tangent_x))
    return tangent_x, tangent_y


def candidate_offsets(max_angle_deg: float, step_deg: float) -> List[float]:
    if step_deg <= 0.0:
        return [0.0]
    values = []
    angle = -max_angle_deg
    while angle <= max_angle_deg + 1e-6:
        values.append(round(angle, 6))
        angle += step_deg
    if 0.0 not in values:
        values.append(0.0)
    return sorted(set(values))


def inside_workspace(position_mm: np.ndarray, cfg: Dict) -> bool:
    return (
        float(cfg["workspace_x_min_mm"]) <= float(position_mm[0]) <= float(cfg["workspace_x_max_mm"])
        and float(cfg["workspace_y_min_mm"]) <= float(position_mm[1]) <= float(cfg["workspace_y_max_mm"])
        and float(cfg["workspace_z_min_mm"]) <= float(position_mm[2]) <= float(cfg["workspace_z_max_mm"])
    )


def score_candidate(
    candidate_tool_tf: np.ndarray,
    current_tool_tf: np.ndarray,
    preferred_direction: np.ndarray,
    candidate_direction: np.ndarray,
    cfg: Dict,
) -> float:
    pos_delta_m = float(np.linalg.norm(candidate_tool_tf[:3, 3] - current_tool_tf[:3, 3]))
    orient_delta_deg = rotation_angle_deg(current_tool_tf[:3, :3], candidate_tool_tf[:3, :3])
    view_delta_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(preferred_direction, candidate_direction))))))
    return (
        float(cfg["score_position_weight"]) * pos_delta_m
        + float(cfg["score_orientation_weight"]) * orient_delta_deg
        + float(cfg["score_view_weight"]) * view_delta_deg
    )


def main():
    parser = argparse.ArgumentParser(description="Markerless spherical collector pose optimizer")
    parser.add_argument("--config", default="tools/markerless_collector_pose_optimizer_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--control-robot", action="store_true")
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--speed", type=int, default=16)
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
    world_up = normalize(np.array(cfg.get("preferred_world_up", [0.0, 0.0, 1.0]), dtype=float))
    simulated_coords = cfg.get("simulated_robot_coords_mm_deg", [160.0, 0.0, 180.0, 0.0, -90.0, 0.0])
    sphere_radius_m = 0.5 * float(cfg["sphere_diameter_m"])
    desired_gap_m = float(cfg["desired_gap_m"])
    yaw_offsets_deg = candidate_offsets(float(cfg["candidate_yaw_max_deg"]), float(cfg["candidate_angle_step_deg"]))
    pitch_offsets_deg = candidate_offsets(float(cfg["candidate_pitch_max_deg"]), float(cfg["candidate_angle_step_deg"]))

    auto_optimize = False
    last_payload = None
    last_command_time = 0.0

    print("Markerless collector pose optimizer started.")
    print("Keys: o=optimize once, g=toggle auto optimize, s=save last JSON, q=quit")
    print("This tries to choose a better approach pose around the sphere, not just recenter the image.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            gray = preprocess(frame)
            candidate = None
            if args.method in {"auto", "hough"}:
                candidate = detect_circle_hough(gray, cfg)
            if candidate is None and args.method in {"auto", "contour"}:
                candidate = detect_circle_contour(frame, cfg)

            status = "collector sphere not detected"
            best_coords = None
            apply_now = False

            if candidate is not None:
                x, y, radius_px = candidate
                payload = estimate_depth_and_offset(x, y, radius_px, frame.shape[1], frame.shape[0], cfg)
                sphere_camera_xyz = np.array(payload["camera_xyz_m"], dtype=float)

                if robot is not None:
                    current_coords = robot.get_coords()
                else:
                    current_coords = simulated_coords
                if current_coords is None or len(current_coords) != 6:
                    raise RuntimeError(f"Invalid robot coords: {current_coords}")

                base_to_tool = coords_deg_to_transform(tuple(float(v) for v in current_coords))
                base_to_camera = base_to_tool @ tool_from_camera
                sphere_camera_tf = make_transform(np.eye(3, dtype=float), sphere_camera_xyz)
                base_to_collector_center = base_to_camera @ sphere_camera_tf
                collector_center_base = base_to_collector_center[:3, 3]
                camera_origin_base = base_to_camera[:3, 3]
                preferred_direction = normalize(camera_origin_base - collector_center_base)
                tangent_x, tangent_y = build_local_basis(preferred_direction, world_up)

                best_score = None
                best_tool_tf = None
                best_direction = None

                for yaw_deg in yaw_offsets_deg:
                    for pitch_deg in pitch_offsets_deg:
                        yaw_rad = math.radians(yaw_deg)
                        pitch_rad = math.radians(pitch_deg)
                        direction = normalize(
                            preferred_direction
                            + math.tan(yaw_rad) * tangent_x
                            + math.tan(pitch_rad) * tangent_y
                        )
                        if np.linalg.norm(direction) <= 1e-6:
                            continue

                        needle_target = collector_center_base + direction * (sphere_radius_m + desired_gap_m)
                        tool_rotation = look_at_rotation(direction, world_up)
                        tool_translation = needle_target - tool_rotation @ tool_to_needle[:3, 3]
                        candidate_tool_tf = make_transform(tool_rotation, tool_translation)
                        candidate_tool_mm = candidate_tool_tf[:3, 3] * 1000.0

                        if not inside_workspace(candidate_tool_mm, cfg):
                            continue

                        score = score_candidate(candidate_tool_tf, base_to_tool, preferred_direction, direction, cfg)
                        if best_score is None or score < best_score:
                            best_score = score
                            best_tool_tf = candidate_tool_tf
                            best_direction = direction

                cv2.circle(debug_frame, (int(x), int(y)), int(radius_px), (0, 255, 0), 2)
                cv2.circle(debug_frame, (int(x), int(y)), 4, (0, 0, 255), -1)

                if best_tool_tf is not None and best_direction is not None:
                    best_coords = transform_to_coords_deg(best_tool_tf)
                    target_needle = collector_center_base + best_direction * (sphere_radius_m + desired_gap_m)
                    best_tool_mm = best_tool_tf[:3, 3] * 1000.0
                    collector_center_mm = collector_center_base * 1000.0
                    target_needle_mm = target_needle * 1000.0
                    score = float(best_score)

                    status = (
                        f"collector=({collector_center_mm[0]:.0f},{collector_center_mm[1]:.0f},{collector_center_mm[2]:.0f})mm "
                        f"tool=({best_tool_mm[0]:.0f},{best_tool_mm[1]:.0f},{best_tool_mm[2]:.0f}) score={score:.2f}"
                    )

                    cv2.putText(
                        debug_frame,
                        f"sphere camera xyz = {sphere_camera_xyz[0]:+.3f}, {sphere_camera_xyz[1]:+.3f}, {sphere_camera_xyz[2]:+.3f} m",
                        (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (255, 220, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        f"collector base mm = {collector_center_mm[0]:.1f}, {collector_center_mm[1]:.1f}, {collector_center_mm[2]:.1f}",
                        (12, 58),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        f"best tool mm = {best_tool_mm[0]:.1f}, {best_tool_mm[1]:.1f}, {best_tool_mm[2]:.1f}",
                        (12, 86),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        f"target needle mm = {target_needle_mm[0]:.1f}, {target_needle_mm[1]:.1f}, {target_needle_mm[2]:.1f}",
                        (12, 114),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        debug_frame,
                        f"best coords = {[round(float(v), 1) for v in best_coords]}",
                        (12, 142),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.53,
                        (255, 180, 0),
                        2,
                        cv2.LINE_AA,
                    )

                    last_payload = {
                        "sphere_center_px": [round(float(x), 4), round(float(y), 4)],
                        "sphere_radius_px": round(float(radius_px), 4),
                        "sphere_camera_xyz_m": [round(float(v), 6) for v in sphere_camera_xyz],
                        "collector_center_base_mm": [round(float(v), 4) for v in collector_center_mm],
                        "target_needle_mm": [round(float(v), 4) for v in target_needle_mm],
                        "best_tool_mm": [round(float(v), 4) for v in best_tool_mm],
                        "best_robot_coords_mm_deg": [round(float(v), 4) for v in best_coords],
                        "score": round(score, 6),
                    }

                    if auto_optimize and robot is not None and (time.time() - last_command_time) >= float(cfg["min_command_interval_s"]):
                        apply_now = True
                else:
                    status = "sphere detected but no workspace-valid optimized pose found"
            else:
                cv2.putText(
                    debug_frame,
                    status,
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.70,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.putText(
                debug_frame,
                status,
                (12, frame.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if best_coords is not None else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            if not args.headless:
                cv2.imshow("markerless_collector_pose_optimizer", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("g"):
                auto_optimize = not auto_optimize
                print(f"Auto-optimize set to {auto_optimize}")
            if key == ord("o"):
                apply_now = True
            if key == ord("s") and args.save_last_json and last_payload is not None:
                save_json(Path(args.save_last_json), last_payload)
                print(f"Saved result to {args.save_last_json}")

            if apply_now and robot is not None and best_coords is not None:
                robot.send_coords(list(best_coords), args.speed, 0)
                last_command_time = time.time()
                print(f"Sent optimized coords: {[round(float(v), 2) for v in best_coords]}")
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
