#!/usr/bin/env python3
"""Interactive target configuration UI for spherical collector approach and arc motion."""

from __future__ import annotations

import argparse
import base64
import math
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, List, Optional, Tuple

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
from markerless_collector_pose_optimizer import (
    build_local_basis,
    candidate_offsets,
    inside_workspace,
    look_at_rotation,
    normalize,
    score_candidate,
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


WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_STATE_PATH = WORKSPACE_ROOT / "tools" / "target_conf_state.json"
DEFAULT_POSES_PATH = WORKSPACE_ROOT / "tools" / "target_conf_saved_poses.json"
CALIBRATION_PATH = WORKSPACE_ROOT / "tools" / "eye_in_hand_collector_servo_config.json"


DEFAULT_STATE: Dict = {
    "camera_index": 0,
    "frame_width": 1280,
    "frame_height": 720,
    "fps": 30,
    "serial_port": "/dev/ttyTHS1",
    "baud_rate": 1000000,
    "robot_speed": 15,
    "simulated_robot_coords_mm_deg": [160.0, 0.0, 180.0, 0.0, -90.0, 0.0],
    "tool_from_camera_position_m": [0.055, 0.0, 0.03],
    "tool_from_camera_rpy_rad": [0.0, 0.0, 0.0],
    "needle_from_tool_position_m": [0.0, 0.0, 0.08],
    "needle_from_tool_rpy_rad": [0.0, 0.0, 0.0],
    "target_distance_mm": 150.0,
    "collector_diameter_mm": 50.0,
    "distance_tolerance_mm": 8.0,
    "pose_tolerance_mm": 12.0,
    "camera_hfov_deg": 69.0,
    "camera_fx": 0.0,
    "camera_fy": 0.0,
    "camera_cx": 0.0,
    "camera_cy": 0.0,
    "brightness_threshold": 150,
    "contour_min_area_px": 2200.0,
    "contour_min_radius_px": 18.0,
    "hough_dp": 1.2,
    "hough_min_dist_px": 110.0,
    "hough_param1": 120.0,
    "hough_param2": 24.0,
    "hough_min_radius_px": 16,
    "hough_max_radius_px": 240,
    "workspace_x_min_mm": -320.0,
    "workspace_x_max_mm": 320.0,
    "workspace_y_min_mm": -80.0,
    "workspace_y_max_mm": 420.0,
    "workspace_z_min_mm": -20.0,
    "workspace_z_max_mm": 360.0,
    "preferred_world_up": [0.0, 0.0, 1.0],
    "candidate_yaw_max_deg": 35.0,
    "candidate_pitch_max_deg": 25.0,
    "candidate_angle_step_deg": 7.0,
    "score_position_weight": 1.0,
    "score_orientation_weight": 0.02,
    "score_view_weight": 0.02,
    "min_command_interval_s": 0.55,
    "max_command_step_mm": 18.0,
    "max_rotation_step_deg": 8.0,
    "arc_span_deg": 120.0,
    "arc_steps": 11,
    "arc_interval_s": 0.55,
}


def merge_defaults(base: Dict, updates: Dict) -> Dict:
    merged = dict(base)
    merged.update(updates)
    return merged


def safe_float(text: str, fallback: float) -> float:
    try:
        return float(text)
    except (TypeError, ValueError):
        return fallback


def safe_int(text: str, fallback: int) -> int:
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return fallback


def clamp_step(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def coords_step_towards(
    current_coords: Tuple[float, float, float, float, float, float],
    target_coords: Tuple[float, float, float, float, float, float],
    max_xyz_step_mm: float,
    max_rot_step_deg: float,
) -> Tuple[float, float, float, float, float, float]:
    stepped = []
    for index in range(3):
        stepped.append(clamp_step(float(current_coords[index]), float(target_coords[index]), max_xyz_step_mm))
    for index in range(3, 6):
        stepped.append(clamp_step(float(current_coords[index]), float(target_coords[index]), max_rot_step_deg))
    return tuple(stepped)


def transform_from_xyz(rotation: np.ndarray, xyz_m: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz_m
    return transform


class TargetConfApp:
    def __init__(self, root: tk.Tk, cfg_path: Path, poses_path: Path, control_robot: bool) -> None:
        self.root = root
        self.cfg_path = cfg_path
        self.poses_path = poses_path
        self.control_robot_requested = control_robot
        self.state = self._load_state()
        self.robot: Optional[MyCobot] = None
        self.cap = cv2.VideoCapture(int(self.state["camera_index"]))
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.state['camera_index']}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.state["frame_width"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.state["frame_height"]))
        self.cap.set(cv2.CAP_PROP_FPS, int(self.state["fps"]))

        self.tool_from_camera = build_transform_from_cfg("tool_from_camera_position_m", "tool_from_camera_rpy_rad", self.state)
        self.tool_to_needle = build_transform_from_cfg("needle_from_tool_position_m", "needle_from_tool_rpy_rad", self.state)
        self.world_up = normalize(np.array(self.state["preferred_world_up"], dtype=float))
        self.yaw_offsets_deg = candidate_offsets(float(self.state["candidate_yaw_max_deg"]), float(self.state["candidate_angle_step_deg"]))
        self.pitch_offsets_deg = candidate_offsets(float(self.state["candidate_pitch_max_deg"]), float(self.state["candidate_angle_step_deg"]))

        self.saved_poses: List[Dict] = []
        self.last_payload: Optional[Dict] = None
        self.last_robot_coords: Optional[Tuple[float, float, float, float, float, float]] = None
        self.last_target_coords: Optional[Tuple[float, float, float, float, float, float]] = None
        self.last_command_time = 0.0
        self.arc_waypoints: List[Tuple[float, float, float, float, float, float]] = []
        self.arc_running = False
        self.arc_index = 0
        self.approach_active = False

        self.photo_image: Optional[tk.PhotoImage] = None
        self.status_text = tk.StringVar(value="Idle")
        self.distance_text = tk.StringVar(value="-")
        self.mode_text = tk.StringVar(value="idle")
        self.center_text = tk.StringVar(value="-")
        self.robot_text = tk.StringVar(value="-")
        self.connect_text = tk.StringVar(value="Connect Robot")

        self.target_distance_var = tk.StringVar(value=f"{float(self.state['target_distance_mm']):.1f}")
        self.collector_diameter_var = tk.StringVar(value=f"{float(self.state['collector_diameter_mm']):.1f}")
        self.speed_var = tk.StringVar(value=f"{int(self.state['robot_speed'])}")
        self.serial_port_var = tk.StringVar(value=str(self.state["serial_port"]))
        self.baud_rate_var = tk.StringVar(value=str(int(self.state["baud_rate"])))
        self.auto_arc_var = tk.BooleanVar(value=True)
        self.auto_approach_var = tk.BooleanVar(value=False)

        self._build_ui()

        if self.control_robot_requested:
            self.connect_robot()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.update_loop)

    def _load_state(self) -> Dict:
        merged = dict(DEFAULT_STATE)
        if CALIBRATION_PATH.exists():
            merged = merge_defaults(merged, load_json(CALIBRATION_PATH))
        if self.cfg_path.exists():
            merged = merge_defaults(merged, load_json(self.cfg_path))
        return merged

    def _build_ui(self) -> None:
        self.root.title("Target Config UI")
        self.root.geometry("1480x860")

        container = ttk.Frame(self.root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)
        container.columnconfigure(0, weight=3)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)

        video_frame = ttk.LabelFrame(container, text="Video / Detection")
        video_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        video_frame.rowconfigure(0, weight=1)
        video_frame.columnconfigure(0, weight=1)

        self.video_label = ttk.Label(video_frame)
        self.video_label.grid(row=0, column=0, sticky="nsew")

        control = ttk.LabelFrame(container, text="Robot / Process Controls")
        control.grid(row=0, column=1, sticky="nsew")
        for idx in range(16):
            control.rowconfigure(idx, weight=0)
        control.columnconfigure(1, weight=1)

        row = 0
        ttk.Label(control, text="Distance cible (mm)").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.target_distance_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="Diametre collecteur (mm)").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.collector_diameter_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="Vitesse robot").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.speed_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="Serial port").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.serial_port_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="Baud rate").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.baud_rate_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1

        ttk.Checkbutton(control, text="Auto approach", variable=self.auto_approach_var).grid(row=row, column=0, columnspan=2, sticky="w", pady=3)
        row += 1
        ttk.Checkbutton(control, text="Auto arc quand la cible est atteinte", variable=self.auto_arc_var).grid(row=row, column=0, columnspan=2, sticky="w", pady=3)
        row += 1

        ttk.Button(control, textvariable=self.connect_text, command=self.toggle_robot).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        row += 1
        ttk.Button(control, text="Move To Target", command=self.start_approach).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Run Arc Now", command=self.start_arc_manually).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Save Current Pose", command=self.save_current_pose).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Save Settings", command=self.save_settings).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="STOP", command=self.stop_all).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        row += 1

        status_box = ttk.LabelFrame(control, text="Status")
        status_box.grid(row=row, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        status_box.columnconfigure(1, weight=1)
        labels = [
            ("Mode", self.mode_text),
            ("Distance", self.distance_text),
            ("Collector center", self.center_text),
            ("Robot coords", self.robot_text),
            ("Message", self.status_text),
        ]
        for idx, (name, variable) in enumerate(labels):
            ttk.Label(status_box, text=name).grid(row=idx, column=0, sticky="nw", padx=4, pady=4)
            ttk.Label(status_box, textvariable=variable, wraplength=300, justify=tk.LEFT).grid(row=idx, column=1, sticky="nw", padx=4, pady=4)

    def toggle_robot(self) -> None:
        if self.robot is None:
            self.connect_robot()
        else:
            self.robot = None
            self.connect_text.set("Connect Robot")
            self.status_text.set("Robot disconnected")

    def connect_robot(self) -> None:
        if not PYMCOBOT_AVAILABLE:
            messagebox.showerror("pymycobot missing", "Install pymycobot to control the robot.")
            return
        try:
            self.robot = MyCobot(self.serial_port_var.get().strip(), safe_int(self.baud_rate_var.get(), int(self.state["baud_rate"])))
            time.sleep(1.0)
            self.connect_text.set("Disconnect Robot")
            self.status_text.set("Robot connected")
        except Exception as exc:  # pragma: no cover - hardware specific
            self.robot = None
            messagebox.showerror("Robot connection failed", str(exc))

    def get_runtime_cfg(self) -> Dict:
        cfg = dict(self.state)
        cfg["target_distance_mm"] = safe_float(self.target_distance_var.get(), float(self.state["target_distance_mm"]))
        cfg["collector_diameter_mm"] = safe_float(self.collector_diameter_var.get(), float(self.state["collector_diameter_mm"]))
        cfg["robot_speed"] = safe_int(self.speed_var.get(), int(self.state["robot_speed"]))
        cfg["serial_port"] = self.serial_port_var.get().strip() or str(self.state["serial_port"])
        cfg["baud_rate"] = safe_int(self.baud_rate_var.get(), int(self.state["baud_rate"]))
        cfg["sphere_diameter_m"] = cfg["collector_diameter_mm"] / 1000.0
        return cfg

    def get_robot_coords(self, cfg: Dict) -> Tuple[float, float, float, float, float, float]:
        if self.robot is not None:
            coords = self.robot.get_coords()
            if coords is not None and len(coords) == 6:
                return tuple(float(v) for v in coords)
        return tuple(float(v) for v in cfg["simulated_robot_coords_mm_deg"])

    def optimize_pose(
        self,
        current_coords: Tuple[float, float, float, float, float, float],
        sphere_camera_xyz: np.ndarray,
        cfg: Dict,
    ) -> Dict:
        base_to_tool = coords_deg_to_transform(current_coords)
        base_to_camera = base_to_tool @ self.tool_from_camera
        base_to_collector_center = base_to_camera @ make_transform(np.eye(3, dtype=float), sphere_camera_xyz)
        collector_center_base = base_to_collector_center[:3, 3]
        camera_origin_base = base_to_camera[:3, 3]
        preferred_direction = normalize(camera_origin_base - collector_center_base)
        tangent_x, tangent_y = build_local_basis(preferred_direction, self.world_up)
        sphere_radius_m = 0.5 * float(cfg["collector_diameter_mm"]) / 1000.0
        desired_gap_m = float(cfg["target_distance_mm"]) / 1000.0

        current_needle_tf = base_to_tool @ self.tool_to_needle
        current_needle_position = current_needle_tf[:3, 3]
        current_gap_m = float(np.linalg.norm(current_needle_position - collector_center_base) - sphere_radius_m)

        best_score = None
        best_tool_tf = None
        best_direction = None
        best_needle_target = None

        for yaw_deg in self.yaw_offsets_deg:
            for pitch_deg in self.pitch_offsets_deg:
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
                tool_rotation = look_at_rotation(direction, self.world_up)
                tool_translation = needle_target - tool_rotation @ self.tool_to_needle[:3, 3]
                candidate_tool_tf = transform_from_xyz(tool_rotation, tool_translation)
                candidate_tool_mm = candidate_tool_tf[:3, 3] * 1000.0
                if not inside_workspace(candidate_tool_mm, cfg):
                    continue
                score = score_candidate(candidate_tool_tf, base_to_tool, preferred_direction, direction, cfg)
                if best_score is None or score < best_score:
                    best_score = score
                    best_tool_tf = candidate_tool_tf
                    best_direction = direction
                    best_needle_target = needle_target

        return {
            "base_to_tool": base_to_tool,
            "collector_center_base": collector_center_base,
            "current_gap_m": current_gap_m,
            "best_tool_tf": best_tool_tf,
            "best_direction": best_direction,
            "best_needle_target": best_needle_target,
            "best_coords": transform_to_coords_deg(best_tool_tf) if best_tool_tf is not None else None,
            "best_score": best_score,
            "tangent_x": tangent_x,
            "tangent_y": tangent_y,
            "sphere_radius_m": sphere_radius_m,
            "desired_gap_m": desired_gap_m,
        }

    def build_arc_waypoints(self, optimize_result: Dict, cfg: Dict) -> List[Tuple[float, float, float, float, float, float]]:
        center = optimize_result["collector_center_base"]
        sphere_radius_m = float(optimize_result["sphere_radius_m"])
        desired_gap_m = float(optimize_result["desired_gap_m"])
        tangent_x = optimize_result["tangent_x"]
        tangent_y = optimize_result["tangent_y"]
        side_direction = normalize(-tangent_x)
        span_deg = float(cfg["arc_span_deg"])
        steps = max(3, safe_int(str(cfg["arc_steps"]), int(cfg["arc_steps"])))
        radius = sphere_radius_m + desired_gap_m
        values = np.linspace(math.radians(span_deg * 0.5), math.radians(-span_deg * 0.5), steps)
        waypoints: List[Tuple[float, float, float, float, float, float]] = []
        for value in values:
            direction = normalize(math.cos(value) * side_direction + math.sin(value) * tangent_y)
            needle_target = center + direction * radius
            tool_rotation = look_at_rotation(direction, self.world_up)
            tool_translation = needle_target - tool_rotation @ self.tool_to_needle[:3, 3]
            tool_tf = transform_from_xyz(tool_rotation, tool_translation)
            coords = transform_to_coords_deg(tool_tf)
            waypoints.append(coords)
        return waypoints

    def send_robot_coords(self, current_coords: Tuple[float, float, float, float, float, float], target_coords: Tuple[float, float, float, float, float, float], cfg: Dict) -> None:
        if self.robot is None:
            return
        stepped = coords_step_towards(
            current_coords,
            target_coords,
            float(cfg["max_command_step_mm"]),
            float(cfg["max_rotation_step_deg"]),
        )
        self.robot.send_coords(list(stepped), int(cfg["robot_speed"]), 0)
        self.last_command_time = time.time()
        self.last_target_coords = stepped

    def start_approach(self) -> None:
        self.approach_active = True
        self.arc_running = False
        self.arc_waypoints = []
        self.arc_index = 0
        self.mode_text.set("approach")
        self.status_text.set("Approach started")

    def start_arc_manually(self) -> None:
        if self.last_payload is None or "arc_waypoints" not in self.last_payload:
            self.status_text.set("Arc unavailable: no optimized pose yet")
            return
        self.arc_waypoints = [tuple(float(v) for v in pose) for pose in self.last_payload["arc_waypoints"]]
        self.arc_running = True
        self.approach_active = False
        self.arc_index = 0
        self.mode_text.set("arc")
        self.status_text.set("Arc started")

    def stop_all(self) -> None:
        self.approach_active = False
        self.auto_approach_var.set(False)
        self.arc_running = False
        self.arc_waypoints = []
        self.arc_index = 0
        self.mode_text.set("idle")
        self.status_text.set("Stopped")

    def save_settings(self) -> None:
        cfg = self.get_runtime_cfg()
        self.state.update(
            {
                "target_distance_mm": cfg["target_distance_mm"],
                "collector_diameter_mm": cfg["collector_diameter_mm"],
                "serial_port": cfg["serial_port"],
                "baud_rate": cfg["baud_rate"],
                "robot_speed": cfg["robot_speed"],
            }
        )
        save_json(self.cfg_path, self.state)
        self.status_text.set(f"Settings saved to {self.cfg_path}")

    def save_current_pose(self) -> None:
        payload = {
            "timestamp": time.time(),
            "robot_coords_mm_deg": list(self.last_robot_coords) if self.last_robot_coords is not None else None,
            "target_coords_mm_deg": list(self.last_target_coords) if self.last_target_coords is not None else None,
            "payload": self.last_payload,
        }
        self.saved_poses.append(payload)
        save_json(self.poses_path, {"saved_poses": self.saved_poses})
        self.status_text.set(f"Saved pose #{len(self.saved_poses)} to {self.poses_path}")

    def update_loop(self) -> None:
        try:
            self._update_frame()
        except Exception as exc:  # pragma: no cover - UI safety
            self.status_text.set(f"Error: {exc}")
        self.root.after(30, self.update_loop)

    def _update_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            return

        cfg = self.get_runtime_cfg()
        current_coords = self.get_robot_coords(cfg)
        self.last_robot_coords = current_coords
        gray = preprocess(frame)
        candidate = None
        hough_candidate = detect_circle_hough(gray, cfg)
        contour_candidate = detect_circle_contour(frame, cfg)
        if hough_candidate is not None:
            candidate = hough_candidate
        if candidate is None and contour_candidate is not None:
            candidate = contour_candidate
        status = "collector not detected"

        if candidate is not None:
            sphere_x, sphere_y, sphere_radius = candidate
            payload = estimate_depth_and_offset(sphere_x, sphere_y, sphere_radius, frame.shape[1], frame.shape[0], cfg)
            sphere_camera_xyz = np.array(payload["camera_xyz_m"], dtype=float)
            optimize_result = self.optimize_pose(current_coords, sphere_camera_xyz, cfg)
            best_coords = optimize_result["best_coords"]
            collector_center_base = optimize_result["collector_center_base"]
            current_gap_mm = optimize_result["current_gap_m"] * 1000.0
            target_gap_mm = float(cfg["target_distance_mm"])

            cv2.circle(frame, (int(sphere_x), int(sphere_y)), int(sphere_radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(sphere_x), int(sphere_y)), 4, (0, 0, 255), -1)

            collector_center_mm = collector_center_base * 1000.0
            self.center_text.set(f"{collector_center_mm[0]:.1f}, {collector_center_mm[1]:.1f}, {collector_center_mm[2]:.1f} mm")
            self.distance_text.set(f"{current_gap_mm:.1f} mm (target {target_gap_mm:.1f} mm)")
            self.robot_text.set(", ".join(f"{value:.1f}" for value in current_coords))

            if best_coords is not None:
                current_tool_pos_mm = optimize_result["base_to_tool"][:3, 3] * 1000.0
                target_tool_pos_mm = optimize_result["best_tool_tf"][:3, 3] * 1000.0
                target_pose_error_mm = float(np.linalg.norm(current_tool_pos_mm - target_tool_pos_mm))
                arc_waypoints = self.build_arc_waypoints(optimize_result, cfg)
                self.last_payload = {
                    "sphere_center_px": [round(float(sphere_x), 3), round(float(sphere_y), 3)],
                    "sphere_radius_px": round(float(sphere_radius), 3),
                    "sphere_camera_xyz_m": [round(float(v), 6) for v in sphere_camera_xyz],
                    "collector_center_base_mm": [round(float(v), 3) for v in collector_center_mm],
                    "current_gap_mm": round(current_gap_mm, 3),
                    "target_gap_mm": round(target_gap_mm, 3),
                    "target_coords_mm_deg": [round(float(v), 3) for v in best_coords],
                    "arc_waypoints": [[round(float(v), 3) for v in pose] for pose in arc_waypoints],
                }

                status = (
                    f"gap={current_gap_mm:.1f}mm target={target_gap_mm:.1f}mm "
                    f"pose_err={target_pose_error_mm:.1f}mm score={float(optimize_result['best_score']):.2f}"
                )

                if self.arc_running and self.robot is not None and self.arc_waypoints:
                    if (time.time() - self.last_command_time) >= float(cfg["arc_interval_s"]):
                        target_coords = self.arc_waypoints[min(self.arc_index, len(self.arc_waypoints) - 1)]
                        self.send_robot_coords(current_coords, target_coords, cfg)
                        self.arc_index += 1
                        self.mode_text.set("arc")
                        status = f"Arc step {self.arc_index}/{len(self.arc_waypoints)}"
                        if self.arc_index >= len(self.arc_waypoints):
                            self.arc_running = False
                            self.arc_waypoints = []
                            self.arc_index = 0
                            self.mode_text.set("hold")
                            status = "Arc completed"
                else:
                    if self.auto_approach_var.get():
                        self.approach_active = True
                    if self.approach_active and self.robot is not None and (time.time() - self.last_command_time) >= float(cfg["min_command_interval_s"]):
                        self.send_robot_coords(current_coords, best_coords, cfg)
                        self.mode_text.set("approach")
                    elif self.approach_active:
                        self.mode_text.set("approach")
                    else:
                        self.mode_text.set("preview")

                    if (
                        self.approach_active
                        and abs(current_gap_mm - target_gap_mm) <= float(cfg["distance_tolerance_mm"])
                        and target_pose_error_mm <= float(cfg["pose_tolerance_mm"])
                    ):
                        self.mode_text.set("hold")
                        status = f"Target distance reached ({current_gap_mm:.1f}mm)"
                        if self.auto_arc_var.get() and not self.arc_running:
                            self.arc_waypoints = [tuple(float(v) for v in pose) for pose in arc_waypoints]
                            self.arc_running = True
                            self.approach_active = False
                            self.arc_index = 0
                            self.mode_text.set("arc")
                            status = "Target reached, starting arc"

                cv2.putText(frame, f"gap={current_gap_mm:.1f}mm / target={target_gap_mm:.1f}mm", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f"center(base)={collector_center_mm[0]:.0f},{collector_center_mm[1]:.0f},{collector_center_mm[2]:.0f} mm", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, f"mode={self.mode_text.get()} auto={self.auto_approach_var.get()} arc={self.arc_running}", (12, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                self.mode_text.set("no-solution")
                status = "No valid workspace pose found"
        else:
            self.center_text.set("-")
            self.distance_text.set("-")
            self.robot_text.set(", ".join(f"{value:.1f}" for value in current_coords))
            if self.arc_running:
                self.arc_running = False
                self.arc_waypoints = []
                self.arc_index = 0
            if self.approach_active:
                self.mode_text.set("waiting-vision")
            else:
                self.mode_text.set("idle")

        self.status_text.set(status)
        cv2.putText(frame, status, (12, frame.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0) if candidate is not None else (0, 0, 255), 2, cv2.LINE_AA)

        display = cv2.resize(frame, (960, 540))
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        ok, buffer = cv2.imencode(".png", rgb)
        if ok:
            self.photo_image = tk.PhotoImage(data=base64.b64encode(buffer).decode("ascii"))
            self.video_label.configure(image=self.photo_image)

    def on_close(self) -> None:
        try:
            self.save_settings()
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Target configuration UI for the spherical collector")
    parser.add_argument("--config", default=str(DEFAULT_STATE_PATH))
    parser.add_argument("--poses", default=str(DEFAULT_POSES_PATH))
    parser.add_argument("--control-robot", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    app = TargetConfApp(root, Path(args.config), Path(args.poses), control_robot=args.control_robot)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
