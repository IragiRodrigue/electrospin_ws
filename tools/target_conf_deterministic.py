#!/usr/bin/env python3
"""Deterministic 3-phase collector workflow: localize, optimize, execute."""

from __future__ import annotations

import argparse
import base64
import importlib.util
import math
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, List, Optional, Tuple


THIS_DIR = Path(__file__).resolve().parent
BASE_TARGET_CONF_PATH = THIS_DIR / "target_conf.py"

spec = importlib.util.spec_from_file_location("target_conf_base_ui", BASE_TARGET_CONF_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load base target_conf module from {BASE_TARGET_CONF_PATH}")
base_ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base_ui)

try:
    from pymycobot.mycobot import MyCobot

    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


WORKSPACE_ROOT = THIS_DIR.parent
DEFAULT_STATE_PATH = WORKSPACE_ROOT / "tools" / "target_conf_deterministic_state.json"
DEFAULT_COLLECTOR_POSE_PATH = WORKSPACE_ROOT / "tools" / "collector_locked_pose.json"
DEFAULT_TARGET_POSE_PATH = WORKSPACE_ROOT / "tools" / "collector_target_pose.json"
CALIBRATION_PATH = WORKSPACE_ROOT / "tools" / "eye_in_hand_collector_servo_config.json"


DETERMINISTIC_DEFAULTS: Dict = dict(base_ui.DEFAULT_STATE)
DETERMINISTIC_DEFAULTS.update(
    {
        "localize_sample_count": 20,
        "localize_sample_interval_s": 0.18,
        "localize_min_radius_px": 10.0,
        "pre_approach_extra_mm": 60.0,
        "execute_interval_s": 1.0,
        "arc_interval_s": 0.7,
        "sphere_head_only": True,
        "head_refine_margin_scale": 1.5,
        "head_refine_top_scale": 1.35,
        "head_refine_bottom_scale": 0.25,
        "head_refine_value_threshold": 150,
        "head_refine_min_area_px": 500.0,
        "head_refine_min_radius_px": 10.0,
        "head_refine_max_aspect_ratio": 1.35,
    }
)


def median_vector(samples: List[List[float]]) -> List[float]:
    return [float(v) for v in base_ui.np.median(base_ui.np.array(samples, dtype=float), axis=0)]


def refine_sphere_head_candidate(frame, candidate, cfg: Dict):
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
    hsv = base_ui.cv2.cvtColor(roi, base_ui.cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    _, mask = base_ui.cv2.threshold(value, int(cfg.get("head_refine_value_threshold", 150)), 255, base_ui.cv2.THRESH_BINARY)
    kernel = base_ui.np.ones((5, 5), dtype=base_ui.np.uint8)
    mask = base_ui.cv2.morphologyEx(mask, base_ui.cv2.MORPH_OPEN, kernel)
    mask = base_ui.cv2.morphologyEx(mask, base_ui.cv2.MORPH_CLOSE, kernel)

    contours, _ = base_ui.cv2.findContours(mask, base_ui.cv2.RETR_EXTERNAL, base_ui.cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1.0
    min_area = float(cfg.get("head_refine_min_area_px", 500.0))
    min_radius = float(cfg.get("head_refine_min_radius_px", 10.0))
    max_aspect = float(cfg.get("head_refine_max_aspect_ratio", 1.35))

    for contour in contours:
        area = base_ui.cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = base_ui.cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue
        rx, ry, rw, rh = base_ui.cv2.boundingRect(contour)
        aspect = max(rw / max(rh, 1.0), rh / max(rw, 1.0))
        if aspect > max_aspect:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        (cx, cy), cr = base_ui.cv2.minEnclosingCircle(contour)
        if cr < min_radius:
            continue
        score = circularity * area
        if score > best_score:
            best = (float(cx + roi_x0), float(cy + roi_y0), float(cr))
            best_score = score

    return best if best is not None else candidate


class DeterministicTargetConfApp:
    def __init__(self, root: tk.Tk, state_path: Path, collector_path: Path, target_path: Path, control_robot: bool) -> None:
        self.root = root
        self.state_path = state_path
        self.collector_path = collector_path
        self.target_path = target_path
        self.state = self._load_state()

        self.cap = base_ui.cv2.VideoCapture(int(self.state["camera_index"]))
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.state['camera_index']}")
        self.cap.set(base_ui.cv2.CAP_PROP_FRAME_WIDTH, int(self.state["frame_width"]))
        self.cap.set(base_ui.cv2.CAP_PROP_FRAME_HEIGHT, int(self.state["frame_height"]))
        self.cap.set(base_ui.cv2.CAP_PROP_FPS, int(self.state["fps"]))

        self.robot: Optional[MyCobot] = None
        self.control_robot_requested = control_robot

        self.tool_from_camera = base_ui.build_transform_from_cfg("tool_from_camera_position_m", "tool_from_camera_rpy_rad", self.state)
        self.tool_to_needle = base_ui.build_transform_from_cfg("needle_from_tool_position_m", "needle_from_tool_rpy_rad", self.state)
        self.world_up = base_ui.normalize(base_ui.np.array(self.state["preferred_world_up"], dtype=float))
        self.yaw_offsets_deg = base_ui.candidate_offsets(float(self.state["candidate_yaw_max_deg"]), float(self.state["candidate_angle_step_deg"]))
        self.pitch_offsets_deg = base_ui.candidate_offsets(float(self.state["candidate_pitch_max_deg"]), float(self.state["candidate_angle_step_deg"]))

        self.phase = "idle"
        self.localization_samples: List[Dict] = []
        self.last_localize_time = 0.0
        self.locked_collector: Optional[Dict] = None
        self.optimized_target: Optional[Dict] = None
        self.execution_waypoints: List[Tuple[float, float, float, float, float, float]] = []
        self.execution_index = 0
        self.last_command_time = 0.0
        self.last_robot_coords: Optional[Tuple[float, float, float, float, float, float]] = None
        self.photo_image: Optional[tk.PhotoImage] = None

        self.status_text = tk.StringVar(value="Idle")
        self.phase_text = tk.StringVar(value="idle")
        self.detect_text = tk.StringVar(value="-")
        self.collector_text = tk.StringVar(value="-")
        self.target_text = tk.StringVar(value="-")
        self.robot_text = tk.StringVar(value="-")
        self.connect_text = tk.StringVar(value="Connect Robot")

        self.target_distance_var = tk.StringVar(value=f"{float(self.state['target_distance_mm']):.1f}")
        self.collector_diameter_var = tk.StringVar(value=f"{float(self.state['collector_diameter_mm']):.1f}")
        self.speed_var = tk.StringVar(value=f"{int(self.state['robot_speed'])}")
        self.serial_port_var = tk.StringVar(value=str(self.state["serial_port"]))
        self.baud_rate_var = tk.StringVar(value=str(int(self.state["baud_rate"])))
        self.localize_samples_var = tk.StringVar(value=str(int(self.state["localize_sample_count"])))
        self.sphere_head_only_var = tk.BooleanVar(value=bool(self.state.get("sphere_head_only", True)))

        self._build_ui()
        if self.control_robot_requested:
            self.connect_robot()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.update_loop)

    def _load_state(self) -> Dict:
        merged = dict(DETERMINISTIC_DEFAULTS)
        if CALIBRATION_PATH.exists():
            merged = base_ui.merge_defaults(merged, base_ui.load_json(CALIBRATION_PATH))
        if self.state_path.exists():
            merged = base_ui.merge_defaults(merged, base_ui.load_json(self.state_path))
        return merged

    def _build_ui(self) -> None:
        self.root.title("Deterministic Target Config UI")
        self.root.geometry("1500x880")

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

        control = ttk.LabelFrame(container, text="Deterministic Workflow Controls")
        control.grid(row=0, column=1, sticky="nsew")
        control.columnconfigure(1, weight=1)

        row = 0
        fields = [
            ("Distance cible (mm)", self.target_distance_var),
            ("Diametre collecteur (mm)", self.collector_diameter_var),
            ("Vitesse robot", self.speed_var),
            ("Serial port", self.serial_port_var),
            ("Baud rate", self.baud_rate_var),
            ("Nb samples localization", self.localize_samples_var),
        ]
        for label, variable in fields:
            ttk.Label(control, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(control, textvariable=variable, width=16).grid(row=row, column=1, sticky="ew", pady=4)
            row += 1

        ttk.Checkbutton(control, text="Sphere head only", variable=self.sphere_head_only_var).grid(row=row, column=0, columnspan=2, sticky="w", pady=(2, 6))
        row += 1

        ttk.Button(control, textvariable=self.connect_text, command=self.toggle_robot).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        row += 1
        ttk.Button(control, text="1. Localize Collector", command=self.start_localization).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Save Collector Pose", command=self.save_locked_collector).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Load Collector Pose", command=self.load_locked_collector).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="2. Optimize Target", command=self.optimize_locked_target).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Save Target Pose", command=self.save_optimized_target).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Load Target Pose", command=self.load_optimized_target).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="3. Execute Approach", command=self.execute_approach).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="Execute Arc", command=self.execute_arc).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1
        ttk.Button(control, text="STOP", command=self.stop_all).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        row += 1
        ttk.Button(control, text="Save Settings", command=self.save_settings).grid(row=row, column=0, columnspan=2, sticky="ew", pady=(4, 8))
        row += 1

        status_box = ttk.LabelFrame(control, text="Status")
        status_box.grid(row=row, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        status_box.columnconfigure(1, weight=1)
        labels = [
            ("Phase", self.phase_text),
            ("Detection", self.detect_text),
            ("Collector lock", self.collector_text),
            ("Target", self.target_text),
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
            self.robot = MyCobot(self.serial_port_var.get().strip(), base_ui.safe_int(self.baud_rate_var.get(), int(self.state["baud_rate"])))
            time.sleep(1.0)
            self.connect_text.set("Disconnect Robot")
            self.status_text.set("Robot connected")
        except Exception as exc:
            self.robot = None
            messagebox.showerror("Robot connection failed", str(exc))

    def get_runtime_cfg(self) -> Dict:
        cfg = dict(self.state)
        cfg["target_distance_mm"] = base_ui.safe_float(self.target_distance_var.get(), float(self.state["target_distance_mm"]))
        cfg["collector_diameter_mm"] = base_ui.safe_float(self.collector_diameter_var.get(), float(self.state["collector_diameter_mm"]))
        cfg["robot_speed"] = base_ui.safe_int(self.speed_var.get(), int(self.state["robot_speed"]))
        cfg["serial_port"] = self.serial_port_var.get().strip() or str(self.state["serial_port"])
        cfg["baud_rate"] = base_ui.safe_int(self.baud_rate_var.get(), int(self.state["baud_rate"]))
        cfg["localize_sample_count"] = base_ui.safe_int(self.localize_samples_var.get(), int(self.state["localize_sample_count"]))
        cfg["sphere_head_only"] = bool(self.sphere_head_only_var.get())
        cfg["sphere_diameter_m"] = cfg["collector_diameter_mm"] / 1000.0
        return cfg

    def get_robot_coords(self, cfg: Dict) -> Tuple[float, float, float, float, float, float]:
        if self.robot is not None:
            coords = self.robot.get_coords()
            if coords is not None and len(coords) == 6:
                return tuple(float(v) for v in coords)
        return tuple(float(v) for v in cfg["simulated_robot_coords_mm_deg"])

    def detect_candidate(self, frame, cfg: Dict):
        gray = base_ui.preprocess(frame)
        candidate = None
        hough_candidate = base_ui.detect_circle_hough(gray, cfg)
        contour_candidate = base_ui.detect_circle_contour(frame, cfg)
        if hough_candidate is not None:
            candidate = hough_candidate
        if candidate is None and contour_candidate is not None:
            candidate = contour_candidate
        if candidate is not None and bool(cfg.get("sphere_head_only", True)):
            candidate = refine_sphere_head_candidate(frame, candidate, cfg)
        return candidate

    def localize_collector_from_candidate(self, current_coords, sphere_xyz_m):
        base_to_tool = base_ui.coords_deg_to_transform(current_coords)
        base_to_camera = base_to_tool @ self.tool_from_camera
        base_to_collector_center = base_to_camera @ base_ui.make_transform(base_ui.np.eye(3, dtype=float), sphere_xyz_m)
        return base_to_collector_center[:3, 3]

    def optimize_from_locked_pose(self, current_coords, collector_center_base_m, collector_radius_m, cfg: Dict) -> Optional[Dict]:
        base_to_tool = base_ui.coords_deg_to_transform(current_coords)
        camera_origin_base = (base_to_tool @ self.tool_from_camera)[:3, 3]
        preferred_direction = base_ui.normalize(camera_origin_base - collector_center_base_m)
        tangent_x, tangent_y = base_ui.build_local_basis(preferred_direction, self.world_up)
        desired_gap_m = float(cfg["target_distance_mm"]) / 1000.0

        best_score = None
        best_tool_tf = None
        best_direction = None
        best_needle_target = None

        for yaw_deg in self.yaw_offsets_deg:
            for pitch_deg in self.pitch_offsets_deg:
                yaw_rad = math.radians(yaw_deg)
                pitch_rad = math.radians(pitch_deg)
                direction = base_ui.normalize(
                    preferred_direction
                    + math.tan(yaw_rad) * tangent_x
                    + math.tan(pitch_rad) * tangent_y
                )
                if base_ui.np.linalg.norm(direction) <= 1e-6:
                    continue
                needle_target = collector_center_base_m + direction * (collector_radius_m + desired_gap_m)
                tool_rotation = base_ui.look_at_rotation(direction, self.world_up)
                tool_translation = needle_target - tool_rotation @ self.tool_to_needle[:3, 3]
                candidate_tool_tf = base_ui.transform_from_xyz(tool_rotation, tool_translation)
                candidate_tool_mm = candidate_tool_tf[:3, 3] * 1000.0
                if not base_ui.inside_workspace(candidate_tool_mm, cfg):
                    continue
                score = base_ui.score_candidate(candidate_tool_tf, base_to_tool, preferred_direction, direction, cfg)
                if best_score is None or score < best_score:
                    best_score = score
                    best_tool_tf = candidate_tool_tf
                    best_direction = direction
                    best_needle_target = needle_target

        if best_tool_tf is None or best_direction is None or best_needle_target is None:
            return None

        return {
            "best_score": float(best_score),
            "best_tool_tf": best_tool_tf,
            "best_direction": best_direction,
            "best_coords": base_ui.transform_to_coords_deg(best_tool_tf),
            "best_needle_target": best_needle_target,
            "tangent_x": tangent_x,
            "tangent_y": tangent_y,
            "collector_center_base_m": collector_center_base_m,
            "collector_radius_m": collector_radius_m,
            "desired_gap_m": desired_gap_m,
        }

    def build_arc_waypoints(self, optimized: Dict, cfg: Dict) -> List[Tuple[float, float, float, float, float, float]]:
        center = optimized["collector_center_base_m"]
        tangent_x = optimized["tangent_x"]
        tangent_y = optimized["tangent_y"]
        radius = optimized["collector_radius_m"] + optimized["desired_gap_m"]
        side_direction = base_ui.normalize(-tangent_x)
        span_deg = float(cfg["arc_span_deg"])
        steps = max(3, base_ui.safe_int(str(cfg["arc_steps"]), int(cfg["arc_steps"])))
        values = base_ui.np.linspace(math.radians(span_deg * 0.5), math.radians(-span_deg * 0.5), steps)
        waypoints = []
        for value in values:
            direction = base_ui.normalize(math.cos(value) * side_direction + math.sin(value) * tangent_y)
            needle_target = center + direction * radius
            tool_rotation = base_ui.look_at_rotation(direction, self.world_up)
            tool_translation = needle_target - tool_rotation @ self.tool_to_needle[:3, 3]
            tool_tf = base_ui.transform_from_xyz(tool_rotation, tool_translation)
            waypoints.append(base_ui.transform_to_coords_deg(tool_tf))
        return waypoints

    def send_robot_coords(self, current_coords, target_coords, cfg: Dict) -> None:
        if self.robot is None:
            return
        stepped = base_ui.coords_step_towards(
            current_coords,
            target_coords,
            float(cfg["max_command_step_mm"]),
            float(cfg["max_rotation_step_deg"]),
        )
        self.robot.send_coords(list(stepped), int(cfg["robot_speed"]), 0)
        self.last_command_time = time.time()
        self.last_robot_coords = stepped

    def start_localization(self) -> None:
        self.localization_samples = []
        self.last_localize_time = 0.0
        self.optimized_target = None
        self.execution_waypoints = []
        self.execution_index = 0
        self.phase = "localizing"
        self.phase_text.set("localizing")
        self.status_text.set("Collecting localization samples")

    def save_locked_collector(self) -> None:
        if self.locked_collector is None:
            self.status_text.set("No locked collector pose to save")
            return
        base_ui.save_json(self.collector_path, self.locked_collector)
        self.status_text.set(f"Collector pose saved to {self.collector_path}")

    def load_locked_collector(self) -> None:
        if not self.collector_path.exists():
            self.status_text.set("No saved collector pose file found")
            return
        payload = base_ui.load_json(self.collector_path)
        if "collector_center_base_m" not in payload or "collector_radius_mm" not in payload:
            self.status_text.set("Collector pose file is invalid")
            return
        self.locked_collector = payload
        self.optimized_target = None
        self.execution_waypoints = []
        self.execution_index = 0
        center_mm = payload.get("collector_center_base_mm")
        if center_mm is None:
            center_mm = [float(v) * 1000.0 for v in payload["collector_center_base_m"]]
            payload["collector_center_base_mm"] = center_mm
        self.collector_text.set(", ".join(f"{float(v):.1f}" for v in center_mm))
        self.target_text.set("-")
        self.phase = "collector_locked"
        self.phase_text.set("collector_locked")
        self.status_text.set(f"Collector pose loaded from {self.collector_path}")

    def optimize_locked_target(self) -> None:
        cfg = self.get_runtime_cfg()
        if self.locked_collector is None:
            self.status_text.set("Localize and lock the collector first")
            return
        current_coords = self.get_robot_coords(cfg)
        collector_center_base_m = base_ui.np.array(self.locked_collector["collector_center_base_m"], dtype=float)
        collector_radius_m = float(self.locked_collector["collector_radius_mm"]) / 1000.0
        optimized = self.optimize_from_locked_pose(current_coords, collector_center_base_m, collector_radius_m, cfg)
        if optimized is None:
            self.phase = "collector_locked"
            self.phase_text.set("collector_locked")
            self.status_text.set("No valid target pose found in workspace")
            return
        self.optimized_target = {
            "timestamp": time.time(),
            "collector_center_base_m": [float(v) for v in collector_center_base_m],
            "collector_radius_mm": float(self.locked_collector["collector_radius_mm"]),
            "target_distance_mm": float(cfg["target_distance_mm"]),
            "best_score": float(optimized["best_score"]),
            "target_coords_mm_deg": [float(v) for v in optimized["best_coords"]],
            "arc_waypoints": [[float(v) for v in pose] for pose in self.build_arc_waypoints(optimized, cfg)],
        }
        self.phase = "target_optimized"
        self.phase_text.set("target_optimized")
        self.target_text.set(", ".join(f"{v:.1f}" for v in self.optimized_target["target_coords_mm_deg"]))
        self.status_text.set("Target optimized and locked")

    def save_optimized_target(self) -> None:
        if self.optimized_target is None:
            self.status_text.set("No optimized target to save")
            return
        base_ui.save_json(self.target_path, self.optimized_target)
        self.status_text.set(f"Target pose saved to {self.target_path}")

    def load_optimized_target(self) -> None:
        if not self.target_path.exists():
            self.status_text.set("No saved target pose file found")
            return
        payload = base_ui.load_json(self.target_path)
        if "target_coords_mm_deg" not in payload or "arc_waypoints" not in payload:
            self.status_text.set("Target pose file is invalid")
            return
        self.optimized_target = payload
        if self.locked_collector is None and "collector_center_base_m" in payload and "collector_radius_mm" in payload:
            collector_center_base_m = [float(v) for v in payload["collector_center_base_m"]]
            self.locked_collector = {
                "timestamp": payload.get("timestamp", time.time()),
                "sample_count": 0,
                "collector_center_base_m": collector_center_base_m,
                "collector_center_base_mm": [round(v * 1000.0, 3) for v in collector_center_base_m],
                "collector_radius_mm": float(payload["collector_radius_mm"]),
                "sphere_radius_px_median": 0.0,
            }
        self.execution_waypoints = []
        self.execution_index = 0
        self.target_text.set(", ".join(f"{float(v):.1f}" for v in payload["target_coords_mm_deg"]))
        if self.locked_collector is not None:
            self.collector_text.set(", ".join(f"{float(v):.1f}" for v in self.locked_collector["collector_center_base_mm"]))
        self.phase = "target_optimized"
        self.phase_text.set("target_optimized")
        self.status_text.set(f"Target pose loaded from {self.target_path}")

    def execute_approach(self) -> None:
        cfg = self.get_runtime_cfg()
        if self.optimized_target is None:
            self.status_text.set("Optimize the target first")
            return
        final_target = tuple(float(v) for v in self.optimized_target["target_coords_mm_deg"])
        collector_center = base_ui.np.array(self.optimized_target["collector_center_base_m"], dtype=float)
        radius_m = float(self.optimized_target["collector_radius_mm"]) / 1000.0
        gap_m = float(self.optimized_target["target_distance_mm"]) / 1000.0
        best_tool_tf = base_ui.coords_deg_to_transform(final_target)
        needle_tip = best_tool_tf @ self.tool_to_needle
        direction = base_ui.normalize(needle_tip[:3, 3] - collector_center)
        pre_needle = collector_center + direction * (radius_m + gap_m + float(cfg["pre_approach_extra_mm"]) / 1000.0)
        pre_translation = pre_needle - best_tool_tf[:3, :3] @ self.tool_to_needle[:3, 3]
        pre_tf = base_ui.transform_from_xyz(best_tool_tf[:3, :3], pre_translation)
        pre_target = base_ui.transform_to_coords_deg(pre_tf)
        self.execution_waypoints = [pre_target, final_target]
        self.execution_index = 0
        self.phase = "executing_approach"
        self.phase_text.set("executing_approach")
        self.status_text.set("Executing deterministic approach")

    def execute_arc(self) -> None:
        if self.optimized_target is None:
            self.status_text.set("Optimize the target first")
            return
        self.execution_waypoints = [tuple(float(v) for v in pose) for pose in self.optimized_target["arc_waypoints"]]
        self.execution_index = 0
        self.phase = "executing_arc"
        self.phase_text.set("executing_arc")
        self.status_text.set("Executing deterministic arc")

    def stop_all(self) -> None:
        self.execution_waypoints = []
        self.execution_index = 0
        if self.optimized_target is not None:
            self.phase = "target_optimized"
            self.phase_text.set("target_optimized")
        elif self.locked_collector is not None:
            self.phase = "collector_locked"
            self.phase_text.set("collector_locked")
        else:
            self.phase = "idle"
            self.phase_text.set("idle")
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
                "localize_sample_count": cfg["localize_sample_count"],
                "sphere_head_only": cfg["sphere_head_only"],
            }
        )
        base_ui.save_json(self.state_path, self.state)
        self.status_text.set(f"Settings saved to {self.state_path}")

    def maybe_execute_waypoint(self, current_coords, cfg: Dict) -> None:
        if self.robot is None or not self.execution_waypoints:
            return
        if (time.time() - self.last_command_time) < float(cfg["execute_interval_s"]):
            return
        target = self.execution_waypoints[min(self.execution_index, len(self.execution_waypoints) - 1)]
        self.send_robot_coords(current_coords, target, cfg)
        self.execution_index += 1
        if self.execution_index >= len(self.execution_waypoints):
            self.execution_waypoints = []
            if self.phase == "executing_approach":
                self.phase = "target_optimized"
                self.phase_text.set("target_optimized")
                self.status_text.set("Approach completed")
            elif self.phase == "executing_arc":
                self.phase = "target_optimized"
                self.phase_text.set("target_optimized")
                self.status_text.set("Arc completed")

    def update_loop(self) -> None:
        try:
            self._update_frame()
        except Exception as exc:
            self.status_text.set(f"Error: {exc}")
        self.root.after(30, self.update_loop)

    def _update_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            return
        cfg = self.get_runtime_cfg()
        current_coords = self.get_robot_coords(cfg)
        self.last_robot_coords = current_coords
        self.robot_text.set(", ".join(f"{value:.1f}" for value in current_coords))

        candidate = self.detect_candidate(frame, cfg)
        detection_status = "not detected"
        detection_mode = "head" if bool(cfg.get("sphere_head_only", True)) else "legacy"

        if candidate is not None:
            x, y, radius = candidate
            detection_status = f"{detection_mode} center=({x:.1f},{y:.1f}) radius={radius:.1f}px"
            base_ui.cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            base_ui.cv2.circle(frame, (int(x), int(y)), 4, (0, 0, 255), -1)
            payload = base_ui.estimate_depth_and_offset(x, y, radius, frame.shape[1], frame.shape[0], cfg)
            sphere_camera_xyz = base_ui.np.array(payload["camera_xyz_m"], dtype=float)
            collector_center = self.localize_collector_from_candidate(current_coords, sphere_camera_xyz)
            if self.phase == "localizing" and radius >= float(cfg["localize_min_radius_px"]):
                if (time.time() - self.last_localize_time) >= float(cfg["localize_sample_interval_s"]):
                    self.localization_samples.append(
                        {
                            "collector_center_base_m": [float(v) for v in collector_center],
                            "collector_radius_mm": float(cfg["collector_diameter_mm"]) * 0.5,
                            "sphere_center_px": [float(x), float(y)],
                            "sphere_radius_px": float(radius),
                        }
                    )
                    self.last_localize_time = time.time()
                    self.status_text.set(f"Localization sample {len(self.localization_samples)}/{int(cfg['localize_sample_count'])}")
                if len(self.localization_samples) >= int(cfg["localize_sample_count"]):
                    collector_center_m = median_vector([sample["collector_center_base_m"] for sample in self.localization_samples])
                    radius_px = median_vector([[sample["sphere_radius_px"]] for sample in self.localization_samples])[0]
                    self.locked_collector = {
                        "timestamp": time.time(),
                        "sample_count": len(self.localization_samples),
                        "collector_center_base_m": collector_center_m,
                        "collector_center_base_mm": [round(v * 1000.0, 3) for v in collector_center_m],
                        "collector_radius_mm": float(cfg["collector_diameter_mm"]) * 0.5,
                        "sphere_radius_px_median": float(radius_px),
                    }
                    self.collector_text.set(", ".join(f"{v:.1f}" for v in self.locked_collector["collector_center_base_mm"]))
                    self.phase = "collector_locked"
                    self.phase_text.set("collector_locked")
                    self.status_text.set("Collector localized and locked")

        self.detect_text.set(detection_status)

        if self.locked_collector is not None:
            center_mm = self.locked_collector["collector_center_base_mm"]
            self.collector_text.set(", ".join(f"{v:.1f}" for v in center_mm))
            base_ui.cv2.putText(frame, f"locked collector(mm)={center_mm[0]:.1f},{center_mm[1]:.1f},{center_mm[2]:.1f}", (12, 30), base_ui.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 220, 0), 2, base_ui.cv2.LINE_AA)

        if self.optimized_target is not None:
            target_coords = self.optimized_target["target_coords_mm_deg"]
            self.target_text.set(", ".join(f"{v:.1f}" for v in target_coords))
            base_ui.cv2.putText(frame, f"locked target(mm/deg)={target_coords[0]:.1f},{target_coords[1]:.1f},{target_coords[2]:.1f}", (12, 58), base_ui.cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, base_ui.cv2.LINE_AA)

        self.maybe_execute_waypoint(current_coords, cfg)

        base_ui.cv2.putText(frame, f"phase={self.phase} detect={detection_mode}", (12, frame.shape[0] - 18), base_ui.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, base_ui.cv2.LINE_AA)
        display = base_ui.cv2.resize(frame, (960, 540))
        rgb = base_ui.cv2.cvtColor(display, base_ui.cv2.COLOR_BGR2RGB)
        ok, buffer = base_ui.cv2.imencode(".png", rgb)
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
    parser = argparse.ArgumentParser(description="Deterministic target configuration UI")
    parser.add_argument("--config", default=str(DEFAULT_STATE_PATH))
    parser.add_argument("--collector-pose", default=str(DEFAULT_COLLECTOR_POSE_PATH))
    parser.add_argument("--target-pose", default=str(DEFAULT_TARGET_POSE_PATH))
    parser.add_argument("--control-robot", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    app = DeterministicTargetConfApp(
        root,
        Path(args.config),
        Path(args.collector_pose),
        Path(args.target_pose),
        control_robot=args.control_robot,
    )
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
