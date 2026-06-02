#!/usr/bin/env python3
"""Fairino 6-axis version of the spherical collector target configuration UI."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, Tuple

from fairino_robot_adapter import FairinoRobotAdapter


THIS_DIR = Path(__file__).resolve().parent
PARENT_DIR = THIS_DIR.parent
BASE_TARGET_CONF_PATH = THIS_DIR / "target_conf.py"

spec = importlib.util.spec_from_file_location("mycobot_target_conf_ui", BASE_TARGET_CONF_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load base target configuration UI from {BASE_TARGET_CONF_PATH}")
base_ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base_ui)


FAIRINO_DEFAULT_STATE: Dict = dict(base_ui.DEFAULT_STATE)
FAIRINO_DEFAULT_STATE.update(
    {
        "robot_ip": "192.168.58.2",
        "robot_speed": 20,
        "tool_num": 0,
        "user_num": 0,
        "simulated_robot_coords_mm_deg": [450.0, 0.0, 380.0, 180.0, 0.0, 90.0],
        "workspace_x_min_mm": 150.0,
        "workspace_x_max_mm": 750.0,
        "workspace_y_min_mm": -450.0,
        "workspace_y_max_mm": 450.0,
        "workspace_z_min_mm": 50.0,
        "workspace_z_max_mm": 700.0,
        "max_command_step_mm": 35.0,
        "max_rotation_step_deg": 12.0,
    }
)


DEFAULT_STATE_PATH = PARENT_DIR / "tools" / "fairino_target_conf_state.json"
DEFAULT_POSES_PATH = PARENT_DIR / "tools" / "fairino_target_conf_saved_poses.json"


class FairinoTargetConfApp(base_ui.TargetConfApp):
    def __init__(self, root: tk.Tk, cfg_path: Path, poses_path: Path, control_robot: bool) -> None:
        self.root = root
        self.cfg_path = cfg_path
        self.poses_path = poses_path
        self.control_robot_requested = control_robot
        self.state = self._load_state()
        self.detector = base_ui.SphereDetector()
        self.robot: FairinoRobotAdapter | None = None
        self.cap = base_ui.cv2.VideoCapture(int(self.state["camera_index"]))
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.state['camera_index']}")
        self.cap.set(base_ui.cv2.CAP_PROP_FRAME_WIDTH, int(self.state["frame_width"]))
        self.cap.set(base_ui.cv2.CAP_PROP_FRAME_HEIGHT, int(self.state["frame_height"]))
        self.cap.set(base_ui.cv2.CAP_PROP_FPS, int(self.state["fps"]))

        self.tool_from_camera = base_ui.build_transform_from_cfg("tool_from_camera_position_m", "tool_from_camera_rpy_rad", self.state)
        self.tool_to_needle = base_ui.build_transform_from_cfg("needle_from_tool_position_m", "needle_from_tool_rpy_rad", self.state)
        self.world_up = base_ui.normalize(base_ui.np.array(self.state["preferred_world_up"], dtype=float))
        self.yaw_offsets_deg = base_ui.candidate_offsets(float(self.state["candidate_yaw_max_deg"]), float(self.state["candidate_angle_step_deg"]))
        self.pitch_offsets_deg = base_ui.candidate_offsets(float(self.state["candidate_pitch_max_deg"]), float(self.state["candidate_angle_step_deg"]))

        self.saved_poses = []
        self.last_payload = None
        self.last_robot_coords = None
        self.last_target_coords = None
        self.last_command_time = 0.0
        self.arc_waypoints = []
        self.arc_running = False
        self.arc_index = 0
        self.approach_active = False

        self.photo_image = None
        self.status_text = tk.StringVar(value="Idle")
        self.distance_text = tk.StringVar(value="-")
        self.mode_text = tk.StringVar(value="idle")
        self.center_text = tk.StringVar(value="-")
        self.robot_text = tk.StringVar(value="-")
        self.connect_text = tk.StringVar(value="Connect Fairino")

        self.target_distance_var = tk.StringVar(value=f"{float(self.state['target_distance_mm']):.1f}")
        self.collector_diameter_var = tk.StringVar(value=f"{float(self.state['collector_diameter_mm']):.1f}")
        self.speed_var = tk.StringVar(value=f"{int(self.state['robot_speed'])}")
        self.robot_ip_var = tk.StringVar(value=str(self.state["robot_ip"]))
        self.tool_num_var = tk.StringVar(value=str(int(self.state["tool_num"])))
        self.user_num_var = tk.StringVar(value=str(int(self.state["user_num"])))
        self.auto_arc_var = tk.BooleanVar(value=True)
        self.auto_approach_var = tk.BooleanVar(value=False)

        self._build_ui()

        if self.control_robot_requested:
            self.connect_robot()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.update_loop)

    def _load_state(self) -> Dict:
        merged = dict(FAIRINO_DEFAULT_STATE)
        if base_ui.CALIBRATION_PATH.exists():
            merged = base_ui.merge_defaults(merged, base_ui.load_json(base_ui.CALIBRATION_PATH))
        if self.cfg_path.exists():
            merged = base_ui.merge_defaults(merged, base_ui.load_json(self.cfg_path))
        return merged

    def _build_ui(self) -> None:
        self.root.title("Fairino Target Config UI")
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

        control = ttk.LabelFrame(container, text="Fairino / Process Controls")
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
        ttk.Label(control, text="Robot IP").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.robot_ip_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="Tool num").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.tool_num_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
        row += 1
        ttk.Label(control, text="User num").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(control, textvariable=self.user_num_var, width=16).grid(row=row, column=1, sticky="ew", pady=4)
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

    def connect_robot(self) -> None:
        try:
            self.robot = FairinoRobotAdapter(
                self.robot_ip_var.get().strip(),
                speed=base_ui.safe_int(self.speed_var.get(), int(self.state["robot_speed"])),
                tool_num=base_ui.safe_int(self.tool_num_var.get(), int(self.state["tool_num"])),
                user_num=base_ui.safe_int(self.user_num_var.get(), int(self.state["user_num"])),
            )
            time.sleep(0.5)
            self.connect_text.set("Disconnect Fairino")
            self.status_text.set("Fairino robot connected")
        except Exception as exc:  # pragma: no cover - hardware specific
            self.robot = None
            messagebox.showerror("Fairino connection failed", str(exc))

    def get_runtime_cfg(self) -> Dict:
        cfg = dict(self.state)
        cfg["target_distance_mm"] = base_ui.safe_float(self.target_distance_var.get(), float(self.state["target_distance_mm"]))
        cfg["collector_diameter_mm"] = base_ui.safe_float(self.collector_diameter_var.get(), float(self.state["collector_diameter_mm"]))
        cfg["robot_speed"] = base_ui.safe_int(self.speed_var.get(), int(self.state["robot_speed"]))
        cfg["robot_ip"] = self.robot_ip_var.get().strip() or str(self.state["robot_ip"])
        cfg["tool_num"] = base_ui.safe_int(self.tool_num_var.get(), int(self.state["tool_num"]))
        cfg["user_num"] = base_ui.safe_int(self.user_num_var.get(), int(self.state["user_num"]))
        cfg["sphere_diameter_m"] = cfg["collector_diameter_mm"] / 1000.0
        return cfg

    def get_robot_coords(self, cfg: Dict) -> Tuple[float, float, float, float, float, float]:
        if self.robot is not None:
            coords = self.robot.get_coords()
            if coords is not None and len(coords) == 6:
                return tuple(float(v) for v in coords)
        return tuple(float(v) for v in cfg["simulated_robot_coords_mm_deg"])

    def send_robot_coords(
        self,
        current_coords: Tuple[float, float, float, float, float, float],
        target_coords: Tuple[float, float, float, float, float, float],
        cfg: Dict,
    ) -> None:
        if self.robot is None:
            return
        stepped = base_ui.coords_step_towards(
            current_coords,
            target_coords,
            float(cfg["max_command_step_mm"]),
            float(cfg["max_rotation_step_deg"]),
        )
        self.robot.send_coords(stepped, int(cfg["robot_speed"]))
        self.last_command_time = time.time()
        self.last_target_coords = stepped

    def save_settings(self) -> None:
        cfg = self.get_runtime_cfg()
        self.state.update(
            {
                "target_distance_mm": cfg["target_distance_mm"],
                "collector_diameter_mm": cfg["collector_diameter_mm"],
                "robot_ip": cfg["robot_ip"],
                "tool_num": cfg["tool_num"],
                "user_num": cfg["user_num"],
                "robot_speed": cfg["robot_speed"],
            }
        )
        base_ui.save_json(self.cfg_path, self.state)
        self.status_text.set(f"Settings saved to {self.cfg_path}")

    def on_close(self) -> None:
        try:
            self.save_settings()
        except Exception:
            pass
        try:
            if self.robot is not None:
                self.robot.close()
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fairino target configuration UI for the spherical collector")
    parser.add_argument("--config", default=str(DEFAULT_STATE_PATH))
    parser.add_argument("--poses", default=str(DEFAULT_POSES_PATH))
    parser.add_argument("--control-robot", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    app = FairinoTargetConfApp(root, Path(args.config), Path(args.poses), control_robot=args.control_robot)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
