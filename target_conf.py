#!/usr/bin/env python3
"""
Markerless eye-in-hand collector servo for a spherical collector.
Ajuste automatiquement la distance (Z) pour maintenir un écart cible (ex: 18cm).
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
    coords_deg_to_transform,
    load_json,
    save_json,
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

# ==========================================
# CONFIGURATION DE LA DISTANCE CIBLE
# ==========================================
TARGET_DISTANCE_M = 0.10  # 18 cm (Exemple de distance à maintenir)


def build_rotation_from_cfg(cfg: Dict, pos_key: str, rpy_key: str) -> np.ndarray:
    from eye_in_hand_collector_servo import build_transform_from_cfg
    transform = build_transform_from_cfg(pos_key, rpy_key, cfg)
    return transform[:3, :3]


def clamp_vector(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or norm <= 1e-9:
        return vec
    return vec * (max_norm / norm)


def main():
    parser = argparse.ArgumentParser(description="Markerless spherical collector servo for MyCobot")
    parser.add_argument("--config", default="tools/markerless_collector_servo_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--control-robot", action="store_true")
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--speed", type=int, default=15)  # Vitesse prudente pour les tests
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

    camera_from_tool_rotation = build_rotation_from_cfg(cfg, "tool_from_camera_position_m", "tool_from_camera_rpy_rad")
    tool_from_camera_rotation = camera_from_tool_rotation.T
    
    # Configuration des coordonnées cibles de la caméra : 
    # [0.0, 0.0] pour centrer l'image, et TARGET_DISTANCE_M pour la profondeur (Z)
    desired_camera_xyz = np.array([0.0, 0.0, TARGET_DISTANCE_M], dtype=float)
    
    axis_sign = np.array(cfg.get("camera_servo_axis_sign", [1.0, 1.0, 1.0]), dtype=float)
    translation_gain = float(cfg.get("translation_gain", 0.35))
    max_step_m = float(cfg.get("max_step_m", 0.02))  # Pas maximum de déplacement par itération (2 cm max)
    min_interval_s = float(cfg.get("min_command_interval_s", 0.5))
    simulated_coords = cfg.get("simulated_robot_coords_mm_deg", [160.0, 0.0, 180.0, 0.0, -90.0, 0.0])

    auto_follow = False
    last_payload = None
    last_command_time = 0.0

    print(f"Markerless collector servo started. Distance cible réglée sur : {TARGET_DISTANCE_M*100:.1f} cm")
    print("Cles: g=activer/couper le suivi, m=correction unique, s=sauvegarder JSON, q=quitter")

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
            target_coords = None
            apply_now = False

            if candidate is not None:
                x, y, radius = candidate
                payload = estimate_depth_and_offset(x, y, radius, frame.shape[1], frame.shape[0], cfg)
                sphere_camera_xyz = np.array(payload["camera_xyz_m"], dtype=float)
                
                # --- CALCUL DE L'ERREUR DE DISTANCE (Ex: 32cm - 18cm = 14cm) ---
                # sphere_camera_xyz[2] est la distance Z actuelle mesurée par la caméra
                sphere_error_camera = (sphere_camera_xyz - desired_camera_xyz) * axis_sign
                
                # On applique le gain proportionnel pour adoucir le mouvement et on limite le pas max
                sphere_error_camera = clamp_vector(sphere_error_camera * translation_gain, max_step_m)

                cv2.circle(debug_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(debug_frame, (int(x), int(y)), 4, (0, 0, 255), -1)

                if robot is not None:
                    current_coords = robot.get_coords()
                else:
                    current_coords = simulated_coords
                if current_coords is None or len(current_coords) != 6:
                    current_coords = simulated_coords  # Sécurité anti-coupure de liaison

                base_to_tool = coords_deg_to_transform(tuple(float(v) for v in current_coords))
                base_rotation = base_to_tool[:3, :3]
                
                # Conversion du mouvement de l'espace caméra vers l'espace de la base du robot
                tool_delta = tool_from_camera_rotation @ sphere_error_camera
                base_delta = base_rotation @ tool_delta
                
                new_position_mm = base_to_tool[:3, 3] * 1000.0 + base_delta * 1000.0
                target_coords = (
                    float(new_position_mm[0]),
                    float(new_position_mm[1]),
                    float(new_position_mm[2]),
                    float(current_coords[3]),
                    float(current_coords[4]),
                    float(current_coords[5]),
                )

                # CORRECTION : Suppression de la ligne send_coordinates(sphere_camera_xyz) qui faisait crasher le script

                status = (
                    f"Mesure Z={sphere_camera_xyz[2]*100:.1f}cm | Target={TARGET_DISTANCE_M*100:.1f}cm | "
                    f"Correction Z={sphere_error_camera[2]*100:+.1f}cm"
                )
                
                last_payload = {
                    "sphere_center_px": [round(float(x), 4), round(float(y), 4)],
                    "sphere_radius_px": round(float(radius), 4),
                    "sphere_camera_xyz_m": [round(float(v), 6) for v in sphere_camera_xyz],
                    "camera_error_xyz_m": [round(float(v), 6) for v in sphere_error_camera],
                    "target_robot_coords_mm_deg": [round(float(v), 4) for v in target_coords],
                }

                # Affichages à l'écran
                cv2.putText(
                    debug_frame,
                    f"Sphere Z mesuré = {sphere_camera_xyz[2]*100:.1f} cm (Cible: {TARGET_DISTANCE_M*100:.1f} cm)",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    f"Delta requis (Camera) = {sphere_error_camera[0]*100:+.1f}, {sphere_error_camera[1]*100:+.1f}, {sphere_error_camera[2]*100:+.1f} cm",
                    (12, 58),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    f"Suivi Actif={auto_follow} | Target Robot XYZ mm = {target_coords[0]:.1f}, {target_coords[1]:.1f}, {target_coords[2]:.1f}",
                    (12, 86),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                if auto_follow and robot is not None and (time.time() - last_command_time) >= min_interval_s:
                    apply_now = True
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

            # Affichage de la barre d'état inférieure
            cv2.putText(
                debug_frame,
                status,
                (12, frame.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if target_coords is not None else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            if not args.headless:
                cv2.imshow("markerless_collector_servo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("g"):
                auto_follow = not auto_follow
                print(f"Auto-follow basculé sur : {auto_follow}")
            if key == ord("m"):
                apply_now = True
            if key == ord("s") and args.save_last_json and last_payload is not None:
                save_json(Path(args.save_last_json), last_payload)
                print(f"Sauvegarde réussie dans {args.save_last_json}")

            # Envoi de la commande de déplacement au robot
            if apply_now and robot is not None and target_coords is not None:
                robot.send_coords(list(target_coords), args.speed, 0)
                last_command_time = time.time()
                print(f"[ACTION] Déplacement vers : {[round(float(v), 1) for v in target_coords]}")
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