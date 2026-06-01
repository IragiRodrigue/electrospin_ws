#!/usr/bin/env python3
"""
Markerless collector tracker for a reflective spherical collector.

This script tracks the spherical collector directly from the image without an ArUco
marker. It estimates:
- image center of the sphere
- apparent radius in pixels
- approximate depth from known real diameter

This is useful to test whether the collector can still be followed when it moves,
but it is less reliable than a fiducial marker for full 3D pose estimation.
"""

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


def load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_json(path: Path, payload: Dict):
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def build_camera_matrix(width: int, height: int, cfg: Dict) -> Tuple[float, float, float, float]:
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

    return fx, fy, cx, cy


def preprocess(frame_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 1.6)
    return gray


def detect_circle_hough(gray: np.ndarray, cfg: Dict) -> Optional[Tuple[float, float, float]]:
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=float(cfg.get("hough_dp", 1.2)),
        minDist=float(cfg.get("hough_min_dist_px", 120.0)),
        param1=float(cfg.get("hough_param1", 120.0)),
        param2=float(cfg.get("hough_param2", 28.0)),
        minRadius=int(cfg.get("hough_min_radius_px", 40)),
        maxRadius=int(cfg.get("hough_max_radius_px", 320)),
    )
    if circles is None or len(circles[0]) == 0:
        return None
    best = circles[0][0]
    return float(best[0]), float(best[1]), float(best[2])


def detect_circle_contour(frame_bgr: np.ndarray, cfg: Dict) -> Optional[Tuple[float, float, float]]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    _, mask = cv2.threshold(value, int(cfg.get("brightness_threshold", 160)), 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1.0
    min_area = float(cfg.get("contour_min_area_px", 3000.0))

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        score = circularity * area
        if radius < float(cfg.get("contour_min_radius_px", 25.0)):
            continue
        if score > best_score:
            best = (float(x), float(y), float(radius))
            best_score = score
    return best


def estimate_depth_and_offset(
    center_x: float,
    center_y: float,
    radius_px: float,
    frame_width: int,
    frame_height: int,
    cfg: Dict,
) -> Dict:
    fx, fy, cx, cy = build_camera_matrix(frame_width, frame_height, cfg)
    sphere_diameter_m = float(cfg.get("sphere_diameter_m", 0.07))
    diameter_px = max(radius_px * 2.0, 1.0)
    z_m = fx * sphere_diameter_m / diameter_px
    x_m = (center_x - cx) * z_m / fx
    y_m = (center_y - cy) * z_m / fy
    return {
        "center_px": [float(center_x), float(center_y)],
        "radius_px": float(radius_px),
        "diameter_px": float(diameter_px),
        "camera_xyz_m": [float(x_m), float(y_m), float(z_m)],
    }


def main():
    parser = argparse.ArgumentParser(description="Markerless spherical collector tracker")
    parser.add_argument("--config", default="tools/markerless_collector_tracker_config.example.json")
    parser.add_argument("--camera-index", type=int, default=None)
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

    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.get('camera_index', 0)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("frame_width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("frame_height", 720)))
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    last_payload = None
    last_report = 0.0
    print("Markerless collector tracker started.")
    print("Keys: q=quit, s=save latest JSON if --save-last-json was provided")

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

            if candidate is not None:
                x, y, radius = candidate
                payload = estimate_depth_and_offset(x, y, radius, frame.shape[1], frame.shape[0], cfg)
                last_payload = payload

                cv2.circle(debug_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(debug_frame, (int(x), int(y)), 4, (0, 0, 255), -1)
                cv2.putText(
                    debug_frame,
                    f"center=({x:.0f},{y:.0f}) r={radius:.1f}px",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    (
                        f"camera xyz = "
                        f"{payload['camera_xyz_m'][0]:+.3f}, "
                        f"{payload['camera_xyz_m'][1]:+.3f}, "
                        f"{payload['camera_xyz_m'][2]:+.3f} m"
                    ),
                    (12, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    debug_frame,
                    "collector sphere not detected",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            now = time.time()
            if last_payload is not None and now - last_report > 0.5:
                xyz = last_payload["camera_xyz_m"]
                print(
                    f"center_px=({last_payload['center_px'][0]:7.1f},{last_payload['center_px'][1]:7.1f}) "
                    f"radius_px={last_payload['radius_px']:6.1f} "
                    f"camera_xyz_m=({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f})"
                )
                last_report = now

            if not args.headless:
                cv2.imshow("markerless_collector_tracker", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
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
