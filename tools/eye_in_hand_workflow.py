#!/usr/bin/env python3
"""End-to-end eye-in-hand workflow for ArUco generation, hand-eye capture, calibration, and deterministic launch."""

from __future__ import annotations

import argparse
import json
import runpy
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np

import eye_in_hand_handeye_calibration as handeye
from eye_in_hand_collector_servo import (
    build_camera_matrix,
    build_detector,
    coords_deg_to_transform,
    load_json,
    marker_area,
    save_json,
)

try:
    from pymycobot.mycobot import MyCobot

    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


THIS_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = THIS_DIR.parent
DEFAULT_CONFIG_PATH = WORKSPACE_ROOT / "tools" / "eye_in_hand_collector_servo_config.json"
DEFAULT_SAMPLES_PATH = WORKSPACE_ROOT / "tools" / "handeye_samples.json"
DEFAULT_LAUNCHER_PATH = WORKSPACE_ROOT / "target_conf_deterministic.py"


def default_marker_png_path(cfg: Dict) -> Path:
    marker_id = int(cfg.get("marker_id", 0))
    marker_length_mm = int(round(float(cfg.get("marker_length_m", 0.23)) * 1000.0))
    return WORKSPACE_ROOT / "tools" / f"aruco_marker_id{marker_id}_{marker_length_mm}mm.png"


def default_marker_html_path(cfg: Dict) -> Path:
    return default_marker_png_path(cfg).with_suffix(".html")


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def generate_marker_image(dictionary_name: str, marker_id: int, pixels: int) -> np.ndarray:
    dictionary, _, _ = build_detector(dictionary_name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        image = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    else:
        image = np.zeros((pixels, pixels), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, pixels, image, 1)
    return image


def write_printable_html(html_path: Path, image_path: Path, marker_length_mm: float) -> None:
    image_name = image_path.name
    html = f"""<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="utf-8" />
  <title>Aruco Marker Print</title>
  <style>
    body {{
      margin: 0;
      padding: 16mm;
      font-family: Arial, sans-serif;
      background: white;
      color: black;
    }}
    .page {{
      text-align: center;
    }}
    img {{
      width: {marker_length_mm:.2f}mm;
      height: {marker_length_mm:.2f}mm;
      image-rendering: pixelated;
      display: block;
      margin: 0 auto 6mm auto;
    }}
    .note {{
      font-size: 12pt;
      line-height: 1.4;
    }}
    .warn {{
      font-weight: bold;
    }}
  </style>
</head>
<body>
  <div class="page">
    <img src="{image_name}" alt="Aruco marker" />
    <div class="note">
      <div class="warn">Imprimer a 100% sans mise a l'echelle.</div>
      <div>Taille reelle du carre: {marker_length_mm:.1f} mm</div>
      <div>Verifier a la regle apres impression.</div>
    </div>
  </div>
</body>
</html>
"""
    ensure_parent(html_path)
    html_path.write_text(html, encoding="utf-8")


def generate_aruco_files(config_path: Path, png_path: Optional[Path], html_path: Optional[Path], pixels: int) -> Dict[str, Path]:
    cfg = load_json(config_path)
    png_output = png_path or default_marker_png_path(cfg)
    html_output = html_path or default_marker_html_path(cfg)
    marker_id = int(cfg["marker_id"])
    dictionary_name = cfg.get("marker_dictionary", "DICT_4X4_50")
    marker_length_mm = float(cfg["marker_length_m"]) * 1000.0

    marker_image = generate_marker_image(dictionary_name, marker_id, pixels)
    ensure_parent(png_output)
    if not cv2.imwrite(str(png_output), marker_image):
        raise RuntimeError(f"Failed to write marker image: {png_output}")
    write_printable_html(html_output, png_output, marker_length_mm)

    return {
        "png": png_output,
        "html": html_output,
    }


def save_calibration_into_config(config_path: Path, cfg: Dict, calibrated_transform: np.ndarray) -> Dict:
    payload = handeye.transform_to_payload(calibrated_transform)
    cfg["tool_from_camera_position_m"] = payload["position_m"]
    cfg["tool_from_camera_rpy_rad"] = payload["rpy_rad"]
    save_json(config_path, cfg)
    return payload


def launch_target_deterministic() -> None:
    previous_argv = sys.argv[:]
    try:
        # Launch the deterministic UI with a clean argv so it does not inherit
        # the workflow subcommand such as `wizard` or `capture-calibration`.
        sys.argv = [str(DEFAULT_LAUNCHER_PATH)]
        runpy.run_path(str(DEFAULT_LAUNCHER_PATH), run_name="__main__")
    finally:
        sys.argv = previous_argv


def run_capture_session(args: argparse.Namespace) -> bool:
    if not PYMCOBOT_AVAILABLE:
        raise RuntimeError("pymycobot is not installed. This workflow needs the real robot.")

    config_path = Path(args.config).resolve()
    samples_path = Path(args.samples_json).resolve()

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    cfg: Dict = load_json(config_path)
    if samples_path.exists():
        samples = load_json(samples_path)
        if not isinstance(samples, list):
            raise RuntimeError(f"Samples file must contain a JSON list: {samples_path}")
    else:
        samples = []

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
    sample_target = int(args.sample_target)

    robot = MyCobot(cfg.get("serial_port", "/dev/ttyTHS1"), int(cfg.get("baud_rate", 1000000)))
    time.sleep(1.0)

    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {cfg.get('camera_index', 0)}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("frame_width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("frame_height", 720)))
    cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))

    latest_detection = None
    calibrated_transform = None
    saved_config = False

    print("Workflow hand-eye started.")
    print("1. Imprime le marqueur ArUco a 100% et place-le a la position du collecteur.")
    print("2. Bouge le robot sur 20-30 poses tres differentes en gardant le marqueur visible.")
    print("3. Capture les poses puis calibre et sauve.")
    print("")
    print("Touches:")
    print("  c = capturer la pose courante")
    print("  d = supprimer la derniere pose")
    print("  k = calibrer")
    print("  p = sauver la calibration dans la config")
    print("  l = lancer target_conf_deterministic")
    print("  q = quitter")
    print(f"Poses deja chargees: {len(samples)} / objectif {sample_target}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            camera_matrix = build_camera_matrix(frame.shape[1], frame.shape[0], cfg)
            corners, ids, _ = (
                detector.detectMarkers(frame)
                if detector is not None
                else cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            )

            status = f"samples={len(samples)}/{sample_target}"
            latest_detection = None

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
                    rotation_target_to_camera, _ = cv2.Rodrigues(rvec)
                    target_to_camera = np.eye(4, dtype=float)
                    target_to_camera[:3, :3] = rotation_target_to_camera
                    target_to_camera[:3, 3] = tvec

                    robot_coords = [float(v) for v in robot.get_coords()]
                    base_to_tool = coords_deg_to_transform(tuple(robot_coords))
                    latest_detection = {
                        "base_to_tool": base_to_tool,
                        "target_to_camera": target_to_camera,
                        "robot_coords": robot_coords,
                    }

                    cv2.aruco.drawDetectedMarkers(debug_frame, selected_corners, np.array([[marker_id]], dtype=np.int32))
                    cv2.drawFrameAxes(debug_frame, camera_matrix, distortion, rvec, tvec, marker_length_m * 0.5)
                    tag_mm = target_to_camera[:3, 3] * 1000.0
                    status = f"samples={len(samples)}/{sample_target} tag_mm={[round(float(v), 1) for v in tag_mm]}"
                else:
                    status = f"samples={len(samples)}/{sample_target} marker visible but invalid"
            else:
                status = f"samples={len(samples)}/{sample_target} marker not detected"

            color = (0, 255, 0) if len(samples) >= sample_target else (0, 220, 255)
            cv2.putText(debug_frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
            cv2.putText(
                debug_frame,
                "c=capture d=drop k=calibrate p=save l=launch q=quit",
                (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (255, 220, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug_frame,
                "Varie fortement: gauche/droite, haut/bas, proche/loin, rotations du poignet.",
                (12, 84),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if calibrated_transform is not None:
                payload = handeye.transform_to_payload(calibrated_transform)
                cv2.putText(
                    debug_frame,
                    f"tool_from_camera pos={payload['position_m']}",
                    (12, 112),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    f"tool_from_camera rpy={payload['rpy_rad']}",
                    (12, 138),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("eye_in_hand_workflow", debug_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord("c"):
                if latest_detection is None:
                    print("Aucune detection valide du marqueur pour capturer.")
                    continue
                sample_payload = handeye.sample_to_json(
                    latest_detection["base_to_tool"],
                    latest_detection["target_to_camera"],
                    latest_detection["robot_coords"],
                )
                samples.append(sample_payload)
                save_json(samples_path, samples)
                print(f"Pose capturee {len(samples)} / {sample_target} -> {samples_path}")
                if len(samples) == sample_target:
                    print("Objectif de poses atteint. Tu peux encore en ajouter un peu, puis appuie sur 'k'.")
            elif key == ord("d"):
                if samples:
                    samples.pop()
                    save_json(samples_path, samples)
                    print(f"Derniere pose supprimee. Restant: {len(samples)}")
            elif key == ord("k"):
                if len(samples) < args.min_samples:
                    print(f"Il faut au moins {args.min_samples} poses. Actuel: {len(samples)}")
                    continue
                calibrated_transform = handeye.calibrate(samples, args.method)
                payload = handeye.transform_to_payload(calibrated_transform)
                print("Calibration calculee:")
                print(json.dumps(payload, indent=2))
                if args.auto_save_config:
                    payload = save_calibration_into_config(config_path, cfg, calibrated_transform)
                    saved_config = True
                    print(f"Calibration sauvee automatiquement dans {config_path}")
                    print(json.dumps(payload, indent=2))
                    if args.launch_after_save:
                        print("Lancement de target_conf_deterministic...")
                        break
            elif key == ord("p"):
                if calibrated_transform is None:
                    print("Aucune calibration disponible. Appuie d'abord sur 'k'.")
                    continue
                payload = save_calibration_into_config(config_path, cfg, calibrated_transform)
                saved_config = True
                print(f"Calibration sauvee dans {config_path}")
                print(json.dumps(payload, indent=2))
            elif key == ord("l"):
                if not saved_config and calibrated_transform is not None:
                    payload = save_calibration_into_config(config_path, cfg, calibrated_transform)
                    saved_config = True
                    print(f"Calibration sauvee dans {config_path}")
                    print(json.dumps(payload, indent=2))
                if not saved_config:
                    print("Sauve d'abord une calibration valide avant de lancer la suite.")
                    continue
                print("Lancement de target_conf_deterministic...")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        # Give V4L a short moment to release the camera before reopening it.
        time.sleep(0.5)

    if saved_config and (args.launch_after_save or key == ord("l")):
        launch_target_deterministic()
    return saved_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate ArUco, capture hand-eye samples, calibrate, and launch deterministic targeting.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate_parser = subparsers.add_parser("generate-aruco", help="Generate a printable ArUco marker PNG and HTML page.")
    generate_parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    generate_parser.add_argument("--output-png", default="")
    generate_parser.add_argument("--output-html", default="")
    generate_parser.add_argument("--pixels", type=int, default=1200)

    capture_parser = subparsers.add_parser("capture-calibration", help="Capture robot + ArUco poses and compute tool_from_camera.")
    capture_parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    capture_parser.add_argument("--samples-json", default=str(DEFAULT_SAMPLES_PATH))
    capture_parser.add_argument("--camera-index", type=int, default=None)
    capture_parser.add_argument("--serial-port", default=None)
    capture_parser.add_argument("--baud-rate", type=int, default=None)
    capture_parser.add_argument("--method", default="tsai", choices=sorted(handeye.HAND_EYE_METHODS))
    capture_parser.add_argument("--min-samples", type=int, default=8)
    capture_parser.add_argument("--sample-target", type=int, default=20)
    capture_parser.add_argument("--auto-save-config", action="store_true")
    capture_parser.add_argument("--launch-after-save", action="store_true")

    wizard_parser = subparsers.add_parser("wizard", help="Generate the marker, then enter the capture/calibration workflow.")
    wizard_parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    wizard_parser.add_argument("--samples-json", default=str(DEFAULT_SAMPLES_PATH))
    wizard_parser.add_argument("--output-png", default="")
    wizard_parser.add_argument("--output-html", default="")
    wizard_parser.add_argument("--pixels", type=int, default=1200)
    wizard_parser.add_argument("--camera-index", type=int, default=None)
    wizard_parser.add_argument("--serial-port", default=None)
    wizard_parser.add_argument("--baud-rate", type=int, default=None)
    wizard_parser.add_argument("--method", default="tsai", choices=sorted(handeye.HAND_EYE_METHODS))
    wizard_parser.add_argument("--min-samples", type=int, default=8)
    wizard_parser.add_argument("--sample-target", type=int, default=20)
    wizard_parser.add_argument("--auto-save-config", action="store_true", default=True)
    wizard_parser.add_argument("--launch-after-save", action="store_true")

    launch_parser = subparsers.add_parser("launch-deterministic", help="Launch target_conf_deterministic.py")
    launch_parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    _ = launch_parser

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "generate-aruco":
        config_path = Path(args.config).resolve()
        outputs = generate_aruco_files(
            config_path,
            Path(args.output_png).resolve() if args.output_png else None,
            Path(args.output_html).resolve() if args.output_html else None,
            int(args.pixels),
        )
        print(f"PNG marker: {outputs['png']}")
        print(f"Printable HTML: {outputs['html']}")
        print("Imprime la page HTML a 100% et verifie la taille a la regle.")
        return

    if args.command == "capture-calibration":
        run_capture_session(args)
        return

    if args.command == "wizard":
        config_path = Path(args.config).resolve()
        outputs = generate_aruco_files(
            config_path,
            Path(args.output_png).resolve() if args.output_png else None,
            Path(args.output_html).resolve() if args.output_html else None,
            int(args.pixels),
        )
        print("")
        print("=== Marqueur genere ===")
        print(f"PNG : {outputs['png']}")
        print(f"HTML: {outputs['html']}")
        print("Imprime le HTML a 100%, fixe le marqueur au niveau du collecteur, puis ferme cette ligne et commence la capture.")
        print("")
        run_capture_session(args)
        return

    if args.command == "launch-deterministic":
        launch_target_deterministic()
        return

    raise RuntimeError(f"Unknown command: {args.command}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
