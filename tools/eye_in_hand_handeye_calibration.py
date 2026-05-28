#!/usr/bin/env python3
"""Interactive hand-eye calibration for a camera mounted on the MyCobot tool."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

from eye_in_hand_collector_servo import (
    build_camera_matrix,
    build_detector,
    coords_deg_to_transform,
    load_json,
    marker_area,
    matrix_to_rpy,
    save_json,
)

try:
    from pymycobot.mycobot import MyCobot
    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


HAND_EYE_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverted = np.eye(4, dtype=float)
    inverted[:3, :3] = rotation.T
    inverted[:3, 3] = -(rotation.T @ translation)
    return inverted


def transform_to_payload(transform: np.ndarray) -> Dict:
    roll, pitch, yaw = matrix_to_rpy(transform[:3, :3])
    return {
        "position_m": [round(float(v), 8) for v in transform[:3, 3]],
        "rpy_rad": [round(float(v), 8) for v in (roll, pitch, yaw)],
    }


def sample_to_json(base_to_tool: np.ndarray, target_to_camera: np.ndarray, robot_coords: List[float]) -> Dict:
    return {
        "robot_coords_mm_deg": [round(float(v), 6) for v in robot_coords],
        "base_to_tool": {
            "rotation": [[round(float(v), 10) for v in row] for row in base_to_tool[:3, :3]],
            "translation_m": [round(float(v), 10) for v in base_to_tool[:3, 3]],
        },
        "target_to_camera": {
            "rotation": [[round(float(v), 10) for v in row] for row in target_to_camera[:3, :3]],
            "translation_m": [round(float(v), 10) for v in target_to_camera[:3, 3]],
        },
    }


def payload_to_transform(payload: Dict) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.array(payload["rotation"], dtype=float)
    transform[:3, 3] = np.array(payload["translation_m"], dtype=float)
    return transform


def calibrate(samples: List[Dict], method_name: str) -> np.ndarray:
    if len(samples) < 3:
        raise RuntimeError("Need at least 3 samples for hand-eye calibration. 8-12 is recommended.")

    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []

    for sample in samples:
        base_to_tool = payload_to_transform(sample["base_to_tool"])
        tool_to_base = invert_transform(base_to_tool)
        target_to_camera = payload_to_transform(sample["target_to_camera"])

        r_gripper2base.append(tool_to_base[:3, :3])
        t_gripper2base.append(tool_to_base[:3, 3].reshape(3, 1))
        r_target2cam.append(target_to_camera[:3, :3])
        t_target2cam.append(target_to_camera[:3, 3].reshape(3, 1))

    method = HAND_EYE_METHODS[method_name]
    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=method,
    )
    result = np.eye(4, dtype=float)
    result[:3, :3] = r_cam2gripper
    result[:3, 3] = t_cam2gripper.reshape(3)
    return result


def main():
    parser = argparse.ArgumentParser(description="Eye-in-hand hand-eye calibration for MyCobot")
    parser.add_argument("--config", default="tools/eye_in_hand_collector_servo_config.json")
    parser.add_argument("--samples-json", default="tools/handeye_samples.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--baud-rate", type=int, default=None)
    parser.add_argument("--method", default="tsai", choices=sorted(HAND_EYE_METHODS))
    parser.add_argument("--min-samples", type=int, default=8)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if not PYMCOBOT_AVAILABLE:
        raise RuntimeError("pymycobot is not installed. This calibration requires the real robot.")

    config_path = Path(args.config)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    cfg: Dict = load_json(config_path)

    samples_path = Path(args.samples_json)
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

    print("Hand-eye calibration started.")
    print("Move the robot to many different poses while keeping the marker visible.")
    print("Keys: c=capture sample, k=calibrate, p=save result to config, d=drop last sample, q=quit")
    print(f"Current samples loaded: {len(samples)}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue

            debug_frame = frame.copy()
            camera_matrix = build_camera_matrix(frame.shape[1], frame.shape[0], cfg)
            corners, ids, _ = detector.detectMarkers(frame) if detector is not None else cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            status = f"samples={len(samples)} method={args.method}"
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
                    status = f"samples={len(samples)} tag_mm={[round(float(v), 1) for v in tag_mm]}"
                else:
                    status = f"marker {marker_id} visible but invalid"
            else:
                status = f"samples={len(samples)} marker not detected"

            cv2.putText(debug_frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(debug_frame, "c=capture k=calibrate p=save d=drop q=quit", (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 220, 0), 2, cv2.LINE_AA)

            if calibrated_transform is not None:
                payload = transform_to_payload(calibrated_transform)
                cv2.putText(
                    debug_frame,
                    f"tool_from_camera pos={payload['position_m']}",
                    (12, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    f"tool_from_camera rpy={payload['rpy_rad']}",
                    (12, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            if not args.headless:
                cv2.imshow("eye_in_hand_handeye_calibration", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("c"):
                if latest_detection is None:
                    print("No valid marker detection to capture.")
                    continue
                sample_payload = sample_to_json(
                    latest_detection["base_to_tool"],
                    latest_detection["target_to_camera"],
                    latest_detection["robot_coords"],
                )
                samples.append(sample_payload)
                save_json(samples_path, samples)
                print(f"Captured sample {len(samples)} to {samples_path}")
            if key == ord("d"):
                if samples:
                    samples.pop()
                    save_json(samples_path, samples)
                    print(f"Dropped last sample. Remaining: {len(samples)}")
            if key == ord("k"):
                if len(samples) < args.min_samples:
                    print(f"Need at least {args.min_samples} samples before calibration. Current: {len(samples)}")
                    continue
                calibrated_transform = calibrate(samples, args.method)
                payload = transform_to_payload(calibrated_transform)
                print("Calibrated tool_from_camera:")
                print(json.dumps(payload, indent=2))
            if key == ord("p"):
                if calibrated_transform is None:
                    print("No calibration result yet. Press 'k' first.")
                    continue
                payload = transform_to_payload(calibrated_transform)
                cfg["tool_from_camera_position_m"] = payload["position_m"]
                cfg["tool_from_camera_rpy_rad"] = payload["rpy_rad"]
                save_json(config_path, cfg)
                print(f"Saved calibration into {config_path}")
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
