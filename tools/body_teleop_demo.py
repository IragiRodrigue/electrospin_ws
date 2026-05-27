#!/usr/bin/env python3
"""Standalone body teleoperation demo for MyCobot 280 without ROS 2."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from human_tracking_common import HumanTracker

try:
    from pymycobot.mycobot import MyCobot
    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


JOINT_LIMITS_DEG = [
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-175.0, 175.0),
]


def parse_joint_list(text: str) -> List[float]:
    values = [v.strip() for v in text.split(",") if v.strip()]
    if len(values) != 6:
        raise ValueError("Expected 6 comma-separated joint angles")
    return [float(v) for v in values]


def clamp_angle(angle_deg: float, joint_index: int) -> float:
    lower, upper = JOINT_LIMITS_DEG[joint_index]
    return max(lower, min(upper, angle_deg))


def clamp_angles(angles_deg: List[float]) -> List[float]:
    return [clamp_angle(angle, idx) for idx, angle in enumerate(angles_deg)]


def shoulder_center(state) -> Tuple[float, float]:
    return (
        0.5 * (state.left_shoulder_xy[0] + state.right_shoulder_xy[0]),
        0.5 * (state.left_shoulder_xy[1] + state.right_shoulder_xy[1]),
    )


def shoulder_width(state) -> float:
    return max(0.02, abs(state.right_shoulder_xy[0] - state.left_shoulder_xy[0]))


def wrist_relative(state, hand_name: str) -> Optional[Tuple[float, float]]:
    hand_name = hand_name.lower()
    if hand_name == "left":
        if min(state.left_wrist_visibility, state.left_shoulder_visibility) < 0.2:
            return None
        return (
            state.left_wrist_xy[0] - state.left_shoulder_xy[0],
            state.left_wrist_xy[1] - state.left_shoulder_xy[1],
        )
    if min(state.right_wrist_visibility, state.right_shoulder_visibility) < 0.2:
        return None
    return (
        state.right_wrist_xy[0] - state.right_shoulder_xy[0],
        state.right_wrist_xy[1] - state.right_shoulder_xy[1],
    )


class JointSmoother:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.value: Optional[np.ndarray] = None

    def filter(self, sample: List[float]) -> List[float]:
        sample_array = np.array(sample, dtype=float)
        if self.value is None:
            self.value = sample_array.copy()
        else:
            self.value = self.alpha * sample_array + (1.0 - self.alpha) * self.value
        return [float(v) for v in self.value]

    def reset(self, value: Optional[List[float]] = None):
        self.value = None if value is None else np.array(value, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Standalone body teleoperation demo for MyCobot 280")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index")
    parser.add_argument("--frame-width", type=int, default=1280, help="Capture width")
    parser.add_argument("--frame-height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=20, help="Target FPS")
    parser.add_argument("--base-joints-deg", default="0,20,-20,0,-90,0", help="Neutral robot joint angles")
    parser.add_argument("--fist-joints-deg", default="0,35,-45,30,-60,0", help="Compact fist-like robot pose")
    parser.add_argument("--body-yaw-max-deg", type=float, default=45.0, help="Max base rotation from torso side shift")
    parser.add_argument("--body-bend-max-deg", type=float, default=25.0, help="Max bend contribution from torso vertical motion")
    parser.add_argument("--wrist-roll-max-deg", type=float, default=30.0, help="Max wrist roll from right hand lateral motion")
    parser.add_argument("--wrist-pitch-max-deg", type=float, default=35.0, help="Max wrist pitch from right hand vertical motion")
    parser.add_argument("--left-arm-assist-max-deg", type=float, default=25.0, help="Max left-arm contribution to joint5")
    parser.add_argument("--deadband", type=float, default=0.03, help="Motion deadband in normalized units")
    parser.add_argument("--stillness-threshold", type=float, default=0.02, help="Freeze when body motion energy is below this threshold")
    parser.add_argument("--smoothing-alpha", type=float, default=0.28, help="EMA smoothing alpha")
    parser.add_argument("--min-pose-confidence", type=float, default=0.5, help="Minimum pose confidence")
    parser.add_argument("--control-robot", action="store_true", help="Send commands to the real robot")
    parser.add_argument("--serial-port", default="/dev/ttyTHS1", help="Robot serial port")
    parser.add_argument("--baud-rate", type=int, default=1000000, help="Robot serial baud rate")
    parser.add_argument("--robot-speed", type=int, default=18, help="MyCobot move speed")
    parser.add_argument("--send-interval", type=float, default=0.18, help="Minimum interval between sends")
    parser.add_argument("--send-deadband-deg", type=float, default=1.2, help="Min total joint delta before sending")
    parser.add_argument("--headless", action="store_true", help="No OpenCV preview window")
    parser.add_argument("--save-last-json", default="", help="Optional JSON output path")
    args = parser.parse_args()

    base_joints_deg = clamp_angles(parse_joint_list(args.base_joints_deg))
    fist_joints_deg = clamp_angles(parse_joint_list(args.fist_joints_deg))

    tracker = HumanTracker(
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        enable_pose=True,
        enable_hands=True,
    )

    robot = None
    if args.control_robot:
        if not PYMCOBOT_AVAILABLE:
            raise RuntimeError("pymycobot is not installed. Disable --control-robot or install pymycobot.")
        robot = MyCobot(args.serial_port, args.baud_rate)
        time.sleep(1.0)
        robot.send_angles(base_joints_deg, args.robot_speed)
        time.sleep(2.0)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera_index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    neutral_reference: Optional[Dict[str, object]] = None
    smoother = JointSmoother(args.smoothing_alpha)
    smoother.reset(base_joints_deg)

    last_saved_state = None
    last_send_time = 0.0
    last_report_time = 0.0
    last_sent_angles = list(base_joints_deg)

    print("Body teleoperation demo started.")
    print("Keys: b=capture neutral pose, r=reset neutral, s=save latest JSON, q=quit")
    print("Open palm freezes. Fist triggers a compact robot gesture.")
    print("Because the MyCobot has no fingers, fist is approximated with a compact arm-and-wrist pose.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame.")
                time.sleep(0.1)
                continue

            state, debug_frame = tracker.process(frame, draw_debug=not args.headless)
            confidence_ok = state.person_detected and state.overall_confidence >= args.min_pose_confidence

            current_center = shoulder_center(state)
            current_width = shoulder_width(state)
            right_rel = wrist_relative(state, "right")
            left_rel = wrist_relative(state, "left")

            right_gesture = state.hand_states.get("right").gesture_name if state.hand_states.get("right") else "none"
            left_gesture = state.hand_states.get("left").gesture_name if state.hand_states.get("left") else "none"

            if neutral_reference is None and confidence_ok:
                neutral_reference = {
                    "shoulder_center": current_center,
                    "shoulder_width": current_width,
                    "right_rel": right_rel or (0.18, 0.05),
                    "left_rel": left_rel or (-0.18, 0.05),
                }
                print("Captured initial neutral body pose automatically.")

            target_angles = list(last_sent_angles)
            active = False
            freeze_reason = "waiting_for_person"
            fist_override = right_gesture == "fist" or left_gesture == "fist"
            open_freeze = right_gesture == "open" or left_gesture == "open"
            motion_energy = 0.0

            if confidence_ok and neutral_reference is not None:
                center_x_error = (current_center[0] - neutral_reference["shoulder_center"][0]) / neutral_reference["shoulder_width"]
                center_y_error = (current_center[1] - neutral_reference["shoulder_center"][1]) / neutral_reference["shoulder_width"]
                right_x = 0.0
                right_y = 0.0
                left_y = 0.0

                if right_rel is not None:
                    right_x = (right_rel[0] - neutral_reference["right_rel"][0]) / neutral_reference["shoulder_width"]
                    right_y = (right_rel[1] - neutral_reference["right_rel"][1]) / neutral_reference["shoulder_width"]
                if left_rel is not None:
                    left_y = (left_rel[1] - neutral_reference["left_rel"][1]) / neutral_reference["shoulder_width"]

                motion_energy = max(abs(center_x_error), abs(center_y_error), abs(right_x), abs(right_y), abs(left_y))

                if fist_override:
                    target_angles = list(fist_joints_deg)
                    freeze_reason = "fist_override"
                elif open_freeze:
                    target_angles = list(last_sent_angles)
                    freeze_reason = "open_palm_freeze"
                elif motion_energy < args.stillness_threshold:
                    target_angles = list(last_sent_angles)
                    freeze_reason = "body_still"
                else:
                    active = True
                    freeze_reason = "body_follow"

                    yaw_cmd = 0.0 if abs(center_x_error) < args.deadband else center_x_error
                    bend_cmd = 0.0 if abs(center_y_error) < args.deadband else center_y_error
                    right_x_cmd = 0.0 if abs(right_x) < args.deadband else right_x
                    right_y_cmd = 0.0 if abs(right_y) < args.deadband else right_y
                    left_y_cmd = 0.0 if abs(left_y) < args.deadband else left_y

                    target_angles = list(base_joints_deg)
                    target_angles[0] += yaw_cmd * args.body_yaw_max_deg
                    target_angles[1] += bend_cmd * args.body_bend_max_deg
                    target_angles[2] -= bend_cmd * (args.body_bend_max_deg * 1.25)
                    target_angles[3] += right_y_cmd * args.wrist_pitch_max_deg
                    target_angles[4] += left_y_cmd * args.left_arm_assist_max_deg
                    target_angles[5] += right_x_cmd * args.wrist_roll_max_deg

                target_angles = clamp_angles(target_angles)
            else:
                target_angles = list(last_sent_angles)

            smoothed_angles = clamp_angles(smoother.filter(target_angles))
            joint_delta_sum = sum(abs(a - b) for a, b in zip(smoothed_angles, last_sent_angles))

            if robot is not None and confidence_ok:
                now = time.time()
                if (
                    (active or fist_override or open_freeze)
                    and (now - last_send_time) >= args.send_interval
                    and joint_delta_sum >= args.send_deadband_deg
                ):
                    robot.send_angles(smoothed_angles, args.robot_speed)
                    last_send_time = now
                    last_sent_angles = list(smoothed_angles)

            if robot is None:
                last_sent_angles = list(smoothed_angles)

            last_saved_state = {
                "active": active,
                "freeze_reason": freeze_reason,
                "motion_energy": round(float(motion_energy), 5),
                "right_gesture": right_gesture,
                "left_gesture": left_gesture,
                "neutral_captured": neutral_reference is not None,
                "target_angles_deg": [round(v, 3) for v in target_angles],
                "smoothed_angles_deg": [round(v, 3) for v in smoothed_angles],
                "control_robot": args.control_robot,
                "serial_port": args.serial_port if args.control_robot else "",
            }

            if not args.headless:
                overlay_y = 120
                status_color = (0, 255, 0) if active else (0, 165, 255)
                cv2.putText(
                    debug_frame,
                    f"teleop active={active} reason={freeze_reason}",
                    (12, overlay_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    status_color,
                    2,
                    cv2.LINE_AA,
                )
                overlay_y += 28
                cv2.putText(
                    debug_frame,
                    f"motion_energy={motion_energy:.3f} neutral={'yes' if neutral_reference else 'no'}",
                    (12, overlay_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
                overlay_y += 28
                cv2.putText(
                    debug_frame,
                    f"target={', '.join(f'{v:.1f}' for v in smoothed_angles)}",
                    (12, overlay_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("body_teleop_demo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            now = time.time()
            if now - last_report_time > 0.6:
                print(
                    f"active={active} reason={freeze_reason} motion={motion_energy:.3f} "
                    f"right={right_gesture} left={left_gesture} "
                    f"joints={[round(v, 1) for v in smoothed_angles]}"
                )
                last_report_time = now

            if key == ord("q"):
                break
            if key == ord("b") and confidence_ok:
                neutral_reference = {
                    "shoulder_center": current_center,
                    "shoulder_width": current_width,
                    "right_rel": right_rel or (0.18, 0.05),
                    "left_rel": left_rel or (-0.18, 0.05),
                }
                print("Captured new neutral body pose.")
            if key == ord("r"):
                neutral_reference = None
                print("Neutral body pose reset.")
            if key == ord("s") and args.save_last_json and last_saved_state is not None:
                output_path = Path(args.save_last_json)
                output_path.write_text(json.dumps(last_saved_state, indent=2), encoding="utf-8")
                print(f"Saved state to {output_path}")
    finally:
        if robot is not None:
            try:
                robot.send_angles(base_joints_deg, args.robot_speed)
                time.sleep(0.5)
            except Exception:
                pass
        tracker.close()
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
