#!/usr/bin/env python3
"""Standalone presentation game demo without ROS 2."""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

import cv2

from human_tracking_common import HumanTracker

try:
    from pymycobot.mycobot import MyCobot
    PYMCOBOT_AVAILABLE = True
except ImportError:
    PYMCOBOT_AVAILABLE = False


def parse_joint_list(text: str) -> List[float]:
    values = [v.strip() for v in text.split(",") if v.strip()]
    if len(values) != 6:
        raise ValueError("Expected 6 comma-separated joint angles")
    return [float(v) for v in values]


def gesture_allows_tracking(activation_gesture: str, gesture_name: Optional[str]) -> bool:
    if activation_gesture in {"always", "none"}:
        return True
    if gesture_name is None:
        return False
    if activation_gesture == "point":
        return gesture_name == "point"
    if activation_gesture == "open":
        return gesture_name == "open"
    if activation_gesture == "point_or_open":
        return gesture_name in {"point", "open"}
    return False


def main():
    parser = argparse.ArgumentParser(description="Standalone presentation game demo")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index")
    parser.add_argument("--frame-width", type=int, default=1280, help="Capture width")
    parser.add_argument("--frame-height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=20, help="Target FPS")
    parser.add_argument("--tracked-hand", default="right", choices=["left", "right"], help="Tracked hand")
    parser.add_argument("--invert-direction", action="store_true", help="Invert left/right mapping")
    parser.add_argument("--activation-gesture", default="point_or_open", choices=["always", "none", "point", "open", "point_or_open"], help="Gesture gate")
    parser.add_argument("--deadband", type=float, default=0.05, help="Deadband on wrist offset in normalized image units")
    parser.add_argument("--max-lateral-offset", type=float, default=0.07, help="Maximum normalized wrist offset")
    parser.add_argument("--joint6-max-deg", type=float, default=35.0, help="Maximum joint6 deviation")
    parser.add_argument("--base-joints-deg", default="0,20,-20,0,-90,0", help="Base joint angles in degrees")
    parser.add_argument("--direction-smoothing", type=float, default=0.35, help="EMA smoothing for left/right command")
    parser.add_argument("--min-pose-confidence", type=float, default=0.45, help="Minimum pose confidence")
    parser.add_argument("--control-robot", action="store_true", help="Send commands to a real MyCobot")
    parser.add_argument("--serial-port", default="/dev/ttyTHS1", help="Robot serial port")
    parser.add_argument("--baud-rate", type=int, default=1000000, help="Robot serial baud rate")
    parser.add_argument("--robot-speed", type=int, default=20, help="MyCobot move speed")
    parser.add_argument("--send-interval", type=float, default=0.15, help="Minimum interval between robot sends")
    parser.add_argument("--headless", action="store_true", help="No OpenCV preview window")
    parser.add_argument("--save-last-json", default="", help="Optional JSON output path")
    args = parser.parse_args()

    base_joints_deg = parse_joint_list(args.base_joints_deg)
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

    filtered_direction = 0.0
    last_send_time = 0.0
    last_saved_state = None
    last_report_time = 0.0
    last_sent_joint6 = base_joints_deg[5]

    print("Presentation game demo started.")
    print("Keys: q=quit, s=save latest JSON if --save-last-json was given")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame.")
                time.sleep(0.1)
                continue

            state, debug_frame = tracker.process(frame, draw_debug=not args.headless)
            hand_state = state.hand_states.get(args.tracked_hand)
            gesture_name = hand_state.gesture_name if hand_state is not None else None
            tracking_enabled = (
                state.person_detected
                and state.overall_confidence >= args.min_pose_confidence
                and gesture_allows_tracking(args.activation_gesture, gesture_name)
            )

            raw_offset = HumanTracker.hand_offset_x(state, args.tracked_hand)
            if raw_offset is None:
                raw_offset = 0.0
                tracking_enabled = False

            if args.invert_direction:
                raw_offset = -raw_offset

            bounded = max(-args.max_lateral_offset, min(args.max_lateral_offset, raw_offset))
            if abs(bounded) < args.deadband:
                bounded = 0.0

            filtered_direction = (
                args.direction_smoothing * bounded
                + (1.0 - args.direction_smoothing) * filtered_direction
            )
            normalized = 0.0
            if args.max_lateral_offset > 1e-6:
                normalized = max(-1.0, min(1.0, filtered_direction / args.max_lateral_offset))

            side = "center"
            if filtered_direction > args.deadband * 0.5:
                side = "right"
            elif filtered_direction < -args.deadband * 0.5:
                side = "left"

            target_joint6_deg = base_joints_deg[5]
            if tracking_enabled:
                target_joint6_deg += normalized * args.joint6_max_deg

            joint_targets = list(base_joints_deg)
            joint_targets[5] = target_joint6_deg

            if robot is not None and tracking_enabled:
                now = time.time()
                if now - last_send_time >= args.send_interval and abs(target_joint6_deg - last_sent_joint6) >= 1.0:
                    robot.send_angles(joint_targets, args.robot_speed)
                    last_send_time = now
                    last_sent_joint6 = target_joint6_deg

            last_saved_state = {
                "active": tracking_enabled,
                "tracked_hand": args.tracked_hand,
                "gesture_name": gesture_name or "none",
                "side": side,
                "raw_offset": round(raw_offset, 4),
                "filtered_offset": round(filtered_direction, 4),
                "joint_targets_deg": [round(v, 3) for v in joint_targets],
                "control_robot": args.control_robot,
                "serial_port": args.serial_port if args.control_robot else "",
            }

            if not args.headless:
                cv2.putText(
                    debug_frame,
                    f"game active={tracking_enabled} side={side} gesture={gesture_name or 'none'}",
                    (12, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0) if tracking_enabled else (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug_frame,
                    f"joint6={target_joint6_deg:.1f} deg",
                    (12, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("presentation_game_demo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            now = time.time()
            if now - last_report_time > 0.5:
                print(
                    f"active={tracking_enabled} side={side} gesture={gesture_name or 'none'} "
                    f"joint6={target_joint6_deg:6.1f} control_robot={args.control_robot}"
                )
                last_report_time = now

            if key == ord("q"):
                break
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
