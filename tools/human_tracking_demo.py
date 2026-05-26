#!/usr/bin/env python3
"""Standalone human tracking demo without ROS 2."""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from human_tracking_common import HumanTracker


def main():
    parser = argparse.ArgumentParser(description="Standalone human tracking demo")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index")
    parser.add_argument("--frame-width", type=int, default=1280, help="Capture width")
    parser.add_argument("--frame-height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=30, help="Target camera FPS")
    parser.add_argument("--model-complexity", type=int, default=1, help="MediaPipe pose model complexity")
    parser.add_argument("--min-detection-confidence", type=float, default=0.5, help="Min detection confidence")
    parser.add_argument("--min-tracking-confidence", type=float, default=0.5, help="Min tracking confidence")
    parser.add_argument("--no-pose", action="store_true", help="Disable pose tracking")
    parser.add_argument("--no-hands", action="store_true", help="Disable hand tracking")
    parser.add_argument("--headless", action="store_true", help="No OpenCV preview window")
    parser.add_argument("--save-last-json", default="", help="Optional JSON output path for latest state")
    args = parser.parse_args()

    tracker = HumanTracker(
        model_complexity=args.model_complexity,
        min_detection_confidence=args.min_detection_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
        enable_pose=not args.no_pose,
        enable_hands=not args.no_hands,
    )

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera_index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    last_state = None
    last_report_time = 0.0
    print("Human tracking demo started.")
    print("Keys: q=quit, s=save latest JSON if --save-last-json was given")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame.")
                time.sleep(0.1)
                continue

            state, debug_frame = tracker.process(frame, draw_debug=not args.headless)
            last_state = {
                "person_detected": state.person_detected,
                "overall_confidence": round(state.overall_confidence, 4),
                "left_shoulder_xy": [round(v, 4) for v in state.left_shoulder_xy],
                "right_shoulder_xy": [round(v, 4) for v in state.right_shoulder_xy],
                "left_wrist_xy": [round(v, 4) for v in state.left_wrist_xy],
                "right_wrist_xy": [round(v, 4) for v in state.right_wrist_xy],
                "hand_states": {
                    name: {
                        "gesture_name": hand_state.gesture_name,
                        "gesture_confidence": round(hand_state.gesture_confidence, 4),
                        "wrist_xy": [round(v, 4) for v in hand_state.wrist_xy],
                    }
                    for name, hand_state in state.hand_states.items()
                },
            }

            now = time.time()
            if now - last_report_time > 0.5:
                gestures = ", ".join(
                    f"{name}:{hand_state.gesture_name}"
                    for name, hand_state in state.hand_states.items()
                ) or "no_hands"
                print(
                    f"person={state.person_detected} conf={state.overall_confidence:.2f} | "
                    f"gestures={gestures}"
                )
                last_report_time = now

            if not args.headless:
                cv2.imshow("human_tracking_demo", debug_frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = -1

            if key == ord("q"):
                break
            if key == ord("s") and args.save_last_json and last_state is not None:
                output_path = Path(args.save_last_json)
                output_path.write_text(json.dumps(last_state, indent=2), encoding="utf-8")
                print(f"Saved state to {output_path}")
    finally:
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
