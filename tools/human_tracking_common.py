#!/usr/bin/env python3
"""Shared MediaPipe helpers for standalone Python demos."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import cv2
import mediapipe as mp
import numpy as np


POSE_L_SHOULDER = 11
POSE_R_SHOULDER = 12
POSE_L_ELBOW = 13
POSE_R_ELBOW = 14
POSE_L_WRIST = 15
POSE_R_WRIST = 16

HAND_WRIST = 0
HAND_THUMB_IP = 3
HAND_THUMB_TIP = 4
HAND_INDEX_PIP = 6
HAND_INDEX_TIP = 8
HAND_MIDDLE_PIP = 10
HAND_MIDDLE_TIP = 12
HAND_RING_PIP = 14
HAND_RING_TIP = 16
HAND_PINKY_PIP = 18
HAND_PINKY_TIP = 20

FINGER_TIPS = [HAND_THUMB_TIP, HAND_INDEX_TIP, HAND_MIDDLE_TIP, HAND_RING_TIP, HAND_PINKY_TIP]
FINGER_PIPS = [HAND_THUMB_IP, HAND_INDEX_PIP, HAND_MIDDLE_PIP, HAND_RING_PIP, HAND_PINKY_PIP]


class GestureID(IntEnum):
    NONE = 0
    FIST = 1
    OPEN = 2
    POINT = 3
    THUMBS_UP = 4
    PEACE = 5
    GRAB = 6
    RELEASE = 7


GESTURE_NAMES = {
    GestureID.NONE: "none",
    GestureID.FIST: "fist",
    GestureID.OPEN: "open",
    GestureID.POINT: "point",
    GestureID.THUMBS_UP: "thumbs_up",
    GestureID.PEACE: "peace",
    GestureID.GRAB: "grab",
    GestureID.RELEASE: "release",
}


@dataclass
class HandState:
    handedness: str = "unknown"
    gesture_name: str = "none"
    gesture_confidence: float = 0.0
    wrist_xy: Tuple[float, float] = (0.0, 0.0)
    finger_states: List[int] = field(default_factory=list)


@dataclass
class TrackingState:
    person_detected: bool = False
    overall_confidence: float = 0.0
    left_shoulder_xy: Tuple[float, float] = (0.0, 0.0)
    right_shoulder_xy: Tuple[float, float] = (0.0, 0.0)
    left_wrist_xy: Tuple[float, float] = (0.0, 0.0)
    right_wrist_xy: Tuple[float, float] = (0.0, 0.0)
    left_shoulder_visibility: float = 0.0
    right_shoulder_visibility: float = 0.0
    left_wrist_visibility: float = 0.0
    right_wrist_visibility: float = 0.0
    hand_states: Dict[str, HandState] = field(default_factory=dict)


class GestureClassifier:
    @staticmethod
    def _finger_extended(landmarks, tip_idx: int, pip_idx: int, is_thumb: bool = False) -> bool:
        if is_thumb:
            dx_tip = abs(landmarks[tip_idx].x - landmarks[HAND_WRIST].x)
            dx_ip = abs(landmarks[pip_idx].x - landmarks[HAND_WRIST].x)
            return dx_tip > dx_ip * 1.2
        return landmarks[tip_idx].y < landmarks[pip_idx].y

    @staticmethod
    def classify(landmarks) -> Tuple[GestureID, float, List[int]]:
        if landmarks is None or len(landmarks) < 21:
            return GestureID.NONE, 0.0, [0, 0, 0, 0, 0]

        finger_states = []
        finger_states.append(
            0 if GestureClassifier._finger_extended(landmarks, HAND_THUMB_TIP, HAND_THUMB_IP, is_thumb=True) else 1
        )
        for tip, pip in zip(FINGER_TIPS[1:], FINGER_PIPS[1:]):
            finger_states.append(0 if GestureClassifier._finger_extended(landmarks, tip, pip) else 1)

        extended_count = sum(1 for s in finger_states if s == 0)
        flexed_count = 5 - extended_count

        thumb_tip = landmarks[HAND_THUMB_TIP]
        index_tip = landmarks[HAND_INDEX_TIP]
        pinch_dist = math.sqrt((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2)

        if flexed_count >= 4 and finger_states[0] == 1:
            return GestureID.FIST, 0.9, finger_states
        if extended_count >= 4 and finger_states[0] == 0:
            return GestureID.OPEN, 0.85, finger_states
        if finger_states[1] == 0 and finger_states[2] == 0 and flexed_count >= 2:
            return GestureID.PEACE, 0.85, finger_states
        if finger_states[0] == 0 and finger_states[1] == 0 and flexed_count >= 3:
            return GestureID.THUMBS_UP, 0.8, finger_states
        if finger_states[1] == 0 and flexed_count >= 3 and finger_states[0] == 1:
            return GestureID.POINT, 0.8, finger_states
        if pinch_dist < 0.05 and finger_states[1] == 1:
            return GestureID.GRAB, 0.7, finger_states
        if pinch_dist > 0.08 and finger_states[1] == 0:
            return GestureID.RELEASE, 0.65, finger_states
        return GestureID.NONE, 0.35, finger_states


class HumanTracker:
    def __init__(
        self,
        model_complexity: int = 1,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        enable_pose: bool = True,
        enable_hands: bool = True,
    ):
        self.enable_pose = enable_pose
        self.enable_hands = enable_hands
        self._mp_pose = mp.solutions.pose
        self._mp_hands = mp.solutions.hands
        self._draw = mp.solutions.drawing_utils
        self._pose_styles = mp.solutions.drawing_styles
        self._gesture_classifier = GestureClassifier()

        self._pose_processor = None
        self._hands_processor = None
        if self.enable_pose:
            self._pose_processor = self._mp_pose.Pose(
                static_image_mode=False,
                model_complexity=model_complexity,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
        if self.enable_hands:
            self._hands_processor = self._mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                model_complexity=1,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )

    def process(self, frame_bgr: np.ndarray, draw_debug: bool = True) -> Tuple[TrackingState, np.ndarray]:
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False
        pose_result = self._pose_processor.process(frame_rgb) if self._pose_processor else None
        hands_result = self._hands_processor.process(frame_rgb) if self._hands_processor else None
        frame_rgb.flags.writeable = True

        debug_frame = frame_bgr.copy()
        state = TrackingState()

        if pose_result and pose_result.pose_landmarks:
            landmarks = pose_result.pose_landmarks.landmark
            state.person_detected = True
            visibilities = [
                landmarks[POSE_L_SHOULDER].visibility,
                landmarks[POSE_R_SHOULDER].visibility,
                landmarks[POSE_L_WRIST].visibility,
                landmarks[POSE_R_WRIST].visibility,
            ]
            state.overall_confidence = float(sum(visibilities) / len(visibilities))
            state.left_shoulder_xy = (float(landmarks[POSE_L_SHOULDER].x), float(landmarks[POSE_L_SHOULDER].y))
            state.right_shoulder_xy = (float(landmarks[POSE_R_SHOULDER].x), float(landmarks[POSE_R_SHOULDER].y))
            state.left_wrist_xy = (float(landmarks[POSE_L_WRIST].x), float(landmarks[POSE_L_WRIST].y))
            state.right_wrist_xy = (float(landmarks[POSE_R_WRIST].x), float(landmarks[POSE_R_WRIST].y))
            state.left_shoulder_visibility = float(landmarks[POSE_L_SHOULDER].visibility)
            state.right_shoulder_visibility = float(landmarks[POSE_R_SHOULDER].visibility)
            state.left_wrist_visibility = float(landmarks[POSE_L_WRIST].visibility)
            state.right_wrist_visibility = float(landmarks[POSE_R_WRIST].visibility)

            if draw_debug:
                self._draw.draw_landmarks(
                    debug_frame,
                    pose_result.pose_landmarks,
                    self._mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self._pose_styles.get_default_pose_landmarks_style(),
                )

        if hands_result and hands_result.multi_hand_landmarks and hands_result.multi_handedness:
            for hand_landmarks, handedness in zip(hands_result.multi_hand_landmarks, hands_result.multi_handedness):
                label = handedness.classification[0].label.lower()
                gesture_id, gesture_conf, finger_states = self._gesture_classifier.classify(hand_landmarks.landmark)
                wrist = hand_landmarks.landmark[HAND_WRIST]
                state.hand_states[label] = HandState(
                    handedness=label,
                    gesture_name=GESTURE_NAMES[gesture_id],
                    gesture_confidence=float(gesture_conf),
                    wrist_xy=(float(wrist.x), float(wrist.y)),
                    finger_states=finger_states,
                )
                if draw_debug:
                    self._draw.draw_landmarks(
                        debug_frame,
                        hand_landmarks,
                        self._mp_hands.HAND_CONNECTIONS,
                    )

        if draw_debug:
            self._draw_overlay(debug_frame, state)

        return state, debug_frame

    @staticmethod
    def hand_offset_x(state: TrackingState, tracked_hand: str) -> Optional[float]:
        tracked_hand = tracked_hand.lower()
        if tracked_hand == "left":
            if min(state.left_wrist_visibility, state.left_shoulder_visibility) <= 0.0:
                return None
            return state.left_wrist_xy[0] - state.left_shoulder_xy[0]
        if min(state.right_wrist_visibility, state.right_shoulder_visibility) <= 0.0:
            return None
        return state.right_wrist_xy[0] - state.right_shoulder_xy[0]

    def close(self):
        if self._pose_processor:
            self._pose_processor.close()
        if self._hands_processor:
            self._hands_processor.close()

    @staticmethod
    def _draw_overlay(frame: np.ndarray, state: TrackingState):
        y = 30
        cv2.putText(
            frame,
            f"person={state.person_detected} conf={state.overall_confidence:.2f}",
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if state.person_detected else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28
        for hand_name in ("left", "right"):
            hand_state = state.hand_states.get(hand_name)
            if hand_state is None:
                text = f"{hand_name}: no hand"
                color = (80, 80, 255)
            else:
                text = f"{hand_name}: {hand_state.gesture_name} ({hand_state.gesture_confidence:.2f})"
                color = (255, 220, 0)
            cv2.putText(
                frame,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )
            y += 28
