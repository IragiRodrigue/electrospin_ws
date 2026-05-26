# Python-Only Demo Guide

This guide is the quickest way to test the requested features without ROS 2.

It covers:

1. collector recognition with the process camera
2. human tracking with MediaPipe
3. the presentation game with optional direct robot control through `pymycobot`

## Install Python Dependencies

```bash
python3 -m pip install numpy mediapipe opencv-python opencv-contrib-python
```

Optional, only if you want to command the real robot directly from Python:

```bash
python3 -m pip install pymycobot
```

## 1. Collector Camera Demo

Files:

- [collector_camera_demo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/collector_camera_demo.py)
- [collector_camera_demo_config.example.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/collector_camera_demo_config.example.json)

Run:

```bash
cd ~/pymycobot/electrospin_ws
cp tools/collector_camera_demo_config.example.json tools/collector_camera_demo_config.json
python3 tools/collector_camera_demo.py \
  --config tools/collector_camera_demo_config.json \
  --show-tag-frame
```

What it does:

- opens `/dev/video0`
- detects one ArUco marker
- estimates the marker pose
- computes the collector center pose
- overlays the result on the image

## 2. Human Tracking Demo

Files:

- [human_tracking_demo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/human_tracking_demo.py)
- [human_tracking_common.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/human_tracking_common.py)

Run:

```bash
python3 tools/human_tracking_demo.py \
  --camera-index 0
```

What it does:

- detects upper-body pose
- tracks both hands if visible
- classifies gestures like `open`, `point`, `fist`, `peace`
- shows annotated camera output

Useful options:

```bash
python3 tools/human_tracking_demo.py --camera-index 0 --save-last-json tools/last_human_tracking_state.json
python3 tools/human_tracking_demo.py --camera-index 0 --no-hands
python3 tools/human_tracking_demo.py --camera-index 0 --headless
```

## 3. Presentation Game Demo

File:

- [presentation_game_demo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/presentation_game_demo.py)

### Safe dry-run first

```bash
python3 tools/presentation_game_demo.py \
  --camera-index 0 \
  --tracked-hand right
```

This version:

- uses the camera only
- computes left/right/center from your hand
- computes the target `joint 6` angle
- does not send anything to the robot

### If the image is mirrored

```bash
python3 tools/presentation_game_demo.py \
  --camera-index 0 \
  --tracked-hand right \
  --invert-direction
```

### Real robot mode

Only after the dry-run is stable:

```bash
python3 tools/presentation_game_demo.py \
  --camera-index 0 \
  --tracked-hand right \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

What it does:

- tracks your hand relative to your shoulder
- gates activation through gesture recognition
- maps left/right motion to `joint 6`
- optionally sends `send_angles(...)` to the MyCobot

## Suggested Test Order

### Step 1

Test the collector camera only:

```bash
python3 tools/collector_camera_demo.py --config tools/collector_camera_demo_config.json --show-tag-frame
```

### Step 2

Test human tracking only:

```bash
python3 tools/human_tracking_demo.py --camera-index 0
```

### Step 3

Test the presentation game without the robot:

```bash
python3 tools/presentation_game_demo.py --camera-index 0 --tracked-hand right
```

### Step 4

Test the presentation game with the robot:

```bash
python3 tools/presentation_game_demo.py \
  --camera-index 0 \
  --tracked-hand right \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

## Notes

### Camera sharing

For early testing, run one script at a time.

Do not launch:

- collector camera demo
- human tracking demo
- presentation game demo

on the same `/dev/video0` at the same time.

### Robot safety

For the presentation game:

- do not mount the syringe
- keep the amplitude small
- stand in a clear area
- test dry-run before direct robot control

### If `cv2.aruco` is missing

Use an OpenCV build with contrib modules:

```bash
python3 -m pip install opencv-contrib-python
```

### If `pymycobot` is missing

Install it only when you are ready to test hardware:

```bash
python3 -m pip install pymycobot
```
