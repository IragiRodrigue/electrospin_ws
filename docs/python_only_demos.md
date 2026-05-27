# Python-Only Demo Guide

This guide is the quickest way to test the requested features without ROS 2.

It covers:

1. collector recognition with the process camera
2. human tracking with MediaPipe
3. the presentation game with optional direct robot control through `pymycobot`
4. body teleoperation of the MyCobot 280 with safety limits
5. markerless collector pose optimization around a spherical target

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

## 4. Body Teleoperation Demo

Files:

- [body_teleop_demo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/body_teleop_demo.py)
- [body_teleop_demo.md](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/docs/body_teleop_demo.md)

### Safe dry-run first

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0
```

What it does:

- maps torso motion to robot joint motion
- uses wrist motion to drive wrist joints
- freezes when your body stays still
- freezes when an open palm is detected
- switches to a compact robot pose when a fist is detected

### Real robot mode

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0 \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

## 5. Markerless Collector Pose Optimizer

Files:

- [markerless_collector_pose_optimizer.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/markerless_collector_pose_optimizer.py)
- [markerless_collector_pose_optimizer.md](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/docs/markerless_collector_pose_optimizer.md)

This version does more than visual following:

- estimates the sphere center in 3D
- keeps a configured gap to the collector
- searches several candidate poses around the sphere
- chooses a better pose before sending the robot

Dry run:

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json
```

Real robot:

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

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

### Step 5

Test body teleoperation without the robot:

```bash
python3 tools/body_teleop_demo.py --camera-index 0
```

### Step 6

Test body teleoperation with the robot:

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0 \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

### Step 7

Test optimized markerless collector approach without the robot:

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json
```

### Step 8

Test optimized markerless collector approach with the robot:

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json \
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
- body teleoperation demo
- markerless collector pose optimizer

on the same `/dev/video0` at the same time.

### Robot safety

For the presentation game:

- do not mount the syringe
- keep the amplitude small
- stand in a clear area
- test dry-run before direct robot control

For body teleoperation:

- press `b` to capture your neutral pose before judging the mapping
- test the dry-run before direct robot control
- keep the syringe removed
- keep open palm available as a freeze gesture
- start with low robot speed

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
