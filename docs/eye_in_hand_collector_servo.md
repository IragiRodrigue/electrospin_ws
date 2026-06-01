# Eye-In-Hand Collector Servo Guide

This guide is for the case where:

- the camera is mounted on joint 6
- the collector is a sphere
- a marker is attached above or near the sphere
- you want Python only, without ROS 2

## Important First Observation From Your Photos

What the robot needs is a real fiducial marker, not only a square outline.

For reliable pose estimation:

- use a real printed ArUco marker with black and white cells
- keep the full marker visible
- do not let the sphere hide the marker center
- keep the marker flat and rigid

If the sphere blocks the marker, OpenCV cannot estimate the pose correctly.

## Files

- [eye_in_hand_collector_servo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/eye_in_hand_collector_servo.py)
- [eye_in_hand_collector_servo_config.example.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/eye_in_hand_collector_servo_config.example.json)
- [eye_in_hand_calibrator.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/eye_in_hand_calibrator.py)

## What This Script Does

The script:

1. detects the marker from the camera mounted on joint 6
2. reads the current robot pose with `get_coords()`
3. computes the marker pose in the robot base frame
4. computes the collector center in the robot base frame
5. computes the desired needle target pose
6. computes the desired robot tool pose
7. can send that pose with `send_coords(...)`

So this is the missing step you were asking for:

- not only detect the collector
- but also convert that detection into robot coordinates and a target pose

## Install

```bash
python3 -m pip install numpy opencv-contrib-python pymycobot
```

## Copy The Config

```bash
cd ~/pymycobot/electrospin_ws
cp tools/eye_in_hand_collector_servo_config.example.json tools/eye_in_hand_collector_servo_config.json
```

## Three Critical Geometric Offsets

### 1. `tool_from_camera_*`

Where is the camera relative to the robot tool frame.

Because your camera is fixed on joint 6, this is mandatory.

### 2. `collector_from_tag_*`

Where is the center of the sphere relative to the marker.

If your marker is above the sphere, the vertical offset is especially important.

### 3. `needle_target_from_collector_*`

Where should the needle tip be relative to the collector center.

This is where you define the desired working distance.

For example:

- sphere center known
- desired approach direction known
- desired gap `150 mm`

## About Your `23 cm` Marker

If the printed square is truly `23 cm`, then:

```json
"marker_length_m": 0.23
```

That value is already the default in the example config.

## Suggested Physical Placement

Best practice:

- place the marker beside or above the sphere
- keep the full marker visible from the camera
- keep the marker rigid
- avoid glare

Less good:

- marker directly behind the sphere
- marker partly hidden by the sphere
- hand-drawn border only

## Detect-Only Test

Start without moving the robot:

```bash
python3 tools/eye_in_hand_collector_servo.py \
  --config tools/eye_in_hand_collector_servo_config.json
```

You should see:

- detected marker axes
- collector center coordinates
- computed target robot coordinates

## Interactive Calibration

Before commanding the robot, tune the offsets live:

```bash
python3 tools/eye_in_hand_calibrator.py \
  --config tools/eye_in_hand_collector_servo_config.json
```

Useful keys:

- `c`: edit `collector_from_tag_position_m`
- `n`: edit `needle_target_from_collector_position_m`
- `t`: edit `tool_from_camera_position_m`
- `a/d`: x minus / plus
- `w/s`: y plus / minus
- `r/f`: z plus / minus
- `[` and `]`: decrease / increase step size
- `p`: save config
- `q`: quit

This is the fastest way to make the computed collector center and target pose match your real setup.

## Save One Measurement

```bash
python3 tools/eye_in_hand_collector_servo.py \
  --config tools/eye_in_hand_collector_servo_config.json \
  --save-last-json tools/last_eye_in_hand_result.json
```

Press `s` to save.

## Real Robot Mode

Only after detect-only is stable:

```bash
python3 tools/eye_in_hand_collector_servo.py \
  --config tools/eye_in_hand_collector_servo_config.json \
  --control-robot
```

Then:

- press `g` to send the computed target pose
- press `q` to quit

## Why The Robot Did Not Adjust Before

Because detection alone is not enough.

The missing chain was:

1. camera pose relative to tool
2. current robot pose in base frame
3. marker pose in camera frame
4. collector center pose in base frame
5. desired needle pose
6. desired tool pose
7. command to `send_coords(...)`

This new script adds exactly that chain.

## What Still Must Be Calibrated By Hand

You still need to adjust these values carefully:

- `tool_from_camera_position_m`
- `tool_from_camera_rpy_rad`
- `collector_from_tag_position_m`
- `collector_from_tag_rpy_rad`
- `needle_from_tool_position_m`
- `needle_target_from_collector_position_m`

Without these, the math runs, but the target point will be physically wrong.

## Recommended Practical Procedure

### Step 1

Make sure the marker is a real printed ArUco and fully visible.

### Step 2

Run detect-only mode and check that the marker is stable.

### Step 3

Adjust `collector_from_tag_position_m` until the computed collector center matches the real sphere center.

### Step 4

Adjust `needle_target_from_collector_position_m` until the desired gap is correct.

### Step 5

Only then test `--control-robot`.

## Troubleshooting

### Marker not detected

Most likely:

- not a real ArUco pattern
- marker partly hidden
- glare on the paper
- wrong `marker_id`
- wrong `marker_length_m`

### Robot target is computed but obviously wrong

Most likely:

- `tool_from_camera_*` is wrong
- `collector_from_tag_*` is wrong
- `needle_from_tool_*` is wrong

### Robot moves but does not face the collector well

That means the target orientation assumptions are still too rough.

The script already computes a usable target pose, but the exact `rpy` offsets still need refinement for your physical mount.
