# Eye-in-Hand Hand-Eye Calibration Guide

This Python-only script estimates the fixed transform from the robot tool to
the camera:

- `tool_from_camera_position_m`
- `tool_from_camera_rpy_rad`

It uses:

- the real MyCobot pose from `get_coords()`
- OpenCV marker pose estimation
- `cv2.calibrateHandEye(...)`

## What It Solves

The robot cannot directly tell you how the camera is mounted.

It only knows its own pose.

OpenCV can tell you the pose of the marker relative to the camera.

By collecting several robot poses while observing the same marker, we can solve
the fixed mounting transform between tool and camera.

## Requirements

- camera mounted on the robot tool / joint 6 area
- real robot connected
- a real, fully visible ArUco marker
- the same marker size and marker id already configured in your JSON config

## Command

```bash
python3 tools/eye_in_hand_handeye_calibration.py \
  --config tools/eye_in_hand_collector_servo_config.json \
  --samples-json tools/handeye_samples.json
```

## Workflow

1. launch the script
2. move the robot to a pose where the marker is visible
3. press `c` to capture one sample
4. repeat for 8 to 12 different poses
5. press `k` to compute the calibration
6. press `p` to save the result into the config
7. press `q` to quit

## Keys

- `c`: capture a sample
- `d`: drop the last sample
- `k`: run OpenCV hand-eye calibration
- `p`: save the calibrated `tool_from_camera_*` values into the config
- `q`: quit

## Important Advice

- change the robot pose significantly between captures
- keep the marker visible and not too small
- avoid capturing nearly identical poses
- use a real ArUco marker, not only a hand-drawn square

## Output

The script saves the captured samples to:

`tools/handeye_samples.json`

And after calibration it can update:

- [eye_in_hand_collector_servo_config.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/eye_in_hand_collector_servo_config.json)

with the estimated camera mounting transform.
