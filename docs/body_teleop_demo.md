# Body Teleoperation Demo Guide

This Python-only demo lets the MyCobot 280 follow simple upper-body gestures
with safety limits in joint space.

It is designed for:

- camera-only testing first
- then safe robot teleoperation on the Jetson Nano MyCobot 280
- direct joint-space control with conservative limits

## What It Does

- tracks your shoulders and wrists with MediaPipe
- learns a neutral pose
- maps torso side motion to robot base rotation
- maps torso bend to shoulder and elbow motion
- maps right hand motion to wrist orientation
- freezes when your body is still
- freezes when an open palm is detected
- switches to a compact "fist-like" robot pose when a fist is detected

## Important Limitation

The MyCobot 280 has no human fingers.

So when you make a `fist`, the robot cannot literally close a hand unless you
have a separate gripper. In this script, `fist` is approximated by a compact
arm-and-wrist pose.

## Safety Choices

- all commands stay inside conservative MyCobot 280 joint limits
- commands are smoothed with an EMA filter
- commands are sent only after a minimum interval
- very small body motions are ignored
- if the body is still, the robot holds its current pose
- open palm acts as a stop-and-hold gesture

## Dry Run First

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0
```

## Real Robot Mode

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0 \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000
```

## Keys

- `b`: capture the current body pose as the neutral reference
- `r`: reset the neutral reference
- `s`: save the latest state JSON if `--save-last-json` is provided
- `q`: quit

## Recommended Test Procedure

1. launch the script without `--control-robot`
2. stand in front of the camera in a comfortable neutral pose
3. press `b`
4. lean left and right and verify the target joints change smoothly
5. bend your torso and verify the robot target bends too
6. open your palm and verify the target freezes
7. make a fist and verify the target switches to the compact pose
8. only then relaunch with `--control-robot`

## Notes About Coordinate Systems

This demo intentionally works in robot joint space instead of Cartesian space.

That is safer for early teleoperation because:

- it avoids unstable inverse-kinematics jumps
- it keeps every joint inside explicit angle limits
- it matches the MyCobot 280 control mode used by `pymycobot.send_angles(...)`

## Useful Example

```bash
python3 tools/body_teleop_demo.py \
  --camera-index 0 \
  --control-robot \
  --serial-port /dev/ttyTHS1 \
  --baud-rate 1000000 \
  --robot-speed 15 \
  --send-interval 0.20 \
  --save-last-json tools/body_teleop_state.json
```
