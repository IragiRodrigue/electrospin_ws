# Human Tracking And Presentation Guide

This guide covers:

- live human tracking with MediaPipe
- teleoperation with `human_tracking + motion_mapping`
- the presentation game where your hand direction drives robot joint 6 like a head turn

## What Each Mode Does

### 1. Human tracking only

The camera detects:

- upper-body pose
- left and right wrists
- optional hand landmarks
- static hand gestures

It publishes:

- `/human_pose`
- `/hand_gesture`
- `/tracking/image`

### 2. Teleoperation

This adds motion mapping on top of human tracking.

The chain is:

1. camera sees your arm
2. `human_tracking` estimates pose
3. `motion_mapping` converts pose to a safe `MotionCommand`
4. `robot_controller` executes the motion command

### 3. Presentation game

This is the demo mode for the professor.

The chain is:

1. camera sees your hand and upper body
2. `human_tracking` publishes pose and gesture
3. `presentation_game` decides left, right, or center
4. `robot_controller` rotates joint 6

This makes the robot appear to "turn its head" in the same direction as your hand.

## Important Camera Rule

Do not try to use the same camera for all demo modes at once unless you are very sure about the pipeline.

Recommended separation:

- collector tracking mode: process camera only
- teleoperation mode: human tracking camera only
- presentation game mode: human tracking camera only

In the current bringup, presentation mode already disables the normal process vision node to avoid camera conflicts.

## Dependencies

For human tracking you need:

```bash
python3 -m pip install mediapipe==0.10.0 opencv-python numpy
```

For ROS 2 Galactic, make sure these are present too:

```bash
sudo apt install ros-galactic-cv-bridge
```

## 1. Human Tracking Only

### Launch

```bash
source /opt/ros/galactic/setup.bash
cd ~/pymycobot/electrospin_ws
source install/setup.bash

ros2 launch human_tracking human_tracking.launch.py \
  simulation_mode:=false \
  camera_index:=0 \
  debug_visualization:=true
```

### Check topics

```bash
ros2 topic list
ros2 topic echo /human_pose
ros2 topic echo /hand_gesture
```

### Visual check

```bash
ros2 run image_tools showimage --ros-args --remap image:=/tracking/image
```

You should see:

- pose landmarks on the upper body
- hand tracking when your hand is visible
- gesture output changing when you open, point, or close your hand

## 2. Teleoperation

### Launch combined teleoperation

```bash
ros2 launch human_tracking teleoperation.launch.py \
  simulation_mode:=false \
  camera_index:=0 \
  teleoperation_arm:=right \
  scale_factor:=0.5 \
  debug_visualization:=true
```

### Check motion output

```bash
ros2 topic echo /motion_command
ros2 topic echo /motion_status
```

### What should happen

- your tracked arm drives safe robot motion targets
- motion is smoothed
- workspace and joint limits are enforced by the mapping stage

### If you want the full robot in the loop

Run the robot controller separately or via bringup:

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=false \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only
```

Then in another terminal:

```bash
ros2 launch human_tracking teleoperation.launch.py \
  simulation_mode:=false \
  camera_index:=0 \
  teleoperation_arm:=right \
  scale_factor:=0.5 \
  debug_visualization:=true
```

## 3. Presentation Game

### Purpose

This mode is for a clean, safe live demo. It does not do full teleoperation. It only uses left/right guidance to rotate joint 6.

### Launch

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=true \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_camera_index:=0 \
  presentation_tracked_hand:=right
```

### If left/right feels mirrored

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=true \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_camera_index:=0 \
  presentation_tracked_hand:=right \
  presentation_invert_direction:=true
```

### Topics to inspect

```bash
ros2 topic echo /presentation_game_status
ros2 topic echo /motion_command
ros2 topic echo /robot_status
```

You want to see:

- `/presentation_game_status` switching between `left`, `right`, and `center`
- `/motion_command` containing `target_joint_angles`
- `/robot_status` showing `"teleop_active": true` while commands are flowing

## What Gesture Activates The Game

The current default is `point_or_open`.

That means the game responds only when the recognized gesture is:

- `point`
- or `open`

This reduces false triggers.

## Tuning Parameters

### Human tracking

Main parameters:

- `camera_index`
- `tracking_fps`
- `debug_visualization`
- `enable_hands`
- `enable_pose`

### Teleoperation

Main parameters:

- `teleoperation_arm`
- `scale_factor`

### Presentation game

Main parameters:

- `presentation_tracked_hand`
- `presentation_invert_direction`
- `joint6_max_deg`
- `deadband_m`
- `direction_smoothing`

## Recommended Demo Sequence

### Step 1. Verify the camera and tracking only

```bash
ros2 launch human_tracking human_tracking.launch.py \
  simulation_mode:=false \
  camera_index:=0 \
  debug_visualization:=true
```

### Step 2. Verify the presentation game without the professor

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=false \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_camera_index:=0
```

### Step 3. Use the dashboard once the behavior is stable

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=true \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_camera_index:=0
```

## Troubleshooting

### No `/human_pose`

- check `mediapipe` is installed
- check `/dev/video0` is free
- check the camera is not already used by another node

### `/human_pose` exists but no gesture

- make sure `enable_hands` is true
- move the hand closer to the camera
- improve lighting

### Teleoperation publishes nothing

Check:

```bash
ros2 topic echo /human_pose
ros2 topic echo /motion_status
```

If pose confidence is low, motion mapping will not produce useful output.

### Presentation game starts but robot does not turn

Check:

```bash
ros2 topic echo /presentation_game_status
ros2 topic echo /motion_command
ros2 topic echo /robot_status
```

Possible causes:

- gesture gate not satisfied
- wrist visibility too low
- `joint6_max_deg` too small to notice
- robot controller not receiving motion commands

### Robot and collector tracking together

This is possible, but only if the camera arrangement is planned carefully.

For presentation day, the safer recommendation is:

- use collector tracking for process demos
- use presentation game for the professor demo
- do not try to run both on one camera feed live unless you test it thoroughly beforehand
