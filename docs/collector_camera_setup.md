# Collector Camera Guide

This guide explains how to test collector recognition with a plain Python script first, then enable the same workflow inside ROS 2.

## Goal

The camera does not need to "understand" every object in the scene. Instead, it looks for one unique ArUco marker attached to the collector support. From that marker, the system computes the collector center pose.

The chain is:

1. Camera sees the ArUco marker.
2. OpenCV estimates the marker pose relative to the camera.
3. We define where the camera is relative to the robot base.
4. We define where the collector center is relative to the marker.
5. The script or ROS 2 node computes the collector center in the robot frame.

## Files Added

- Standalone Python test: [collector_camera_demo.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/collector_camera_demo.py)
- Example config: [collector_camera_demo_config.example.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/collector_camera_demo_config.example.json)
- ROS 2 tracker node: [collector_tracker_node.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/src/vision_system/vision_system/collector_tracker_node.py)

## Hardware Setup

You need:

- a process camera visible as `/dev/video0`
- one printed ArUco marker fixed on the collector support
- a stable camera mount
- the robot base and collector kept fixed during calibration

Recommended starting point:

- marker dictionary: `DICT_4X4_50`
- marker id: `0`
- marker size: `30 mm`

## Python-Only Test First

### 1. Install Python dependencies

On Jetson or Ubuntu:

```bash
python3 -m pip install numpy opencv-contrib-python
```

If `opencv-contrib-python` is too heavy or conflicts with the system OpenCV, make sure your existing OpenCV has `cv2.aruco`:

```bash
python3 - <<'PY'
import cv2
print("aruco available:", hasattr(cv2, "aruco"))
PY
```

### 2. Copy the example config

```bash
cd ~/pymycobot/electrospin_ws
cp tools/collector_camera_demo_config.example.json tools/collector_camera_demo_config.json
```

### 3. Edit the config

Open:

- [collector_camera_demo_config.example.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/collector_camera_demo_config.example.json)

The important fields are:

- `camera_index`: should stay `0` for your setup
- `marker_id`: the id printed on the marker
- `marker_length_m`: physical marker size in meters
- `camera_in_robot_position_m`: where the camera is in the robot base frame
- `camera_in_robot_rpy_rad`: how the camera is rotated in the robot base frame
- `collector_from_tag_position_m`: offset from marker center to collector center
- `collector_from_tag_rpy_rad`: orientation offset from tag frame to collector frame

### 4. Run the standalone script

```bash
cd ~/pymycobot/electrospin_ws
python3 tools/collector_camera_demo.py \
  --config tools/collector_camera_demo_config.json \
  --show-tag-frame
```

What you should see:

- a live camera window
- a detected marker outline
- marker axes drawn on the image
- printed `collector xyz` values in millimeters

Useful keys:

- `q`: quit
- `s`: save last pose if `--save-last-pose` was provided

Example with pose export:

```bash
python3 tools/collector_camera_demo.py \
  --config tools/collector_camera_demo_config.json \
  --save-last-pose tools/last_collector_pose.json
```

## How To Measure The Two Critical Transforms

You must define two transforms.

### A. Camera in robot frame

This answers:

"Where is the camera relative to the robot base?"

You set:

- `camera_in_robot_position_m = [x, y, z]`
- `camera_in_robot_rpy_rad = [roll, pitch, yaw]`

Practical way:

1. Choose the robot base frame origin.
2. Measure camera center position relative to that origin.
3. Estimate camera orientation.
4. Run the script and refine until the reported collector position is plausible.

### B. Collector center from marker frame

This answers:

"Where is the real collector center relative to the marker?"

You set:

- `collector_from_tag_position_m = [x, y, z]`
- `collector_from_tag_rpy_rad = [roll, pitch, yaw]`

Practical way:

1. Attach the marker rigidly to the collector support.
2. Measure the offset from the marker center to the collector center.
3. Put that offset into the config.
4. Refine after checking the live output.

## When The Python Demo Is Good

Once the standalone script is stable, use the same values in ROS 2.

## ROS 2 Setup

### 1. Build the relevant packages

```bash
cd ~/pymycobot/electrospin_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select vision_system robot_controller electrospin_bringup dashboard_ui
source install/setup.bash
```

### 2. Launch with collector tracking enabled

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=true \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  camera_index:=0 \
  enable_collector_tracking:=true \
  collector_tracking_marker_id:=0 \
  collector_marker_length_m:=0.03 \
  collector_tracking_camera_frame:=camera_frame \
  camera_in_robot_x_m:=0.18 \
  camera_in_robot_y_m:=0.02 \
  camera_in_robot_z_m:=0.24 \
  camera_in_robot_roll_rad:=0.0 \
  camera_in_robot_pitch_rad:=-0.35 \
  camera_in_robot_yaw_rad:=0.0 \
  collector_from_tag_x_m:=0.0 \
  collector_from_tag_y_m:=0.05 \
  collector_from_tag_z_m:=0.0 \
  collector_from_tag_roll_rad:=0.0 \
  collector_from_tag_pitch_rad:=0.0 \
  collector_from_tag_yaw_rad:=0.0
```

## ROS 2 Runtime Checks

### Collector tracking status

```bash
ros2 topic echo /collector_tracking_status
```

Look for:

- `"tracking_ok": true`
- a fresh `pose_m`

### Collector pose used by the robot

```bash
ros2 topic echo /collector_pose
```

### Robot status

```bash
ros2 topic echo /robot_status
```

Look for:

- `"collector_pose_source": "tracked"`

If tracking is lost, the robot falls back automatically to the fixed collector coordinates.

## Optional Debug Image

To visualize the annotated tracking stream:

```bash
ros2 run image_tools showimage --ros-args --remap image:=/collector_tracking_debug
```

## Recommended Bringup Order

### Camera-only test

```bash
python3 tools/collector_camera_demo.py \
  --config tools/collector_camera_demo_config.json \
  --show-tag-frame
```

### Robot + camera + tracking, no simulation

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=false \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  camera_index:=0 \
  enable_collector_tracking:=true
```

### Full runtime with dashboard

```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  enable_simulation_env:=false \
  enable_dashboard:=true \
  serial_port:=/dev/ttyTHS1 \
  baud_rate:=1000000 \
  camera_index:=0 \
  enable_collector_tracking:=true
```

## Troubleshooting

### `cv2.aruco` missing

Install an OpenCV build that includes contrib modules:

```bash
python3 -m pip install opencv-contrib-python
```

### Camera opens but no marker is detected

- check lighting
- check marker size
- check that the printed marker id matches the config
- move the camera closer
- verify the marker is not too tilted

### Pose is detected but the collector position is wrong

This almost always means one of these is wrong:

- `camera_in_robot_*`
- `collector_from_tag_*`
- `marker_length_m`

### ROS 2 starts but robot still uses fixed pose

Check:

```bash
ros2 topic echo /collector_tracking_status
ros2 topic echo /robot_status
```

If tracking is stale or missing, the robot intentionally switches back to the fixed collector pose for safety.
