# ElectroSpin Autonomous Nanofiber Fabrication Platform

Commande de démo recommandée :
colcon build --packages-select ai_controller electrospin_bringup
source install/setup.bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_plus_vision

ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_tracked_hand:=right
Si l’image caméra est miroir et que droite/gauche paraît inversé :

ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_tracked_hand:=right \
  presentation_invert_direction:=true

ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  collector_mode:=passive_fixed \
  process_capabilities:=robot_only \
  optimization_mode:=off \
  enable_presentation_game:=true \
  presentation_tracked_hand:=right

ROS2-based autonomous electrospinning system for MyCobot 280 robotic arm with AI-assisted nanofiber deposition optimization and real-time teleoperation.

---

## System Architecture

```
                        ┌──────────────┐
                        │  Dashboard UI │
                        │  (PyQt5)     │
                        └──────┬───────┘
                               │
  ┌────────────┐    ┌──────────┴──────────┐    ┌─────────────────┐
  │   Vision    │───▶│   AI Controller    │───▶│ Command Bridge  │
  │  System     │    │  (Rule+Adaptive)   │    │ (Decomposer)    │
  └────────────┘    └──────────┬──────────┘    └────────┬────────┘
       │                       │                         │
       ▼                       ▼                         ▼
  /fiber_quality      /electrospin_command     ┌────────┴────────┐
       │                       │               │                 │
       │               ┌───────┴───────┐       ▼                 ▼
       │               ▼               ▼   /target_rpm    /target_flowrate
       │        Robot Controller  ┌─────┴──────┐     │              │
       │        (MyCobot Arm)    │  Collector  │     ▼              ▼
       │                        │  Controller │  Collector     Syringe
       │                        └─────────────┘  Motor          Pump
       │
  ┌────┴─────────────────────────────────────────────────────────────┐
  │                    Teleoperation Subsystem                        │
  │  ┌──────────────┐         ┌──────────────┐                       │
  │  │   Human      │─/human │   Motion     │─/motion_command       │
  │  │   Tracking   │─/pose─▶│   Mapping    │─────────▶Robot         │
  │  │  (MediaPipe) │         │  (IK+Safety)│                       │
  │  └──────────────┘         └──────────────┘                       │
  └──────────────────────────────────────────────────────────────────┘
```

---

## Packages Overview

| # | Package | Type | Description |
|---|---------|------|-------------|
| 1 | `electrospin_interfaces` | ament_cmake | Custom messages, services, actions |
| 2 | `robot_controller` | ament_python | MyCobot arm control + trajectory |
| 3 | `collector_controller` | ament_python | PID motor controller for collector |
| 4 | `syringe_controller` | ament_python | Precision syringe pump control |
| 5 | `vision_system` | ament_python | Real-time CV quality monitoring |
| 6 | `ai_controller` | ament_python | Rule-based + adaptive optimization |
| 7 | `dashboard_ui` | ament_python | Industrial PyQt5 control interface |
| 8 | `simulation_system` | ament_cmake | Gazebo + RViz + URDF |
| 9 | `electrospin_bringup` | ament_python | Master launch + system monitor |
| 10 | `human_tracking` | ament_python | MediaPipe pose + hand tracking |
| 11 | `motion_mapping` | ament_python | Human-to-robot IK + safety |

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 (or 20.04)
- ROS2 Humble (or Foxy)
- Python 3.8+

### ROS 2 Galactic Notes
- Supported target for Galactic: Ubuntu 20.04 + Python 3.8
- Install ROS packages with `ros-galactic-*` names instead of `ros-humble-*`
- `mediapipe` is optional and should be installed with `pip`, not `apt`
- `cv_bridge` should come from ROS packages, not `python3-cv-bridge`

### Install ROS2 Galactic
```bash
sudo apt install ros-galactic-desktop
source /opt/ros/galactic/setup.bash
```

### Install ROS2 Humble (optional)
```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### Install Python Dependencies
```bash
pip3 install numpy opencv-python mediapipe PyQt5
# Optional (for real hardware):
pip3 install pymycobot
# Optional (for advanced vision):
pip3 install torch ultralytics
```

### Install ROS2 Dependencies (Galactic)
```bash
sudo apt install \
  ros-galactic-gazebo-ros-pkgs \
  ros-galactic-robot-state-publisher \
  ros-galactic-joint-state-publisher-gui \
  ros-galactic-rviz2 \
  ros-galactic-cv-bridge \
  python3-colcon-common-extensions
```

### Install ROS2 Dependencies (Humble, optional)
```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-moveit \
  python3-colcon-common-extensions
```

### Galactic Teleoperation Extras
```bash
pip3 install mediapipe==0.10.0
```

---

## Building

```bash
# Navigate to workspace
cd electrospin_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Build specific packages only
colcon build --packages-select robot_controller collector_controller
```

---

## Launching

### Full Platform (Simulation Mode)
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py
```

### Full Platform (Real Hardware)
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  serial_port:=/dev/ttyUSB0
```

### Individual Packages
```bash
# Robot controller only
ros2 launch robot_controller robot_controller.launch.py

# Collector motor only
ros2 launch collector_controller collector_controller.launch.py

# Vision system only
ros2 launch vision_system vision_system.launch.py

# AI controller only
ros2 launch ai_controller ai_controller.launch.py

# Dashboard only
ros2 launch dashboard_ui dashboard.launch.py

# Syringe pump only
ros2 launch syringe_controller syringe_controller.launch.py
```

### Teleoperation (Human Tracking)
```bash
# Both human tracking + motion mapping together
ros2 launch human_tracking teleoperation.launch.py

# Or individually:
ros2 launch human_tracking human_tracking.launch.py
ros2 launch motion_mapping motion_mapping.launch.py
```

### Simulation Environment
```bash
ros2 launch simulation_system simulation.launch.py
```

### With Custom Parameters
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=true \
  optimization_mode:=adaptive \
  quality_target:=0.80 \
  enable_dashboard:=true \
  enable_simulation_env:=true
```

---

## Topic Map

### Core Electrospinning Topics

| Topic | Message Type | Publisher | Subscriber |
|-------|-------------|-----------|------------|
| `/electrospin_command` | ElectrospinCommand | ai_controller | command_bridge |
| `/fiber_quality` | FiberQuality | vision_system | ai_controller, dashboard |
| `/collector_status` | CollectorStatus | collector_controller | ai_controller, dashboard, system_monitor |
| `/robot_status` | String (JSON) | robot_controller | ai_controller, dashboard |
| `/ai_status` | String (JSON) | ai_controller | dashboard |
| `/system_status` | SystemStatus | system_monitor | dashboard |
| `/pump_status` | String (JSON) | syringe_controller | dashboard |
| `/joint_states` | JointState | robot_controller | dashboard, rviz |
| `/needle_distance` | Float32 | robot_controller | dashboard |
| `/emergency_stop` | Bool | dashboard, any node | all nodes |

### Command Bridge Output Topics

| Topic | Message Type | Publisher | Subscriber |
|-------|-------------|-----------|------------|
| `/target_rpm` | Float32 | command_bridge | collector_controller |
| `/target_flowrate` | Float32 | command_bridge | syringe_controller |
| `/target_distance` | Float32 | command_bridge | robot_controller |

### Teleoperation Topics

| Topic | Message Type | Publisher | Subscriber |
|-------|-------------|-----------|------------|
| `/human_pose` | HumanPose | human_tracking | motion_mapping, dashboard |
| `/hand_gesture` | HandGesture | human_tracking | motion_mapping, dashboard |
| `/motion_command` | MotionCommand | motion_mapping | robot_controller, dashboard |
| `/tracking/image` | Image | human_tracking | dashboard |

### Vision Topics

| Topic | Message Type | Publisher | Subscriber |
|-------|-------------|-----------|------------|
| `/vision_debug` | Image | vision_system | dashboard |
| `/camera/image_raw` | Image | vision_system | (external) |

---

## Custom Interfaces

### Messages

**FiberQuality.msg** — Nanofiber quality assessment
```
float32 uniformity          # Fiber diameter uniformity (0-1)
float32 diameter            # Average fiber diameter (nm)
float32 bead_score          # Bead formation severity (0-1)
bool    jet_stable          # Electrospinning jet stability
float32 taylor_cone_score   # Taylor cone quality (0-1)
float32 deposition_density  # Surface coverage (0-1)
float32 coverage_uniformity # Spatial uniformity (0-1)
float32 overall_quality     # Composite quality (0-1)
uint8   quality_grade       # 0=unusable, 1=poor, 2=fair, 3=good, 4=excellent
string  diagnosis            # Human-readable diagnosis
```

**CollectorStatus.msg** — Collector motor status
```
float32 rpm, target_rpm, torque, vibration_score, temperature_c, duty_cycle
bool    running, at_setpoint, emergency_stop, ramp_active
uint8   control_mode  # 0=IDLE, 1=OPEN_LOOP, 2=PID, 3=ESTOP
```

**ElectrospinCommand.msg** — Unified process command
```
float32 target_distance, target_rpm, target_flowrate, target_voltage
float32 target_scan_speed, scan_amplitude
bool    collector_enable, pump_enable, hv_enable
uint8   source  # 0=MANUAL, 1=AI_AUTO, 2=REPLAY
float32 confidence
string  rationale
```

**HumanPose.msg** — Full upper-body pose
```
float32[3] left/right_shoulder/elbow/wrist_position
float32   left/right_shoulder/elbow_visibility
float32[63] left/right_hand_landmarks  # 21 points x 3
float32   left/right_shoulder/elbow_angle
float32[3] left/right_upper_arm/forearm_dir
float32   overall_confidence
bool      person_detected
```

**HandGesture.msg** — Gesture recognition
```
uint8   gesture_id  # 0=NONE, 1=FIST, 2=OPEN, 3=POINT, 4=THUMBS_UP, 5=PEACE, 6=GRAB, 7=RELEASE
string  gesture_name
float32 confidence
uint8   command  # 0=NONE, 1=START, 2=STOP, 3=ESTOP, 4=MODE_SWITCH, 5=RESET
```

**MotionCommand.msg** — Mapped robot motion
```
float32[6] target_joint_angles
float32[3] target_position
float32[4] target_orientation
float32   confidence
bool      is_safe
uint8     source  # 0=TELEOP_LEFT, 1=TELEOP_RIGHT, 2=GESTURE, 3=AUTO
```

### Services

- **SetRPM.srv** — `float32 target_rpm` → `bool success, string message`
- **SetFlowRate.srv** — `float32 target_flow_ml_hr` → `bool success, string message`
- **SetDistance.srv** — `float32 target_distance_mm` → `bool success, string message`

### Actions

- **OptimizeProcess.action** — Long-running AI optimization with quality target, timeout, and feedback

---

## AI Optimization Modes

| Mode | Description |
|------|-------------|
| `off` | No autonomous control. Manual only. |
| `rule` | Rule-based correction only. 7 deterministic rules for beading, jet instability, uniformity, etc. |
| `adaptive` | Rule-based + gradient-free adaptive optimizer. Blends 70% rule + 30% adaptive. **Default.** |
| `rl` | Future: reinforcement learning policy (architecture ready, not yet implemented) |

### AI Decision Rules

| Condition | Action |
|-----------|--------|
| Severe beading (>0.6) | Reduce flow rate, increase RPM |
| Moderate beading (>0.3) | Slightly reduce flow, increase RPM |
| Jet unstable | Reduce flow rate and voltage |
| Weak Taylor cone | Increase voltage |
| Low uniformity (<0.4) | Increase distance, slow scan |
| Thick fibers (>2000nm) | Increase distance + RPM |
| Thin fibers (<100nm) | Decrease distance |
| Non-uniform coverage | Increase scan amplitude |
| Low deposition density | Slow scan speed, increase flow |

---

## Teleoperation Guide

### Setup
1. Connect a webcam to your system
2. Install mediapipe: `pip3 install mediapipe`
3. Launch teleoperation:
   ```bash
   ros2 launch human_tracking teleoperation.launch.py simulation_mode:=false
   ```

### Gesture Commands

| Gesture | Robot Command |
|---------|--------------|
| Fist | Emergency Stop |
| Thumbs Up | Start Process |
| Open Hand | Stop Process |
| Peace Sign | Switch Mode (Auto/Manual) |
| Point | No command (tracking only) |
| Grab | No command (tracking only) |

### Arm Tracking
- The system tracks the **right arm** by default
- Change with: `teleoperation_arm:=left` or `teleoperation_arm:=auto`
- Motion scaling: `scale_factor:=0.5` (50% of human motion maps to robot)
- Smoothing: `smoothing_alpha:=0.3` (lower = smoother, higher = more responsive)

### Safety
- All teleoperation commands pass through joint limit enforcement
- Velocity is capped at 2.0 rad/s by default
- E-stop from fist gesture immediately halts all motion
- Workspace boundary checking prevents overextension

---

## Dashboard Guide

The industrial dashboard provides real-time monitoring and control:

### Left Panel
- **Camera Feed** — Live vision system debug image
- **Fiber Quality** — 6 quality metric bars (overall, uniformity, bead, cone, coverage, density)
- **Trend Graphs** — Tabbed: Quality / RPM / Flow trends over time

### Center Panel
- **4 Circular Gauges** — RPM, Quality %, Flow rate, Distance
- **Robot Arm Visualization** — 2D side-view of MyCobot joints (blue=current, orange=target)
- **Skeleton Visualization** — Tracked human skeleton for teleoperation
- **Manual Controls** — RPM slider, flow slider, mode switch
- **Tabbed Status** — AI decisions / Motion commands / Pump status

### Right Panel
- **E-STOP** — Industrial emergency stop button
- **System Status** — State, mode, uptime
- **Collector Detail** — Running, setpoint, vibration, temperature, duty
- **Teleoperation** — Gesture, confidence, arm angles, tracking status
- **Diagnosis** — Color-coded quality diagnosis

---

## Simulation vs Real Hardware

### Simulation Mode (default)
- All nodes run with `simulation_mode:=true`
- Robot arm: Simulated joint angles (no pymycobot needed)
- Collector: First-order motor simulation with noise
- Syringe pump: Physics-based flow simulation
- Vision: Generates synthetic frames
- Human tracking: Generates synthetic skeleton motion
- No physical hardware required

### Real Hardware Mode
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py \
  simulation_mode:=false \
  serial_port:=/dev/ttyUSB0
```
- Requires pymycobot: `pip3 install pymycobot`
- Requires USB connection to MyCobot 280
- Requires motor driver for collector (serial/GPIO)
- Requires syringe pump driver
- Requires USB camera for vision

---

## Configuration Files

Each package has a YAML config in `config/`:

| Package | Config File | Key Parameters |
|---------|------------|----------------|
| robot_controller | `robot_controller.yaml` | simulation_mode, serial_port, control_frequency, default_distance_mm |
| collector_controller | `collector_controller.yaml` | pid_kp/ki/kd, max_rpm, ramp_rate_rpm_s |
| syringe_controller | `syringe_controller.yaml` | max_flow_ml_hr, pressure_limit_kpa, syringe_volume_ml |
| vision_system | `vision_system.yaml` | camera_index, processing_fps, use_yolo, debug_visualization |
| ai_controller | `ai_controller.yaml` | optimization_mode, decision_frequency_hz, quality_target |
| dashboard_ui | `dashboard.yaml` | simulation_mode, window_title |
| human_tracking | `human_tracking.yaml` | camera_index, tracking_fps, model_complexity, enable_hands/pose |
| motion_mapping | `motion_mapping.yaml` | teleoperation_arm, scale_factor, smoothing_alpha, max_joint_velocity |
| electrospin_bringup | `electrospin_bringup.yaml` | simulation_mode, watchdog_timeout_s |

---

## Troubleshooting

### Build Errors
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Missing Dependencies
```bash
rosdep install --from-paths src --ignore-src -y
```

### MediaPipe Not Found
```bash
pip3 install mediapipe
# If that fails on ARM:
pip3 install mediapipe==0.10.0
```

### Camera Not Opening
- Check device: `ls /dev/video*`
- Test: `ffplay /dev/video0`
- Change camera index in config: `camera_index: 1`

### No Quality Data
- Vision system needs time to warm up (5 seconds)
- AI controller starts with a 5-second delay
- Check: `ros2 topic echo /fiber_quality`

### E-Stop Active
- Click E-STOP button in dashboard to toggle
- Or publish: `ros2 topic pub /emergency_stop std_msgs/Bool '{data: false}' --once`

### Topic Debugging
```bash
# List all topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /fiber_quality
ros2 topic echo /electrospin_command
ros2 topic echo /human_pose
ros2 topic echo /motion_command

# Check topic frequency
ros2 topic hz /collector_status
ros2 topic hz /joint_states
```

---

## File Structure

```
electrospin_ws/src/
├── electrospin_interfaces/       # Messages, services, actions
│   ├── msg/                      # .msg files
│   ├── srv/                      # .srv files
│   ├── action/                   # .action files
│   └── CMakeLists.txt
├── robot_controller/             # MyCobot arm control
│   ├── robot_controller/         # Python package
│   ├── config/                   # YAML parameters
│   ├── launch/                   # Launch files
│   └── resource/                 # Ament marker
├── collector_controller/         # PID motor control
├── syringe_controller/           # Syringe pump control
├── vision_system/                # Computer vision
├── ai_controller/                # AI optimization
├── dashboard_ui/                 # PyQt5 interface
├── simulation_system/            # Gazebo + RViz
│   ├── urdf/                     # MyCobot URDF
│   ├── worlds/                   # Gazebo world
│   └── rviz/                     # RViz config
├── electrospin_bringup/          # Master launch + monitor
│   ├── electrospin_bringup/      # system_monitor + command_bridge
│   └── launch/                   # Master launch file
├── human_tracking/               # MediaPipe tracking
└── motion_mapping/               # IK + safety mapping
```
