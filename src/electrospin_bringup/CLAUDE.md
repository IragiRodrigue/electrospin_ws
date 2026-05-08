# ElectroSpin Autonomous Nanofiber Fabrication Platform

ROS2-based autonomous electrospinning system for MyCobot robotic arm.

## Architecture

```
Vision System (camera + AI CV)
        |
  FiberQuality analysis
        |
  AI Decision Controller
        |
  Command Optimization
        |
  Robot + Collector + Pump + HV
```

## Packages

| Package | Description |
|---------|-------------|
| `electrospin_interfaces` | Custom ROS2 messages, services, actions |
| `robot_controller` | MyCobot arm control, trajectory planning |
| `collector_controller` | PID motor controller for collector drum |
| `syringe_controller` | Precision syringe pump flow control |
| `vision_system` | Real-time CV quality monitoring |
| `ai_controller` | Rule-based + adaptive optimization |
| `dashboard_ui` | Industrial PyQt5 control interface |
| `simulation_system` | Gazebo + RViz simulation environment |
| `electrospin_bringup` | Master launch + system monitor |

## Build

```bash
cd electrospin_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

Full platform (simulation):
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py
```

Individual nodes:
```bash
ros2 launch robot_controller robot_controller.launch.py
ros2 launch collector_controller collector_controller.launch.py
ros2 launch vision_system vision_system.launch.py
ros2 launch ai_controller ai_controller.launch.py
```

With real hardware:
```bash
ros2 launch electrospin_bringup electrospin_bringup.launch.py simulation_mode:=false
```

## Key Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/fiber_quality` | FiberQuality | vision -> ai |
| `/electrospin_command` | ElectrospinCommand | ai -> robot/collector/pump |
| `/collector_status` | CollectorStatus | collector -> ai/dashboard |
| `/robot_status` | String (JSON) | robot -> ai/dashboard |
| `/ai_status` | String (JSON) | ai -> dashboard |
| `/system_status` | SystemStatus | monitor -> dashboard |
| `/emergency_stop` | Bool | any -> all |

## AI Optimization Modes

- `off` — No autonomous control
- `rule` — Rule-based correction only
- `adaptive` — Rule-based + gradient adaptation (default)
- `rl` — Future: reinforcement learning policy
