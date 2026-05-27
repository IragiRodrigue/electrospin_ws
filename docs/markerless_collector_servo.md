# Markerless Collector Servo Guide

This Python-only script tries to follow the reflective spherical collector with
the camera mounted on the robot.

## What It Does

- detects the sphere directly from the image
- estimates sphere depth from the known real diameter
- computes a small correction for the robot tool pose
- can send `send_coords(...)` commands to the MyCobot

## Limits

- this is less precise than the marker-based version
- the sphere is shiny, so reflections can disturb tracking
- the script keeps the current tool orientation and mainly corrects position

## Recommended Flow

1. first test tracking only
2. then test single-step motion
3. only after that enable repeated follow

## Commands

Dry run:

```bash
python3 tools/markerless_collector_servo.py \
  --config tools/markerless_collector_servo_config.example.json
```

With robot control:

```bash
python3 tools/markerless_collector_servo.py \
  --config tools/markerless_collector_servo_config.example.json \
  --control-robot
```

## Keys

- `g`: toggle continuous follow
- `m`: send one correction step
- `s`: save last detection JSON if `--save-last-json` is used
- `q`: quit

## Important Parameter

Set the real collector diameter:

```json
"sphere_diameter_m": 0.05
```

For a `50 mm` sphere, `0.05` is correct.

If the sphere gets closer to the camera, its image diameter increases. That is
normal and it is exactly what the script uses to estimate distance.
