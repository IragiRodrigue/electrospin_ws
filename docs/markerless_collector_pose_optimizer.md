# Markerless Collector Pose Optimizer Guide

This Python-only script goes beyond simple visual following.

It tries to:

- detect the collector sphere without a marker
- estimate the collector center in 3D from the camera image
- choose a better robot pose around the sphere
- keep a desired gap between the needle and the collector
- keep the collector visible from the camera on the robot

## What Makes It Different

The simpler `markerless_collector_servo.py` mostly applies local corrections.

This optimizer instead:

1. estimates the collector center
2. generates many candidate approach directions around the sphere
3. builds a target tool pose for each candidate
4. rejects poses outside the robot workspace
5. scores the remaining poses
6. sends the best one

## Distance Logic

The sphere diameter in the real world does not change.

What changes is the diameter in pixels in the camera image.

That apparent diameter is what the script uses to estimate depth. For a
`50 mm` sphere, use:

```json
"sphere_diameter_m": 0.05
```

The desired needle gap is configured with:

```json
"desired_gap_m": 0.15
```

That means the target needle position is placed at:

`collector_radius + desired_gap`

from the sphere center, along the chosen approach direction.

## Dry Run

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json
```

## Real Robot Mode

```bash
python3 tools/markerless_collector_pose_optimizer.py \
  --config tools/markerless_collector_pose_optimizer_config.example.json \
  --control-robot
```

## Keys

- `o`: optimize once and send the best pose
- `g`: toggle repeated auto-optimize
- `s`: save the latest optimized result JSON if `--save-last-json` is used
- `q`: quit

## Important Limitation

This is still markerless on a reflective sphere, so the 3D estimate is
approximate. It is more intelligent than simple recentering, but it is still
less robust than a true fiducial-based pipeline.

## Recommended Use

1. test in dry-run first
2. verify the estimated collector center is stable
3. use `o` for single optimized moves
4. only then try `g` for repeated optimization
