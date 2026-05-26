# Markerless Collector Tracking Guide

This guide explains the no-marker version.

It is useful when:

- the collector can move
- you want to see whether the camera can still follow it
- you want a first test without relying on an ArUco marker

## Important Limitation

Markerless tracking is not equivalent to marker-based pose estimation.

With the current single RGB camera, markerless tracking gives you mainly:

- the sphere center in the image
- the apparent radius in pixels
- an approximate depth if the sphere diameter is known

It does not give a robust full 6D pose like a fiducial marker.

## File

- [markerless_collector_tracker.py](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/markerless_collector_tracker.py)
- [markerless_collector_tracker_config.example.json](/C:/Users/Rodrigue/Documents/ai/electrospin_ws/tools/markerless_collector_tracker_config.example.json)

## How It Works

The script tries to find the sphere using:

1. circle detection by Hough transform
2. bright contour fallback for the reflective aluminum sphere

From the radius in pixels and the known real diameter, it estimates:

- approximate `z`
- approximate lateral offsets `x` and `y`

## Your Sphere

From your photos, the sphere is reflective and approximately circular, which makes this method possible.

But it also means:

- glare can disturb the contour
- reflections can move with lighting
- the circle may be unstable if the background is cluttered

So this method is good for testing tracking, but less good for final precise control.

## Install

```bash
python3 -m pip install numpy opencv-python
```

## Copy Config

```bash
cd ~/pymycobot/electrospin_ws
cp tools/markerless_collector_tracker_config.example.json tools/markerless_collector_tracker_config.json
```

## Set Sphere Diameter

In the config, set:

```json
"sphere_diameter_m": 0.07
```

Replace `0.07` with the real diameter of your collector sphere in meters.

## Run

```bash
python3 tools/markerless_collector_tracker.py \
  --config tools/markerless_collector_tracker_config.json
```

## What You Should See

- a green circle on the collector
- the estimated center in pixels
- the estimated camera-frame position

## Save A Result

```bash
python3 tools/markerless_collector_tracker.py \
  --config tools/markerless_collector_tracker_config.json \
  --save-last-json tools/last_markerless_collector.json
```

Press `s` to save.

## Why This Is Useful

This tells you whether the collector can still be followed after removing the marker.

If this script loses the sphere too often, then the robot should not rely on markerless tracking alone.

## Best Practice

Use markerless tracking as:

- a backup
- a visual servo hint
- or a tracking aid

Not as the only source of precise 3D geometry at the beginning.

## Recommended Strategy

1. keep the marker for accurate geometry and calibration
2. test markerless tracking in parallel
3. if markerless tracking is stable enough, use it to maintain visual lock when the marker disappears

## Troubleshooting

### No sphere detected

- increase lighting
- simplify the background
- reduce glare
- adjust `hough_*` values
- adjust `brightness_threshold`

### Wrong depth estimate

Most likely:

- wrong `sphere_diameter_m`
- wrong focal estimate
- camera intrinsics not calibrated

### Jitter

This is expected with reflections.

The next improvement would be temporal smoothing or a Kalman filter.
