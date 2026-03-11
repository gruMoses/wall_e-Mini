# Obstacle Avoidance — Ground-Plane Corridor Plan

**Date:** 2026-03-09

## What We Did Today

### Board-Based Width Measurement

Mounted wooden boards to both sides of the robot to define its physical edges in
the camera frame. Gradient analysis of the RGB frame found:

- Left board edge: x=75 (11.7% of frame)
- Right board edge: x=567 (88.6% of frame)
- Robot width in frame: 77% (492px of 640)
- Camera is centered (0.2% offset)
- Robot angular width: 59.3° of the camera's 73° HFOV

### Config: Physical Geometry

Added to `ObstacleAvoidanceConfig` in `config.py`:

```python
camera_height_m: float = 0.497
robot_width_m: float = 0.820
camera_hfov_deg: float = 73.0
```

### Depth Check: Per-Pixel Corridor Masking

Replaced the old rectangular ROI crop in `oak_depth.py` `_poll_depth()` with
per-pixel corridor masking. For each depth pixel:

```
depth_mm × |x_pixel − center| ≤ focal_length × robot_half_width_mm
```

This checks whether the pixel's real-world X offset falls within the robot's
physical half-width at that pixel's measured depth. Works for obstacles at any
height — walls, furniture, overhangs — not just ground-level.

### Visualization: Ground-Plane Triangle

Changed the overlay in `oak_recorder.py` `_annotate_rgb()` from a rectangle to:
- **Cyan triangle**: Full frame width at bottom, converging to vanishing point
  at the horizon (center of frame). Represents the robot's forward corridor.
- **Yellow dashed lines**: The ROI vertical band where depth is currently checked.

### Bug Fix: Hardcoded ROI

Fixed `_update_rgb_preview()` which was passing hardcoded `roi_pct=(0.5, 0.5)`
regardless of config. Now passes the full `ObstacleAvoidanceConfig`.

---

## Next Step: Ground-Plane-Aware Obstacle Detection

### Problem

The current depth check uses a **fixed vertical ROI band** (upper 55% of frame,
shifted up with `roi_vertical_offset_pct=-0.20`). This has drawbacks:

- At far range, checks a corridor way wider than the robot
- At close range, may miss low obstacles below the ROI band
- The ROI position is a tuning parameter rather than being derived from geometry
- The ground plane itself can trigger false positives

### Proposed Solution

Replace the fixed ROI band with **ground-plane-aware per-pixel detection**.
For every pixel `(x, y)` in the depth frame with measured depth `d`:

1. **Corridor check** (already implemented):
   `d × |x − cx| ≤ fx × robot_half_width`

2. **Ground plane check** (new):
   - Below horizon (`y > cy`): pixel is above ground if `d × (y − cy) < fy × h_cam`
   - At/above horizon (`y ≤ cy`): always above ground
   - Intuition: at each pixel's depth, compute where the ground *should* be
     vertically. If the pixel is higher than that, it's an obstacle.

3. **Min-height filter** (optional):
   - Height above ground: `h_above = h_cam − d × (y − cy) / fy`
   - Require `h_above > min_obstacle_height` (e.g., 3-5cm) to filter noise

### What This Eliminates

- `roi_height_pct` — no longer needed
- `roi_vertical_offset_pct` — no longer needed
- `roi_width_pct` — replaced by per-pixel corridor check

Only physical measurements remain: `camera_height_m`, `robot_width_m`,
`camera_hfov_deg`, and optionally `min_obstacle_height_m`.

### Efficiency

All checks are element-wise numpy operations on the 640×400 depth frame
(~256K pixels). No loops, no pointcloud conversion. Broadcasting handles
the x-offset and y-offset grids.

### Research References

- `laserscan_kinect` (ROS `depth_nav_tools`): column-wise min-distance with
  ground plane removal. Same concept, implemented for ROS laser scan format.
- U-V Disparity method: column/row histograms for ground detection and obstacle
  segmentation in stereo images.
- ROS `costmap_2d`: uses `min_obstacle_height` / `max_obstacle_height` to filter
  points after transforming depth to 3D.

### Caveats

- Assumes flat ground and level camera. On slopes or when the robot pitches,
  the ground plane estimate shifts. IMU pitch data could compensate in the future.
- Very low obstacles (< 3cm) may be filtered out by the min-height threshold.

### Visualization Update

The triangle overlay already shows the correct ground corridor. Once implemented,
the yellow dashed ROI band lines would be removed since the geometry fully
determines the detection zone.

---

## Files Changed (2026-03-09)

| File | Changes |
|------|---------|
| `config.py` | Added `camera_height_m`, `robot_width_m`, `camera_hfov_deg` to `ObstacleAvoidanceConfig` |
| `pi_app/hardware/oak_depth.py` | Per-pixel corridor masking in `_poll_depth()` |
| `pi_app/hardware/oak_recorder.py` | Trapezoid→triangle overlay; accepts `ObstacleAvoidanceConfig`; fixed hardcoded roi_pct |
| `pi_app/app/main.py` | Passes `obstacle_config` to `OakRecorder` |
