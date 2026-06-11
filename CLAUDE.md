# WALL-E Mini — Codebase Guide

## Architecture Overview

Python control stack running on a Raspberry Pi 5. Entry point is `pi_app/app/main.py` (a tight control loop at ~30 Hz). All subsystem state is threaded and shared via lightweight dataclasses; the main loop reads latest snapshots and publishes commands.

```
pi_app/
  app/main.py          # Main loop, subsystem wiring, SSE web server (port 8080)
  control/             # Logic: controller, safety, follow_me, waypoint_nav, obstacle_avoidance, imu_steering
  hardware/            # Drivers: vesc, arduino_rc, oak_depth, oak_imu, imu_reader, bms, rtk_gps
  cli/                 # Calibration and test harnesses
  tests/               # Unit tests
config.py              # Single Config dataclass (root of repo)
```

## Key Subsystems

### Motor Output
VESC over CAN (`can0`) is the primary drive path; Arduino motor-driver fallback is wired in. All motor commands are "bytes" (0–255 centre-stop, range ~10–245). Left/right are issued separately via `VescCanDriver`. A command-space mixer converts `(v_cmd, yaw_cmd)` floats in `[-1, 1]` into left/right bytes.

### IMU
- **Active hardware**: OAK-D Lite onboard **BMI270** (gyro + accelerometer; **no magnetometer**).
  - Heading is integrated from gyro and is **relative to startup orientation** — it is not referenced to magnetic north or GPS.
  - `imu_source = "auto"` (default): tries external I2C breakout first, falls back to OAK-D BMI270.
  - External I2C options: ICM-20948 or ISM330DHCX + MMC5983MA combo (if physically present).
  - `imu_use_magnetometer = False` is the current default — magnetometer fusion is disabled.
- IMU feeds `ImuSteeringCompensator` (PID heading-hold, differential byte correction).
- GPS COG heading alignment is **in progress** — will let the robot zero its heading from GPS course-over-ground at startup rather than assuming the startup pose is the reference.

### OAK-D Lite Camera
- Obstacle avoidance: depth corridor, valid-pixel % threshold, tiered speed reduction/stop.
- Follow Me: **YOLOv8n** blob via `NeuralNetwork` node + host-side NMS (depthai v3). Do **not** use `SpatialDetectionNetwork` / `YoloDetectionNetwork` / `DetectionParser` — those silently yield zero detections with ultralytics blobs in depthai v3.
- Depth EMA filter on person position for smoothing.

### Waypoint Navigation
State machine in `pi_app/control/waypoint_nav.py`:
- **ALIGN**: pivot in place until heading error < `align_threshold_deg` (default 12°). Yaw sign: negative yaw_cmd for positive (right) heading error.
- **DRIVE**: forward at cruise speed with PID steering; falls back to ALIGN if error exceeds `recovery_threshold_deg`.
- **ARRIVE**: within `arrival_radius_m` of target; zeroes commands and advances waypoint.
- `compute()` returns `(v_cmd, yaw_cmd, state)` as floats; caller's mixer converts to motor bytes.
- Requires RTK GPS fix quality ≥ 4 (default). Stale GPS (>3 s) halts motion.

### BMS
Daly SPIM08HP over BLE (`bleak`). Polls SOC / cell voltages / temp / MOSFET status. Auto-reconnect. `is_charging()` feeds charger-inhibit into controller every loop tick. Fail-open after 30 s BLE dropout.

## Known Issues

- **VESC RPM telemetry — RESOLVED 2026-06-11**: bench test (`tools/vesc_rpm_bench.py`, wheels off ground) proved ERPM readback works: commanded +1500, steady-state error −1.0% L / −0.4% R, ~100 Hz per STATUS type, zero parse errors. The old "always 0" observation did not reproduce. Two follow-ups: (1) confirm `vesc_left_rpm`/`vesc_right_rpm` in /api/telemetry go nonzero during the next real drive, then consider re-enabling the velocity PID (`speed_kp/ki/kd` in config.py, currently 0). (2) VESC firmware has a ~1 s command timeout — motors stop if drive commands are not refreshed (confirmed on hardware; the service's 15–30 Hz loop clears it easily, and it acts as a free deadman).
- **OAK USB disconnect**: no auto-reconnect; service restart required if OAK re-enumerates.
- **IMU heading drift**: gyro-only integration drifts over time (minutes). GPS COG alignment (in progress) will partially mitigate this at session start.

## Config

`config.py` (repo root) — one frozen `Config` dataclass with nested sub-configs. Edit this to change thresholds, enable/disable subsystems, set BMS MAC, VESC CAN IDs, etc.

## Running Tests

```bash
cd /home/pi/wall_e-Mini
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

## Branches

- `research/follow-me-trail-strategies`: research notes (not merged; 1 commit ahead of main).
- Remote merged branches (`origin/codex/*`, `origin/feature/property-map`, `origin/feature/trail-follow-pure-pursuit`) can be pruned with `git remote prune origin`.
