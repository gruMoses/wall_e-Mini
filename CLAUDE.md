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
- GPS COG heading alignment is **implemented** (software) — one-shot locks IMU heading to true north during a forward, straight manual RTK-fixed run, then freezes the offset for the armed session; field validation pending (`docs/gps_heading_alignment.md`).

### Camera calibration — READ THIS BEFORE TOUCHING ANY CAMERA GEOMETRY

Geometry comes from the device's per-unit factory EEPROM, **not** from a
hand-entered field of view. `OakDepthReader.get_intrinsics(w, h)` is the single
source; the obstacle corridor, person position and the recorder overlay all go
through it. Never re-derive a focal length from an FOV constant — that is
exactly how the overlay ended up drawing different geometry from the corridor.

Measured on this unit (`python3 -m pi_app.cli.oak_intrinsics`, read-only; stop
`wall-e.service` first because the OAK allows one process at a time):

| | |
|---|---|
| CAM_A @ 640×400 | fx = 456.89, cx = 334.95 |
| Horizontal FOV | **70.01°** (EEPROM spec 68.794°) |
| Principal point | **+14.9 px off centre** — never assume cx = w/2 |
| VFOV at the 640×352 detection frame | **42.13°** |

Startup logs the numbers in use at WARNING level (the app installs no logging
handler, so INFO is dropped). A silent fallback to the config constant changes
safety geometry, so it is logged loudly.

**Three defects this replaced (2026-07-26), all from one wrong constant:**
`camera_hfov_deg` was 81.0 — the *diagonal* FOV in a horizontal field.
1. Corridor threshold 18% too small: guarded a ~0.67 m robot instead of 0.82 m.
2. `detect_camera_vfov_deg` 65.3 → 42.13. Implied heights had been inflated
   1.66×, so the 1.20 m "reject short ground blobs" gate was really 0.72 m.
3. DetectionFilter Rule 3 now **skips boxes clipped by the top frame edge** —
   height is unmeasurable on a truncated box. Bottom contact is deliberately
   NOT excused, because ground animals rest against the frame bottom.

WARNING: (1) and (2) were cancelling each other. Fixing either alone makes the
robot worse — correcting the FOV without (3) filters the operator out at
`follow_distance_m`. Expect this pattern elsewhere; see
`~/Documents/screenshots/WALLE-constant-audit.md`.

Open: `detect_min_person_height_m = 1.20` was tuned against implied heights
recorded under the inflated VFOV, so it may now be too aggressive. Needs one
recorded walk replayed through `tools/replay_follow_me_log.py`.

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
- Default gate requires exactly RTK fixed quality 4; RTK float quality 5 is rejected. Stale GPS (>3 s) halts motion.

### BMS
Daly SPIM08HP over BLE (`bleak`). Polls SOC / cell voltages / temp / MOSFET status. Auto-reconnect. `is_charging()` feeds charger-inhibit into controller every loop tick. Fail-open after 30 s BLE dropout.

## Known Issues

- **VESC RPM telemetry — RESOLVED 2026-06-11**: bench test (`tools/vesc_rpm_bench.py`, wheels off ground) proved ERPM readback works: commanded +1500, steady-state error −1.0% L / −0.4% R, ~100 Hz per STATUS type, zero parse errors. The old "always 0" observation did not reproduce. Two follow-ups: (1) confirm `vesc_left_rpm`/`vesc_right_rpm` in /api/telemetry go nonzero during the next real drive, then consider re-enabling the velocity PID (`speed_kp/ki/kd` in config.py, currently 0). (2) VESC firmware has a ~1 s command timeout — motors stop if drive commands are not refreshed (confirmed on hardware; the service's 15–30 Hz loop clears it easily, and it acts as a free deadman).
- **OAK USB disconnect — RESOLVED 2026-06-11**: `OakDepthReader._run_pipeline` is now a supervisor loop (`pi_app/hardware/oak_depth.py`). On a fatal device/communication error (USB drop / XLink teardown), the worker thread closes the device defensively, marks health `connected=False`, backs off (2s → 5s → then every 10s, interruptible by `stop()` via `Event.wait`), optionally waits for re-enumeration, then rebuilds the pipeline and resumes — no service restart needed. The depth timestamp is **not** refreshed during the outage, so `get_min_distance()` age keeps growing and the staleness fail-safe still stops autonomous motion. `get_health()` adds `connected`, `reconnect_count`, `last_disconnect_ts`. Covered by `pi_app/tests/test_oak_reconnect.py`.
- **IMU heading drift**: gyro-only integration drifts over time (minutes). GPS COG alignment (in progress) will partially mitigate this at session start.
- **OAK yaw chalk under-report (2026)**: Two layers. (1) Sample identity — no host-dt double-integrate of cached packets (6950e14). (2) Host-side loss — shared vision loop drained IMU at poll rate but kept only latest/bounded tail; sparse snapshots gap-froze and under-reported (~73/81° for physical 90° at scale=1 under load; light pipeline accurate). **Fix**: `ImuYawProducer` integrates every drained packet on the producer (`oak_depth._poll_imu`); enlarged nonblocking host IMU queue (512 **message slots**, multi-second at batch≈1 — not a hard 5.1 s guarantee; multi-packet msgs possible); producer `max_packets_per_drain=512` aligned so a full host backlog is not silently halved; `OakImuReader` applies scale once and preserves unread cum across generation bumps (incl. +turn/−turn back to cum≈0). Freeze only on integrated-counter rewind — never near-zero cum heuristic. **Production defaults (field-validated 2026-07-12): `oak_yaw_rate_source="gyro_y"`, `oak_yaw_rate_scale=1.0` — never restore auto/0.46.** Loss proof is integrated≈received + backlog/gap + drain-batch size — not coalesced=0 and not host-queue occupancy (overwrite not observable via tryGet). See `docs/heading_tuning.md`.

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
