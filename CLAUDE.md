# WALL-E Mini — Codebase Guide

## Architecture Overview

Python control stack running on a Raspberry Pi 5. Entry point is `pi_app/app/main.py` (a tight control loop at ~30 Hz; vision is 15 fps via `OakDetectionConfig.camera_fps`, not 30). All subsystem state is threaded and shared via lightweight dataclasses; the main loop reads latest snapshots and publishes commands.

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
- **Sign convention (canonical: `OakImuReader.read` docstring)**: `heading_deg`
  is compass-style, **clockwise-positive** viewed from above, relative to boot
  orientation, in `[0, 360)`. `yaw_rate_world_dps` (published as `gz_dps`) is
  `d(heading_deg)/dt`, so a **right turn is a positive rate**. Every selectable
  yaw channel is "rotation about the body-DOWN axis, CW-positive": `gyro_y`
  already is (BMI270 +Y points down), the gravity-projected channel is negated
  to match, and `gyro_x`/`gyro_z` are mounting-dependent diagnostics.
  - Fixed 2026-09-19. Before that the reader integrated `-gyro_y`, so a right
    turn read as a left turn (field: 90° right moved heading 241.4 → 152.8).
    `ImuSteeringConfig.invert_output = True` and the negated waypoint ALIGN
    pivot were **compensators** for that, and the pair made the steering D-term
    anti-damping. Both are now corrected; `invert_output` defaults to `False`.
  - The 2026-07-12 chalk validation was magnitude-only, which is why the sign
    survived. The chalk harness now prints a separate **SIGN** PASS/FAIL, and
    `pi_app/tests/test_heading_sign_closed_loop.py` closes the loop around a
    kinematic plant. **If steering turns the wrong way, fix the sign at the
    reader and re-chalk — never add a second inversion downstream.**
- **Active hardware**: OAK-D Lite onboard **BMI270** (gyro + accelerometer; **no magnetometer**).
  - Heading is integrated from gyro and is **relative to startup orientation** — it is not referenced to magnetic north or GPS.
  - `imu_source = "auto"` (default): tries external I2C breakout first, falls back to OAK-D BMI270.
  - External I2C options: ICM-20948 or ISM330DHCX + MMC5983MA combo (if physically present).
  - `imu_use_magnetometer = False` is the current default — magnetometer fusion is disabled.
- **Stationary bias tracking + ZUPT (2026-09-19)**: the gyro bias drifts
  thermally after a good boot calibration — measured `gy_body_dps` went −0.03
  (11:12) → −0.46 → −0.83 → −1.07 → −1.25 (11:49), plateauing near −1.2 deg/s,
  winding the parked heading +0.9 deg/s. `ImuYawProducer` now runs a stationary
  detector (rolling 1 s window; gyro-axis std, accel-norm std, and a per-axis
  max-rate bound), relaxes the bias toward the stationary window mean, and with
  `oak_zupt_enabled` freezes yaw integration outright while still.
  `OakImuReader` then reports `yaw_rate_world_dps = 0` and
  `integrate_status = "zupt"`. The old reader-side `bias_adapt` is **removed**:
  it only ran when |rate| < the 0.3 deg/s NMNI threshold, so it could never
  engage once the drift exceeded that. Thresholds and the log lines to look for
  are in `docs/heading_tuning.md`; `oak_stationary_accel_std_g` still needs one
  tuning pass on the robot.
- IMU feeds `ImuSteeringCompensator` (PID heading-hold, differential byte correction).
- GPS COG heading alignment is **implemented** (software) — one-shot locks IMU heading to true north during a forward, straight manual RTK-fixed run, then freezes the offset for the armed session; field validation pending (`docs/gps_heading_alignment.md`).
- Stationary gyro-bias tracking / ZUPT (2026-09-19) requires a fresh wheels-stopped motion witness (`main.py` pushes `rpm_plausibility.wheels_stopped()` into `OakDepthReader.set_motion_witness`) in addition to a quiet gyro/accel window, because a genuine slow steady turn is indistinguishable from bias by gyro alone — see `docs/heading_tuning.md`.
- `OakImuReader.calibrate_gyro` also derives `yaw_axis_sign` from the accelerometer at boot (BMI270 +Y should read about −1 g on the current mount), so a future upside-down re-mount flips the correction instead of silently inverting the heading — see `docs/heading_tuning.md` "Mount orientation".

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
- Follow Me: **YOLOv8n** blob via `NeuralNetwork` node + host-side NMS (depthai v3). `SpatialDetectionNetwork` / `YoloDetectionNetwork` / `DetectionParser` yield zero detections with the CURRENT blob because it comes from a plain ultralytics ONNX export through blobconverter (decode head kept, no NN Archive `heads` metadata). The supported path is the Luxonis conversion tool → NN Archive, which also unblocks on-device `ObjectTracker` and spatial coordinates. Refer to `docs/oak_d_lite_capability_audit.md` before you re-convert the model.
- Depth EMA filter on person position for smoothing.
- **Second follow-me round (2026-09-19, from the 18:50 run log; analysis in
  `docs/follow_me_run_2026-09-19.md`, reproduce with
  `tools/analyze_follow_me_log.py <log>`)**: direct-pursuit steering was
  retuned and direct pursuit now slows into turns
  (`direct_turn_speed_*`). `detect_min_bbox_width` 0.05 (0.09 rejected the
  operator beyond 4.3 m). The velocity PID targets the forward byte that
  actually reached the motors on the previous tick
  (`update_telemetry(emitted_forward_byte=...)`) so it cannot wind up against
  the persistence decay, accel ramp, obstacle throttle or slew limiter; a
  0.30 s grace hold precedes the persistence decay. The depth corridor needs
  400 px of support and 2-poll persistence (phantom obstacles in low sun).
  `VescConfig.max_erpm` 20000 (was 15000; 1.6 m/s top speed in every mode).
  Steering gains from this round were wrong (see round 3). Width filter,
  corridor, `max_erpm` and the emitted-byte speed loop ran on the
  2026-09-20 17:11 and 18:09 validations.
- **Third follow-me round (2026-09-20 14:58, `docs/follow_me_run_2026-09-20.md`)**:
  the 09-19 "0.22 deg/s per L-R byte" yaw gain was WRONG (64 % of logged
  `imu.yaw_rate_dps` samples were 0.0: the duplicate-read freshness bound in
  `oak_imu.py` was 50 ms while the sample age under follow-me load is
  90-290 ms; now 0.20 s). From the heading derivative the plant is
  **0.65-0.71 deg/s per byte**. The 09-19 gains made the robot weave, so:
  `pid_lateral_kp` 0.5, `max_steer_offset_byte` 32,
  `direct_mode_max_steer_byte` 32, `steer_slew_per_tick` 0.25. **Never fit a
  plant from `yaw_rate_dps`; use the heading derivative (the analyzer does).**
  The same run hard-stopped five times because the tracklet id on a lone
  operator churned (narrow box + 0.12-0.35 s detection gaps + ego yaw => IoU
  0) and the sticky lock read each new confirmed id as a different person.
  Fix: centre-distance fallback in `TrackletTracker` (`tracklet_center_gate_*`)
  and a single-candidate rebind in `TargetTracker._find_committed` (exactly
  one candidate, <= 0.5 s, base depth gate). **Validated** first 50 s of
  the 15:49 run (100 % fresh, no hard stops, no weave).
- **Fourth follow-me round (2026-09-20 15:49 near-run-over; validated 17:11)**:
  identity gates decide whom to follow; they never hide a closer range from
  the speed command. Config/telemetry:
  `nearest_person_speed_limit_enabled`, `nearest_person_margin_m`,
  `close_unknown_bbox_height`, `target_depth_coast_max_s`. Sampler
  `depth_status`: `ok` / `no_support` / `ambiguous` / `far_veto` /
  `no_frame` / `height_veto` (off). `person_depth_*` knobs in
  `FollowMeConfig`. Log: `arm_20260920_154844.log` / `arm_20260920_171057.log`.
- **Fifth follow-me round (vision latency, validated 18:09
  `arm_20260920_180915.log`)**: `OakDetectionConfig.camera_fps = 15.0` is
  **load-bearing — NEVER remove it without re-measuring `oak.det_latency_s`**.
  Also: `nn_input_queue_size = 1` (alone did not fix latency),
  `vision_deadline_sleep`, `poll_detections_first`, MediaPipe Hands is not
  run in RC-started FOLLOW_ME. Idle A/B: read `oak.det_latency_s` from
  `logs/latest.log` (no field walk). Under load p50 0.20 s at 15 fps
  (was 0.80 s at 8 fps). Motor→yaw lag 0.47 → 0.12 s was heading latency
  from a blocked vision thread, not the drivetrain. Steer gains left at
  kp 0.5 / cap 32.
- **Arms-up bench twitch (2026-09-22, `docs/NEXT_SESSION.md` section 2)**:
  MediaPipe Pose on the person crop (`pi_app/hardware/pose_worker.py`,
  `pi_app/control/arms_up.py`) gives one 22-byte, 0.25 s reverse pulse.
  Only the volatile latch `POST /api/arms_up/twitch_test` enables it
  (local-only, armed + MANUAL, 3 pulses / 5 min). Pose runs only while
  that latch is on, so measure latency in that state. Do NOT reuse the
  latch for the continuous back-up feature: it needs its own latch and
  its own Grok safety review.
- The onboard IMU is a **BMI270**: raw/calibrated accel + gyro only. `ROTATION_VECTOR` / `GAME_ROTATION_VECTOR` / magnetometer are BNO08x-only and return nothing on this device. Do not try them. Refer to `docs/oak_d_lite_capability_audit.md`.

### Waypoint Navigation
State machine in `pi_app/control/waypoint_nav.py`:
- **ALIGN**: pivot in place until heading error < `align_threshold_deg` (default 12°). Yaw sign: positive yaw_cmd for positive (right) heading error. The pivot is proportional to the error (max `pivot_yaw_cmd` 0.35 at ≥90°, floor `pivot_yaw_min` 0.18); a fixed 0.5 pivot overshot 25–30° and oscillated with DRIVE (2026-09-19).
- **DRIVE**: forward at cruise speed with PID steering; falls back to ALIGN if error exceeds `recovery_threshold_deg`.
- **ARRIVE**: within `arrival_radius_m` of target; zeroes commands and advances waypoint.
- `compute()` returns `(v_cmd, yaw_cmd, state)` as floats; caller's mixer converts to motor bytes.
- Default gate requires exactly RTK fixed quality 4; RTK float quality 5 is rejected. Stale GPS (>3 s) halts motion.

### BMS
Daly SPIM08HP over BLE (`bleak`). Polls SOC / cell voltages / temp / MOSFET status. Auto-reconnect. `is_charging()` feeds charger-inhibit into controller every loop tick. Fail-open after 30 s BLE dropout.

## Known Issues

- **VESC RPM telemetry — RESOLVED 2026-06-11**: bench test (`tools/vesc_rpm_bench.py`, wheels off ground) proved ERPM readback works: commanded +1500, steady-state error −1.0% L / −0.4% R, ~100 Hz per STATUS type, zero parse errors. The old "always 0" observation did not reproduce. Velocity PID is on (`speed_kp` 0.6, `speed_ki` 0.15, `speed_kd` 0.0); 2026-09-20 follow-me logs show nonzero `left_rpm`/`right_rpm`. VESC firmware has a ~1 s command timeout — motors stop if drive commands are not refreshed (confirmed on hardware; the service's ~30 Hz control loop clears it easily, and it acts as a free deadman).
- **OAK USB disconnect — RESOLVED 2026-06-11**: `OakDepthReader._run_pipeline` is now a supervisor loop (`pi_app/hardware/oak_depth.py`). On a fatal device/communication error (USB drop / XLink teardown), the worker thread closes the device defensively, marks health `connected=False`, backs off (2s → 5s → then every 10s, interruptible by `stop()` via `Event.wait`), optionally waits for re-enumeration, then rebuilds the pipeline and resumes — no service restart needed. The depth timestamp is **not** refreshed during the outage, so `get_min_distance()` age keeps growing and the staleness fail-safe still stops autonomous motion. `get_health()` adds `connected`, `reconnect_count`, `last_disconnect_ts`. Covered by `pi_app/tests/test_oak_reconnect.py`.
- **IMU heading drift**: gyro-only integration drifts over time (minutes). GPS COG alignment (in progress) will partially mitigate this at session start.
- **OAK yaw chalk under-report (2026)**: Two layers. (1) Sample identity — no host-dt double-integrate of cached packets (6950e14). (2) Host-side loss — shared vision loop drained IMU at poll rate but kept only latest/bounded tail; sparse snapshots gap-froze and under-reported (~73/81° for physical 90° at scale=1 under load; light pipeline accurate). **Fix**: `ImuYawProducer` integrates every drained packet on the producer (`oak_depth._poll_imu`); enlarged nonblocking host IMU queue (512 **message slots**, multi-second at batch≈1 — not a hard 5.1 s guarantee; multi-packet msgs possible); producer `max_packets_per_drain=512` aligned so a full host backlog is not silently halved; `OakImuReader` applies scale once and preserves unread cum across generation bumps (incl. +turn/−turn back to cum≈0). Freeze only on integrated-counter rewind — never near-zero cum heuristic. **Production defaults (field-validated 2026-07-12): `oak_yaw_rate_source="gyro_y"`, `oak_yaw_rate_scale=1.0` — never restore auto/0.46.** Loss proof is integrated≈received + backlog/gap + drain-batch size — not coalesced=0 and not host-queue occupancy (overwrite not observable via tryGet). See `docs/heading_tuning.md`.
- **Hand gestures do not work as deployed**: FIVE is only honoured in
  gesture phase `ACTIVE` (RC-started FOLLOW_ME never enters it); a 3-4-3
  start is cancelled one tick later by RC ch4 low; MediaPipe Hands on the
  640×480 stream sees ~22 px at 1.5 m vs 40-60 px needed. Fix, do not
  remove; RC stays the authority for now. See
  `docs/gesture_review_2026-09-20.md`.
- **Video recording is suppressed whenever gestures are configured**
  (`oak_depth.py`: the H.265 encoder is skipped when `GestureConfig.enabled`
  and `pi_app/models/hand/` exists; journal line
  `OAK recorder: no recording queues available`).

## Config

`config.py` (repo root) — one frozen `Config` dataclass with nested sub-configs. Edit this to change thresholds, enable/disable subsystems, set BMS MAC, VESC CAN IDs, etc.

## Logging

`logs/run_*.log` / `arm_*.log` mix four JSON line shapes (per-tick, 1 Hz "slow", session header, event) — see `docs/logging_troubleshooting.md` for the field-by-field format.

## Running Tests

```bash
cd /home/pi/wall_e-Mini
python3 -m unittest discover -s pi_app/tests -p "test_*.py"
```

## Branches

- `research/follow-me-trail-strategies`: research notes (not merged; 1 commit ahead of main).
- Remote merged branches (`origin/codex/*`, `origin/feature/property-map`, `origin/feature/trail-follow-pure-pursuit`) can be pruned with `git remote prune origin`.
