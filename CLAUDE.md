# WALL-E Mini — Codebase Guide

## Field test handoff (2026-11-03)

The east-field test plan is `docs/field_test_2026-11-03.md`. Read it before
changing motion, waypoint nav, the web UI, or Wi-Fi behavior for that test.
The Pi was offline on the night of 2026-11-02, so the plan was not run and
the test page was not built yet. The reachability halt in that plan is a
different thing from the reconnect watchdog in `docs/wifi_watchdog.md`.
Kevin, 2026-11-10: the plan is still wanted, but not now.

## Safety principle (Kevin, 2026-09-27) — apply to every motion change

- Human input can carry a little extra risk. Autonomous input must not cause human harm.
- Reason: in MANUAL (RC sticks or phone) the operator stops the robot by letting go of the stick. In FOLLOW_ME and WAYPOINT_NAV the operator must take the remote out of a pocket and disarm, which takes much longer.
- Result: a MANUAL relaxation is acceptable (the creep floor, the MANUAL obstacle curve for netting over the lens). FOLLOW_ME and WAYPOINT_NAV keep every protection.
- The person/animal hard stop (0.8 m) stays absolute in every mode.
- Ruled 2026-11-08: the 0.8 m stop zeroes the forward part of the command only. A pivot in place passes through it, in FOLLOW_ME too (log 14:27:40: steer −32 for about 1 s with Kevin 0.3–0.9 m away). Kevin judged that pivot fine. Do not freeze the pivot during the stop without his word.
- Applied 2026-09-27: Kevin removed the person/animal cap on the MANUAL obstacle curve. It was more cautious than this principle, and it turned the netting relief off whenever the camera saw him near the robot but could not range him through the mesh. Do not add it back without his word.
- Person stop hold (2026-11-10, `c999b0a`, `pi_app/control/person_stop_hold.py`): in FOLLOW_ME and WAYPOINT_NAV the 0.8 m stop now has memory. Before, one bad frame at close range released it (2026-11-08 14:27:40, 0.66 s). After a trigger the hold keeps the forward part at zero and follows the one object that caused the stop. It releases only when that same object is seen beyond 1.0 m (3 polls, then 5 polls of evidence). The corridor is never release evidence there. If the object is lost for more than 0.6 s, or a second person or animal is close or in its place, the hold becomes operator-only: it ends with the arm switch off, with FOLLOW_ME off and on, or with stop and GO on the waypoint page. A hold that is active at a change to MANUAL stays until the sticks are at neutral for 0.25 s. MANUAL has no new hold (`person_stop_hold_in_manual` False): with the netting over the lens it would not release, and that is the fault of the cap above. Do not set it to True, and do not add a timer release or a corridor release, without Kevin's word and a Grok safety review; five reviews were necessary (`docs/status_audit_2026-11-10.md`, section 9.4). The pivot still passes. Journal: `Person stop HOLD engaged / released / operator-only (reason)`; log: `obstacle.person_stop_hold`. Not tested with a person on the robot yet.

## Architecture Overview

Python control stack running on a Raspberry Pi 5. Entry point is `pi_app/app/main.py` (a tight control loop at ~30 Hz; vision is 15 fps via `OakDetectionConfig.camera_fps`, not 30). All subsystem state is threaded and shared via lightweight dataclasses; the main loop reads latest snapshots and publishes commands.

```
pi_app/
  app/main.py          # Main loop, subsystem wiring, SSE web server (port 8080)
  control/             # Logic: controller, safety, follow_me, waypoint_nav, obstacle_avoidance, imu_steering
  hardware/            # Drivers: vesc, arduino_rc, oak_depth, oak_imu, imu_reader, bms, rtk_gps
  cli/                 # Calibration and test harnesses
  tests/               # Unit tests
  web/                 # Flask UIs: viewer, teleop, waypoint nav, property map
config.py              # Single Config dataclass (root of repo)
bin/                   # root services: UPS watcher, I2C bus guard, Wi-Fi watchdog, auto-deploy
tools/                 # log analyzers, replay tools, bench scripts
```

## Key Subsystems

### Motor Output
VESC over CAN (`can0`) is the only drive path. The Arduino only streams the RC channels over USB; its old Model X motor-driver board is removed (Kevin, 2026-11-13), so the Model X fallback code drives nothing. Wiring and open checks: `docs/rc_arduino_gps_wiring.md`. All motor commands are "bytes" (0–254, centre 126 = stop; `pi_app/control/mapping.py`). Left/right are issued separately via `VescCanDriver`. A command-space mixer converts `(v_cmd, yaw_cmd)` floats in `[-1, 1]` into left/right bytes.

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
  - `imu_source = "oak_d"` in `config.py` (pinned 2026-09-19). `"auto"` tries an external I2C breakout first and must not be restored: that path publishes the opposite yaw-rate sign.
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
- **MANUAL heading hold works in the raw IMU frame (2026-09-27).** Never re-apply a heading target every tick in MANUAL: the straight latch survives stick pulses, so such a target stays behind the operator's turns. The GPS-locked true-frame retarget did exactly that and dragged the robot back toward old headings at up to 35 bytes ("it wants to go left"). The compensator re-captures on each return to neutral, and only a MANUAL tick counts as "already straight" across a mode switch (`test_manual_heading_hold_retarget.py`).
- GPS COG heading alignment is **implemented** (software). `GpsHeadingAligner.update_cog()` locks IMU heading to true north during a forward MANUAL run at RTK fixed (the path does not have to be perfectly straight). The offset is kept for the life of the process, including disarm; a service restart clears it. Trust `Controller._on_armed_session_ended` and the "Lock persistence" section of `docs/gps_heading_alignment.md`. Field validation of the lock is still pending.
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
- **Sixth follow-me round (2026-09-26, `arm_20260926_125554.log`,
  `arm_20260926_130713.log`)**: the robot sat at the old 110-byte cap
  (1.38 m/s) behind a 1.55 m/s walk and lost the operator on range alone
  at 6.1 m. Now `max_follow_speed_byte` 128 (byte 254 = `max_erpm` =
  1.61 m/s, the drivetrain limit; ~1.5 m/s on a half pack),
  `max_speed_error_m` 1.75 (keeps the 73 bytes/m approach slope),
  `max_distance_m` 8.0 (the 0.05 width floor binds first near 7-7.5 m),
  `follow_distance_m` 1.2 (Kevin's 4 ft stop; zero forward inside 1.4 m).
  False near matches on the iron gate put 0.7-0.8 m phantoms in the depth
  corridor (slow, jerky approach). Fix: a near pixel needs 16 near
  neighbours within one 100 mm depth bin in its 13x13 window
  (`corridor_speckle_*`); `person_mask_depth_margin_m` 0.5 masks only the
  person's own pixels (the old whole-box mask hid anything between robot
  and person). Tuned on 45 raw frames + dropout (150); a blob-size filter
  lost woven wire fence and was rejected. The recorder's colourized depth cannot tell a
  false match from a hole; capture raw frames with the local-only
  `POST /api/debug/depth_snapshot` (`logs/depth_snapshots/*.npz`). See
  `docs/NEXT_SESSION.md` section 4, item 5.
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
- **ARRIVE**: within `arrival_radius_m` of the last waypoint; zeroes commands. An intermediate arrival advances the index and goes straight to ALIGN for the next waypoint, with no stop.
- `compute()` returns `(v_cmd, yaw_cmd, state)` as floats; caller's mixer converts to motor bytes.
- Default gate requires exactly RTK fixed quality 4; RTK float quality 5 is rejected. Stale GPS (>3 s) halts motion.
- Start gate and run guard (2026-11-10): `Controller.waypoint_nav_activation_blocked` refuses a start when there is no GPS reading (`gps_state_unknown`), when the reading is older than the stale timeout (`gps_stale`), when the detection stream is stale (`vision_stale`), when there is no heading source, when the heading is older than `heading_stale_s` (0.5 s), and when the heading lock is absent. During a run, `_leave_waypoint_nav_if_unsafe` checks the heading and the detection stream on each tick, and it stops one ALIGN episode after `align_timeout_s` (10 s). An exit sets MANUAL and ramps to the sticks through the slew limiter.
- API limits (2026-11-10): `/api/nav/start` and `/api/nav/go` clamp `cruise_speed` to 10–80 and `arrival_radius` to 0.3–3.0 (the slider ranges, one definition in `pi_app/web/waypoint_nav_ui.py`). A speed of 0 or less gets HTTP 400. The routes ask the gate before they write, and they restore the mission from `snapshot_mission` if the activation fails, so a refused start does not change a mission that is in progress.

### BMS
Daly SPIM08HP over BLE (`bleak`). Polls SOC / cell voltages / temp / MOSFET status. Auto-reconnect. `is_charging()` feeds charger-inhibit into controller every loop tick. Fail-open after 30 s BLE dropout.

## Known Issues

- **Status audit (2026-11-10)**: `docs/status_audit_2026-11-10.md` lists the safety gaps in priority order. Items 1 to 5 and the IMU stall were corrected and deployed the same day (section 9 of that document); open: the pack-low latch that cannot trip (39.0 V in `config.py` against the 40.0 V BMS pack trip), and the stale documents. Read it before you plan work.
- **Trail following is OFF (2026-11-10, Kevin's decision)**: `FollowMeConfig.trail_follow_enabled = False`. `GpsOdometry` keeps a math-angle heading, but its `camera_to_world` and pure pursuit use the clockwise dead-reckoning frame, so trail pursuit steers the wrong way after a pivot, and its heading is the boot-relative gyro (`docs/status_audit_2026-11-10.md`, section 8). That frame fed trail pursuit beyond 5.2 m, blind trail pursuit after a lost target and the search-pivot direction; none of them ran in an armed log up to 2026-11-08. With the switch off, the trail, the two odometry objects and pure pursuit are not built. FOLLOW_ME is direct pursuit only. A lost target gives a pivot with zero forward motion toward the last camera position until 3 s after the last valid detection, then a stop; if the person was near the centre of the frame (`|_last_target_x|` ≤ 0.05), the robot stays still for that time. Repaired on 2026-11-10 as inert code (`cd9086e`, `be768cc`, `bf23d47`): one frame contract in `pi_app/control/odometry.py` (+x is heading 0, +y is 90 degrees clockwise, heading clockwise-positive) for the two odometry classes and each user, tests in `test_odometry_frame_contract.py`, and `trail_odometry_source` (`"dead_reckon"` by default; GPS odometry stays selectable). The switch is still False. Do not set it to True before the field checks in `tools/odometry_analysis/README.md` (a stall check for dead reckoning, a straight run on a tape, pursuit beyond 5.2 m and a lost target in an empty area), a Grok review and Kevin's approval. Also from that section: the VESC eRPM speed reads 20 to 25 percent above GPS ground speed on straight segments (slip or scale, not separated); the byte scale `trail_speed_scale_mps_per_byte` 0.0100 agrees with GPS.
- **The IMU stream can stop without a fault (found 2026-11-10)**: on 2026-11-06 at 19:34:37 the OAK IMU stream stopped in a healthy camera session, while depth and detections continued. The heading stayed frozen for 38.8 h (the two drives of 2026-11-07: `integrate_status` `stale`, heading 0.000) until a colour-stall session rebuild on 2026-11-08 at 10:21:56. The journal had one line (`OAK IMU samples stale ... heading frozen`). The cause in the camera is not known. Corrected on 2026-11-10 (`185f239`, `67b3f04`, `07cbc74`): the session loop ends the OAK session when no IMU message arrives for `imu_stall_restart_s` (10 s) with fresh depth, after a 10 s warm-up, and the supervisor rebuilds it. The IMU watchdog and the colour watchdog share one budget (3 restarts in 15 min); then `imu_stall_fault` latches until an IMU message arrives. The watchdog is on only when `OakImuReader` is the heading source, and it does not restart for an IMU that gave no message since the start. `ImuSteeringCompensator.update` returns no correction while the reader reports `stale`, and it captures the target one time on recovery (never apply a target again on later ticks). Alarm: `camera_health.imu_age_s` / `imu_stale` / `imu_watchdog` on `/api/telemetry`, a journal WARNING each minute, `IMU STALE` in the status line, JSON events `oak_imu_stall*` and `oak_imu_recovered`. Not verified on a real stall: refer to `docs/status_audit_2026-11-10.md`, section 9.3. WAYPOINT_NAV refuses a stale heading since 2026-11-10.
- **RC and phone command age use the monotonic clock (2026-11-10)**: `RCInputs.last_update_mono_s` replaced `last_update_epoch_s` (the Pi's wall clock steps at boot, and a backward step made a dead RC link look fresh). A stamp of 0.0 is "no frame yet". The phone override file carries `last_update_mono_s` and the kernel boot ID; `pi_app/app/phone_override.py` accepts a command only if its age is 0 to 0.6 s and it comes from this boot. Arming with the transmitter on was not tested after this change; test it before the next drive.
- **Wheel ground scale (2026-11-10, `962454a` + `1e3a26f`)**: the eRPM-to-m/s conversion read 1.23 times the GPS ground speed (a constant scale error, not slip; the data cannot say if the radius, the gearing or the pole count is wrong). `VescConfig.wheel_ground_scale = 0.815` corrects the speed VALUES only: the VESC runs the same eRPM for the same byte. Byte 254 = `max_erpm` 20000 is approximately 1.31 m/s on the ground. Each "1.61 m/s" and "1.38 m/s" in this file and in the older documents is in the old units (multiply by 0.815). The follow-me speed loop uses one scale for the target and the feedback; `speed_loop_mps_per_byte` is 0.010232325, and the two limits in m/s were scaled (`speed_pid_max_correction_mps` 0.163, `speed_integral_limit` 0.815), so the motor bytes are the same as before (`test_wheel_ground_scale.py::TestIdenticalMotorBytes`). Do not change one of these values alone. Two checks by hand settle the constant: `docs/wheel_speed_scale_2026-11-10.md`, section 8.
- **Slew after an armed stop release (2026-11-10, `cce9b06`)**: after the charger inhibit, the pack-low latch or the person stop released while armed, a held stick went to the motors in one tick (2026-11-05 09:31:19: neutral to full reverse). `_hold_slew_at_neutral()` now makes the release ramp from neutral; a stop is still immediate. Kevin's rulings the same day: the first command after a re-arm stays as it is (`snap_first_command`, "unnecessary" to ramp), and there is no charger latch and no "charger unplugged" button ("stupid idea"); the charger inhibit works only while the BMS Bluetooth link is awake (`docs/charger_inhibit_review_2026-11-10.md`). Do not propose these two again.
- **VESC RPM telemetry — RESOLVED 2026-06-11**: bench test (`tools/vesc_rpm_bench.py`, wheels off ground) proved ERPM readback works: commanded +1500, steady-state error −1.0% L / −0.4% R, ~100 Hz per STATUS type, zero parse errors. The old "always 0" observation did not reproduce. Velocity PID is on (`speed_kp` 0.6, `speed_ki` 0.15, `speed_kd` 0.0); 2026-09-20 follow-me logs show nonzero `left_rpm`/`right_rpm`. VESC firmware has a ~1 s command timeout — motors stop if drive commands are not refreshed (confirmed on hardware; the service's ~30 Hz control loop clears it easily, and it acts as a free deadman).
- **Stale detection stream must not drive follow-me (2026-09-25)**: CAM_A stalled after 23 h. `get_person_detections()` kept the last person for 40 min while `detections_stale` was already true, and FOLLOW_ME drove a left circle for 44 s. The stored list is intentionally not cleared — a stale close person must still force the person/animal stop. Follow-me reads `OakDepthReader.get_detection_freshness()` and drops out when `age_s > OakDetectionConfig.vision_stale_s` (1.5 s). Do not "fix" this by returning `[]` from `get_person_detections()`. A repeated NN sequence number does not refresh the clock. Each new device session clears the last sequence (`_reset_det_sequence_locked()` at session start), so a reconnect's restarted counter refreshes the clock on its first packet. The stale exit ramps to the sticks through the slew limiter (never bypass it: a held stick would jump). Auto-restart (2026-09-25): the OAK session loop ends itself when no fresh NN packet has arrived for `OakDetectionConfig.color_stall_restart_s` (5 s, after a 10 s warm-up), and the supervisor rebuilds it. After 3 watchdog restarts in 15 min it latches `color_stall_fault` (health) and stops restarting; a fresh packet clears the latch. Journal: `OAK colour stream stalled: ...` / `... auto-restart disabled ...`.
- **OAK vision worker hang (2026-10-04 13:41 -> 2026-10-08 17:36)**: a USB hub glitch (the OAK and the Arduino serial dropped in the same second) made the colour-stall watchdog's third restart call `pipeline.stop()`, and depthai's teardown never returned. The worker thread sat in C++ for 4 days with `pipeline_running` True, `loop_age_s` growing, `depth_fps` 0; the supervisor cannot run from inside a hung thread, so nothing restarted it. Depth was 4 days stale, so MANUAL drove at `manual_stale_throttle_scale` (0.10; "extremely slow") and FOLLOW_ME was refused; the only journal sign was `OAK(2.50m scl=0.1)`. Fix: `get_health()` reports `vision_hung` (no session-loop or supervisor heartbeat for `OakDetectionConfig.vision_hang_s` 30 s, or the thread died, while not stopping; a device-absent backoff keeps the supervisor heartbeat fresh and is NOT a hang). `pi_app/app/vision_hang.py` in the main loop prints `OAK vision worker HUNG ...` every minute, writes `oak_vision_hung` / `oak_vision_hung_restart` / `oak_vision_hung_restart_denied` / `oak_vision_recovered` events to the JSON log, and after `vision_hang_restart_s` (60 s) hung ends the loop with exit code 3 via `os._exit` so systemd (`Restart=on-failure`, 5 s) starts a fresh process. **Restart gate** (Grok review): not `cmd.is_armed`, which also goes False on a 1 s RC dropout with the switch still up. With a fresh RC link the raw ch3 level must read OFF; a dropout shorter than `vision_hang_rc_off_s` (30 s) blocks; longer silence or no RC ever allows (transmitter off, nobody can drive). The gate must hold continuously for `vision_hang_restart_settle_s` (10 s). **Budget**: `vision_hang_restart_budget` 3 per `vision_hang_restart_window_s` (60 min) in `logs/vision_hang_restarts.json`; past that it logs only ("restart the service by hand"). Once the exit is chosen SIGTERM is ignored so an auto-deploy restart during the cleanup cannot skip `os._exit`. An armed robot is never restarted under the operator (MANUAL keeps the stale floor; the autonomous modes are already halted by the stale depth). This is the same depthai teardown defect as the 2026-09-26 abort crash loop, in its hang form. **depthai 3.6.1 since 2026-11-05** (`docs/depthai_upgrade.md`): a camera unplug no longer aborts wall-e, and the session reconnects by itself. 3.6.1 reverses Y and Z in the RAW IMU reports, so `oak_depth.imu_report_names()` requests UNCALIBRATED from 3.6.1 (never RAW); the startup line `OAK device: depthai ..., USB ..., IMU reports ...` shows the choice. `DEPTHAI_AUTOCALIBRATION=OFF` by default. Triage: `camera_health.vision_hung` / `vision_worker_age_s` on `/api/telemetry`; recovery before the fix was a disarmed `sudo systemctl restart wall-e`.
- **Arming (2026-09-25)**: ch3 arming is level-based, with a re-arm latch in `SafetyState.rearm_requires_switch_cycle`. `force_disarm()` (auto-disarm after 600 s armed and idle in MANUAL) and the startup interlock (`Controller(startup_interlock=True)` from `main.py`) set it; only ch3 seen at or below the low threshold clears it. The RC-stale rebuild keeps it. Unit tests build the Controller without the interlock. RC arm-channel fail-safe (2026-11-05, after the HotRC CH3/CH4 hybrid-mode fault): ch3 must be held high (>= 1800 µs on every tick for `SafetyParams.arm_high_persist_s`, 0.2 s) to arm. While armed and ch3 is not low, no held-high tick for longer than `SafetyParams.arm_midband_disarm_s` (0.5 s) disarms, sets the latch and emits `ARM_MIDBAND_DISARM` (journal `ARM SWITCH MID-BAND DISARM`, JSON event `arm_midband_disarm`); short button bursts never count as held. All `update_safety` timing (debounce and both rules) runs on `time.monotonic()` (`now_mono_s`), not on the wall clock. A ch3-low disarm and the ch5 e-stop keep their old timing, and a ch4 edge before the arm is granted does not start FOLLOW_ME.
- **I2C bus 1 lock-up (2026-09-25 21:36)**: kernel `SDA stuck at low`; the RTK GPS (0x20) and the UPS watcher (then `upsplus-power.service`, now `walle-ups.service`) share the bus and both went dark for 17 h until a power cycle. `i2c-bus-guard.service` (root, `bin/i2c_bus_guard.py`, `docs/i2c_bus_guard.md`) now unlocks it (UPS status stale + SDA low/SCL high on 10 samples → unbind, clock SCL open-drain, STOP, restore `a3 pu`, rebind, restart `walle-ups`) and warns whenever the UPS status is stale. It does not restart on auto-deploy; re-run `sudo bash bin/install_i2c_bus_guard.sh` after changing it.
- **UPS: Waveshare UPS HAT (B) since 2026-11-03 (UPSPlus retired; its daemon, tests and docs deleted)**: 2S 18650, INA219 at 0x42 on a **10 mΩ** shunt in the battery leg (the schematic says 0.1R; the first unplug test read −52 mA for a 0.6 A Pi and held, so never trust that label), no MCU and no charger IC. The 8.4 V jack is wired straight to the pack node, so the robot buck's setpoint IS the charge limit (Kevin: 7.5 V since 2026-11-04, first 7.6 V; the bench brick is 8.4 V). `bin/walle_ups.py` (`walle-ups.service`, root; `docs/ups_hat_b.md` section 4) infers mains from the battery current I and pack voltage V against the setpoint S (`--supply-v`, now 7.5 in both unit files; it warns on a 0.2 V float mismatch). Rules since 2026-11-04 (d6f41d9): present I ≥ −80 mA; discharging (I ≤ −250 mA) is `absent` only below S − 0.30 V, `above_supply` above S + 0.15 V, and `discharge` (HOLD) in between, because a pack above the supply carries the Pi while the supply idles (field: 1.1 Ω cells at 8.27 V on a 7.5 V supply read 7.59-7.70 V at −0.5 A and the old rules powered the Pi off four times). The 10 s edge rule (`--edge-s`) is OFF: healthy cells turn supply droop into the same signature; measure droop before enabling it. Low-battery halts: 7.0 V/10 s, 6.4 V/one sample. 5 s of counted absent halts, killing `wall-e` first (its camera teardown can take 90 s and the alarm must not fire mid-shutdown), arming the RTC alarm (+90 s) and writing `chain.json`. `walle-ups-wakecheck.service` (before `basic.target`) powers off again on a 75 % majority: ≤ 7.0 V whatever the current (+60 s, or +600 s at ≤ 6.4 V; never capped), absent (+60 s), or band samples inside any non-simulated chain that discharge at ≥ 400 mA and are below S − 0.03 V (one-sided since 2026-11-06) (+60 s; a supply back below the pack pins V at S and shares the load); caps a chain at 24 power-offs (both modes count), after which run mode grants a 10 min grace once (mains-loss halts off, low-battery on); fails OPEN on any doubt. Run mode arms +90 s after every halt, ≥ +300 s if wall-e is still stopping, and retries/forces a poweroff that did not happen (e3bb5a2, Opus review after the third Grok pass timed out). A shutdown hook (`bin/walle-ups-shutdown-alarm`, installed in `/usr/lib/systemd/system-shutdown/`) re-arms the RTC alarm in the last second before power-off from `/run/walle-ups/arm` (ee52b5b; a hung `xdg-document-portal` once held a shutdown 91 s against the 90 s alarm); stop timeouts are capped at 30 s (system) and 10 s (user); the Pi boots to console with autologin since 2026-11-04 (no desktop; Pi Connect shell kept). Live 7.5 V pull 2026-11-04 13:23: held 17 min while the pack sagged from above the supply, clean hard halt at 7.18 V, 18 poll wakes on band readings inside the hard chain, boot within 75 s of the replug. The hard line has 0.10 V of hysteresis while a count runs (the pack dithered at 7.20 V for 6 min in that test). All timers use the boot clock: the RTC resets to 0 when 5 V drops and NTP then jumps the wall clock. Bootloader EEPROM `POWER_OFF_ON_HALT=1 WAKE_ON_GPIO=0` (set 2026-11-03): halt = PMIC standby (~3 mA), the HAT keeps 5 V so the RTC runs without a coin cell. Bench 2026-11-03: poweroff 8 s, RTC wake to ssh 108 s. Real unplug 2026-11-04: dark 9.5 s after the pull; each poll wake is on ~16 s (12 s firmware + 4 s Linux), one cycle ~75 s; the wake check powered off at 11/11 absent and continued at 11/11 present; replug to ssh ≤ ~90 s, no button needed. The HAT cannot switch its own output off and the Pi 5 has no GPIO wake, so mains returning is only seen by the next alarm. Install/update by hand: `sudo bash bin/install_walle_ups.sh` (auto-deploy does not restart it; when the supply changes, change `--supply-v` in BOTH unit files first). Status `/tmp/ups_status.json` (`ts` = last good I2C read, the bus guard's witness); halt override `/var/lib/walle-ups/nohalt` (loud). Each poll wake is ~16 s on at 0.5-0.9 A, about a fifth of the time. The bench cells have ~1 Ω internal resistance (8.35 → 7.5 V under 0.6 A, back to 8.35 V at +0.2 A), so with them the pack hits the HY2120 cutoff within the hour at 60 s polls; after the cutoff the Pi boots on power-up when mains returns. Bench test `sudo bin/walle_ups.py simulate-loss --yes` runs the real halt/alarm/poweroff chain with mains connected. New 18650 cells went in on 2026-11-06 and stay (Kevin, 2026-11-10), but they do not hold: on 2026-11-08 at 19:33 Kevin switched main power off, the pack fell at once from the 7.5 V float to 6.85 V at −0.84 A (about 0.8 Ω), the halt was clean, and the pack was flat 9 minutes later after 6 poll wakes. After a flat pack the Pi does not start by itself when main power returns: on 2026-11-10 it stayed in standby from about 08:30 until Kevin pressed its power button at 10:30. Refer to `docs/ups_hat_b.md`.
- **ch5 e-stop powers the Pi off (kept 2026-11-03)**: `EMERGENCY_TRIGGERED` stops the motors, latches DISARMED and schedules `sudo shutdown -h now` 5 s later (`controller.py`). The robot cannot be driven home after an e-stop; recovery is a Pi power-on (power button, or the UPS switch off and on). The RTC alarm is armed only for mains-loss halts, not for this one.
- **Wi-Fi no-secrets block (2026-09-27)**: at the edge of range NetworkManager 1.42 logs `failed (reason 'no-secrets')` and blocks the profile until reboot, even back in range (the Pi was up and off the network for 36 min). The reconnect watchdog (`bin/wifi_watchdog.py`, `docs/wifi_watchdog.md`) is on main (f4e209f, df1187d) and installed as `wifi-watchdog.service` (active 2026-11-03); `preconfigured` auth-retries is −1 now. When the robot vanishes, check `journalctl -b -1 -u NetworkManager | grep no-secrets` before blaming power.
- **Netting mute (2026-10-08)**: electric-fence mesh netting loaded on the robot hangs over the lens and is a SOLID wall at 0.40-0.50 m to stereo (60 raw frames: 97-98 % of valid corridor pixels at 0.35-0.65 m; replay with `tools/replay_depth_snapshots.py`), so the MANUAL curve held Kevin at 0.3-0.4 throttle for 81 m. Depth cannot tell it from a wall, and an automatic "it moves with us" latch was rejected in review (a missed close person, wheel slip in front of a wall, or anything being pushed satisfies the same test). The mute is EXPLICIT: the phone button "NETTING LOADED" (`POST /api/manual/netting_mute`), `pi_app/control/netting_mute.py`. The tap is accepted only armed, in MANUAL, with the netting already in view (corridor distance <= `netting_mute_max_distance_m` 0.55 m, the drive data's 0.40-0.50 m band, and near/support share >= 0.85); there is no waiting state. While engaged it ignores the corridor as long as the field holds (share >= 0.70); a finite reading beyond 0.55 m drops it on that tick, a dropout gets a 0.5 s hold, and it drops on any person or any stop-tier or slow-tier detection (the stop tier holds cat, dog, horse, sheep and cow; the slow tier holds no animals; birds do not count), on disarm, on leaving MANUAL, after 15 min, or on a second tap; a new tap is needed after any drop. The web thread only queues the request; `process()` is the single writer. The YOLO 0.8 m stop (distance 0.0 channel) and a stale depth are never muted; FOLLOW_ME / WAYPOINT_NAV never use it. Accepted residual: a real wall inside 0.55 m at the moment of the tap is indistinguishable from the mesh. Do not add an automatic engage on depth + odometry; an appearance-based mesh recogniser could engage it later, with motion only as a veto.
- **MANUAL obstacle curve and stick expo (2026-09-27)**: MANUAL throttle is 1.0 at 1.5 m, 0.5 at 0.50 m and the 0.15 floor from 0.35 m (`manual_half_throttle_distance_m`, `manual_floor_distance_m`; netting over the lens read as a 0.50 m wall); FOLLOW_ME and WAYPOINT_NAV keep the linear law. The relief also applies near people and animals in MANUAL (safety principle above); the 0.8 m person/animal hard stop stays absolute. RC sticks have ratio-preserving expo, `rc_map.stick_expo` 0.6 (both tracks scaled together, turn radius kept); steering intent reads the linear stick bytes, never the expo-shaped ones.
- **OAK at USB 2 on purpose (2026-11-06)**: the OAK's USB 3 link jams the RTK GPS (antenna within 30 cm of the hub): 32-34 satellites with RTK float at USB 2, 10-11 and no RTK at USB 3, and 0 from a fresh GPS start. `OakDetectionConfig.max_usb_speed = "HIGH"` caps it; the startup line shows `USB HIGH (cap HIGH)`. Do not set `"SUPER"` without the GPS re-test in `docs/ups_hat_b.md`. BasaltVIO needs the RAW IMU reports on 3.6.1 (wall-e's heading keeps UNCALIBRATED); results in `docs/depthai_upgrade.md` section 5. Log-only VIO inside wall-e: `OakDetectionConfig.vio_enabled` (default False); when on, the IMU runs RAW and `_poll_imu` negates y/z back to RDF, so re-run the hand-turn heading sign test after any change there.
- **OAK USB disconnect — RESOLVED 2026-06-11**: `OakDepthReader._run_pipeline` is now a supervisor loop (`pi_app/hardware/oak_depth.py`). On a fatal device/communication error (USB drop / XLink teardown), the worker thread closes the device defensively, marks health `connected=False`, backs off (2s → 5s → then every 10s, interruptible by `stop()` via `Event.wait`), optionally waits for re-enumeration, then rebuilds the pipeline and resumes — no service restart needed. The depth timestamp is **not** refreshed during the outage, so `get_min_distance()` age keeps growing and the staleness fail-safe still stops autonomous motion. `get_health()` adds `connected`, `reconnect_count`, `last_disconnect_ts`. Covered by `pi_app/tests/test_oak_reconnect.py`.
- **IMU heading drift**: gyro-only integration drifts over time (minutes). GPS COG alignment (implemented, refer to the IMU section) corrects it when the robot drives forward at RTK fixed.
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

`logs/run_*.log` / `arm_*.log` mix four JSON line shapes (per-tick, 1 Hz "slow", session header, event) — see `docs/logging_troubleshooting.md` for the field-by-field format. While armed, a tick with an event always writes a line (2026-11-10); before that date, events on ticks that the 10 Hz gate skipped are not in the armed logs.

## Running Tests

```bash
cd /home/pi/wall_e-Mini
python3 -m pytest pi_app/tests -q
```

Use pytest, not `unittest discover`: eight test files are pytest-style (plain
`def test_` functions, among them `test_teleop.py` with the phone deadman and
`test_spp_control_gate.py`), and `unittest discover` collects zero cases from
them (found 2026-11-03).

## Branches

- `main` auto-deploys to the robot every 5 minutes (`bin/auto_deploy.sh`); a change under `docs/` or to a root `*.md` pulls without a service restart.
- No worktrees and no other local branches since 2026-11-10 (only `main` and `research/follow-me-trail-strategies`); the cleanup record is in `docs/NEXT_SESSION.md`, backlog item 8.
- `fm-pump` (on GitHub only since 2026-11-10, `dfbda57`): log-only pump gesture detector (step 1 of `docs/backup_pump_design.md`). Not deployed as of 2026-09-24.
- `research/follow-me-trail-strategies`: research notes (not merged; 1 commit ahead of main).
- `codex/initialize-_last_imu_update-in-controller`: one unmerged Codex commit ("Clamp IMU dt to prevent PID spike"), not reviewed.
- Merged branches were deleted from GitHub on 2026-09-24; the restore SHAs are in `docs/NEXT_SESSION.md`, backlog item 8.
