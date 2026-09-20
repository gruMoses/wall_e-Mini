# RESULT — fable-follow-fixes (2026-09-19)

This document gives the before and after state of the work on branch `fable-follow-fixes`. Section A (follow-me) is on `main` at a0a2776. Section B (heading) is on `main` and deployed. Section C (logging) is on `main` after this commit.

## A. Follow-me (pushed to main, a0a2776)

### A.1 Person track lock

Before: the tracklet layer matched on IoU only, and the target tracker handed the lock to the closest person on one frame. A closer person who walked in front of the operator took the lock.

After: tracklet association needs depth continuity (0.75 m plus 0.10 m per missed frame). The committed-target match needs depth continuity (0.6 m plus 1.5 m/s multiplied by the time since the last sight). A challenger must be the best candidate for 1.0 s before it takes the lock; the operator reappearing resets the challenger. Set `target_switch_min_s = 0` and the depth gates to 0 to get the old behaviour.

### A.2 Velocity PID

Before: gains were 0 since 2026-06-11 (dead RPM readback caused a lunge and stall cycle). The loop mixed a GPS ground-speed scale with a wheel-speed feedback.

After: one kinematic wheel-speed scale (0.009416 m/s per byte). Gains kp 0.6, ki 0.15, kd 0, integral limit 1.0, correction limit ±0.20 m/s. `RpmPlausibilityGate` trips when a motor is commanded 12 bytes or more from neutral and reports less than 150 eRPM for 0.5 s; the controller then uses open-loop control until real RPM returns (2 s minimum hold). The forward speed is the signed mean of the two wheels, so a turn does not read as overspeed. The PID integral resets on each open-loop tick and on the lost-target reset. Telemetry shows `vesc_rpm_plausible` and `vesc_rpm_gate_trips`.

## B. Heading (on main since 507a0c3; deployed 2026-09-19 15:03 CDT)

### B.1 Field evidence

Live capture from the robot (SSE stream and the JSON run log), robot parked and disarmed, OAK-D Lite BMI270 as the heading source:

- Kevin turned the robot about 90 degrees to the right by hand. The dashboard heading changed 241.4 → 152.8 (−88.6 degrees). The turn back to the left read +92 degrees. The magnitude was correct; the sign was inverted.
- Parked, the heading changed by +0.9 degrees per second. The bias-corrected yaw-axis rate in the log (`gy_body_dps`) was −0.03 right after the boot calibration, −0.46 four minutes later, −1.07 at 24 minutes, and −1.25 at 37 minutes. The heading walked 5,099 degrees in 86 minutes. A second boot at 12:58 gave the same curve (−0.51 at 14 minutes). The robot sat in the sun; after Kevin moved it into shade the residual walked back toward zero (−0.33 one hour later). This is thermal gyro-bias drift after a good boot calibration, with nothing that tracks it.
- Boot log: "External IMU not found, will try OAK-D IMU"; "IMU steering compensation enabled (source: oak_d)". Corrections and GPS were not involved: the GPS offset was not locked, so the dashboard showed the raw gyro heading.

A 57-agent audit of the pipeline found the same two root causes plus three more defects, all verified by three independent lenses each. Grok gave a second opinion on the fix design; its main objection (an IMU cannot vouch for its own stillness) became commit 5.

### B.2 Root causes

1. Sign. `OakImuReader` published `heading = −∫yaw`. The BMI270 +Y axis points down in this mount (the accelerometer reads −1 g on Y at rest; Luxonis documents +Y down for an upright camera), so +gy is already a clockwise turn. Two consumers had compensating flips: `ImuSteeringConfig.invert_output = True` (heading hold) and the waypoint ALIGN pivot (`−yaw` for a positive error). The 2026-07-12 chalk validation compared magnitudes only, so the sign was never checked. The heading-hold D-term was anti-damping under the double flip. Waypoint navigation would have turned away from the target while its numbers converged.
2. Thermal bias drift. The boot calibration removes the bias at boot temperature only. The camera warms by tens of kelvin (more in the sun). The reader-side bias adaptation could never engage: it required a residual below 0.3 degrees per second, which the drift exceeds within minutes.
3. Zeroed yaw rate. `controller.py` read the IMU twice per tick; the second read found no new packets and reported a yaw rate of 0. The D-term and the GPS aligner's "not turning" gate saw a rate that flickered between 0 and real.
4. Silent failures. Boot calibration reported success on zero samples; `is_available` was a one-way latch (11 read errors froze the heading for the session); init failure still printed "enabled".
5. UI. The property map drew the raw boot-relative heading even when a GPS-corrected one existed; the waypoint page never received the heading error or the nav state.

### B.3 Fixes (commits, in order)

1. c09c985 — Compass-native heading, clockwise-positive end to end. `heading = +∫yaw`; the gravity-projected channel is negated to share the convention; `invert_output` default False; waypoint ALIGN pivots +yaw for a positive error; the chalk CLI gains a SIGN pass/fail; closed-loop plant tests prove heading hold and ALIGN converge with the real sign and diverge with the old flag.
2. 8d20063 — Live yaw rate on duplicate IMU reads instead of zero; the stale path still reports 0 and logs once per episode.
3. 2db21ae — Stationary gyro-bias tracking and ZUPT in the producer; boot calibration logs its result; the reader-side bias adaptation is removed.
4. 939eac3 — Honest UI: IMU recovery retry every 5 s; init failure logged with the exception; property map uses the corrected heading when locked; waypoint error and nav state reach the browser.
5. 6c0c157 — Stationary tracking gated on a wheels-stopped witness (VESC eRPM or commanded bytes) in addition to sensor quiet; absolute max-rate bound (5 degrees per second); the reader uses the tracked bias for its rate output; the duplicate-read rate is bounded by sample age; calibration pauses the tracker.
6. 96dd939 — Yaw-axis sign derived from gravity at calibration, with WARNING logs for an inverted or non-vertical mount.
7. 0890138 — `imu_source` pinned to `oak_d`.
8. e4f67e8 — Witness polarity: the robot counts as stopped only when the commanded bytes are at neutral and, when RPM is trusted, both wheels read under 30 eRPM (150 eRPM per wheel in opposite directions is a 1.7 degree-per-second pivot). Defaults aligned (5 degrees per second). Follow-me blind trail search steered toward the mirror of the trail under the new convention; fixed and pinned by a test.

Grok reviewed the full diff: the sign chain is consistent from packet to motor bytes; its one finding (the witness polarity) became commit 8.

Tests: 703 at a0a2776 → 890 on main, all pass, 4 skipped. Changed tests and their reasons are in each commit message.

### B.4 Validation plan on the robot

1. Deploy and reboot the service. Read the boot lines: the calibration line with the measured bias, the "+Y points DOWN … yaw_axis_sign=+1" line, and "IMU steering compensation enabled (source: oak_d)".
2. Parked for 10 minutes: `imu.oak_imu.zupt_active` true, `producer_cum_yaw_y_deg` flat, `tracked_bias_dps[1]` walking with temperature, dashboard heading still.
3. Hand turn about 90 degrees right (Kevin says when he starts): heading increases by about 90; back left: decreases by about 90.
4. Manual drive with heading hold on a straight line: no oscillation; `pid.d` opposes the yaw rate.
5. Waypoint ALIGN toward a bearing to the right: the robot pivots right.

### B.4a Field validation (2026-09-19, after deploy)

- Boot log: "OAK IMU gyro bias measured over 3.00s from 273 samples: x=0.22 y=-0.72 z=0.14 dps"; "+Y points DOWN (ay=-0.99 g): gyro_y is clockwise-positive, yaw_axis_sign=+1"; "IMU steering compensation enabled (source: oak_d)".
- Parked and disarmed for 70 s: heading 0.0 throughout; `zupt_active` true, witness true, tracked bias −0.733 (boot −0.717). Before the fix the same 70 s drifted about 60 degrees.
- Kevin then drove the robot to reposition it (no planned maneuver). In every segment the heading moved with the sign of the track differential: left track faster → heading increased; right track faster → heading decreased; straight → heading held. That is the sign validation from real driving.

### B.4b First autonomous waypoint run (2026-09-19 18:44 CDT)

1. 18:19: first attempt after the sign fix. ALIGN pivoted the correct way, but the fixed 0.5 pivot (about 74 degrees per second) overshot 25–30 degrees on every pivot and ALIGN/DRIVE oscillated left-right (commit 9838203 made the pivot proportional).
2. 18:43:39: GPS heading locked at −33.9 degrees after a short forward drive (lock now persists across disarm, commit f424f74).
3. 18:44:37: nav start accepted. ALIGN error +49.6 → pivot right at ±23 bytes → 43.8 → 34.7 → 10.4 degrees in 3 s → DRIVE. One −12.5 degree correction, then the heading held within ±2 degrees for 30 s while the distance closed 9.6 m → 0.6 m. Arrival inside 1.0 m; mission complete.

This is the first waypoint run on this robot that turned toward the target and arrived.

### B.5 Not done

- GPS course over ground and speed from the DFRobot receiver registers (needs a bench read first; a DFRobot audit is in progress).
- RTK: fix 4 had never appeared in any log since 2026-07-30; at 15:27 today the rover fixed for the first time at a spot 23 m from the usual parking place, after 20 minutes stationary, with drop-outs. Runbook: `docs/rtk_float_investigation.md`; next steps are the short-baseline test and the base checklist.
- The external 9DoF board is parked (`docs/external_imu_postmortem.md`).
- Auto-deploy: script and runbook are on `main` (5a3e41d). The install on the Pi (cron + one restart) is Kevin's step; the classifier blocks it from this session.

## C. Logging (commits 61939ce, 547e2e2, a36be2b, d761ba9)

1. Log the values already computed: VESC gate/temps/duty, heading-hold blend/gain-scale/saturation, GPS age, straight intent; a session header line (git SHA, gains, scales) at the top of each log file; the OAK health transition as an event line; the console line only at 1 per 5 s when not on a terminal (it wrote about 21 MB per hour into the journal).
2. Speed loop: open-loop byte, target and actual m/s, error, P/I/D, correction, closed flag, logged as `follow_me.speed_loop`.
3. Diagnostics demoted: `imu_pipeline` and `oak_camera_health` move to a 1 Hz "slow" line with the OAK chip temperature; duplicated `imu.oak_imu` counters and the flat `heading_offset_*` copies are dropped; the MCAP telemetry gains the corrected heading, GPS, VESC, charger inhibit and nav fields.
4. RTK GPS driver: every fix-quality transition logged at WARNING; UTC and geoid separation parsed; course and speed over ground read from the receiver's RMC sentence (VTG fallback) and logged, not fed to control; low-quality epochs published; 5 Hz poll deduplicated on the UTC second; LoRa mode verified and logged; health counters and bus reopen. `docs/rtk_gps_module.md`.

Tests: 781 → 867, all pass, 4 skipped. Note: `pi_app/tests/test_log_gating.py` uses bare pytest functions, which `unittest discover` does not collect; its tests have never run under the project test command.

## D. Follow-me run 2026-09-19 18:50 — analysis and second round (local commits, not pushed)

Kevin's report: good but not great; jumpy toward the end; did not turn fast enough; the speed felt limited. Full analysis, evidence tables and the field-test procedure: `docs/follow_me_run_2026-09-19.md`. Reproduce with `tools/analyze_follow_me_log.py`.

### D.1 What the log says (215 s, 1,833 ticks, direct pursuit)

1. The detection filter rejected the operator beyond 4.3 m: a 0.45 m person is 0.086 of the frame there and `detect_min_bbox_width` was 0.09, with YOLO still at 0.86–0.90 confidence. One rejected frame at full speed cut the speed by 22 percent (0.3 s decay window); the robot stopped, then surged on re-acquisition. This is the stop-surge cycle.
2. Steering authority was about 5 times too low: 0.22 deg/s of yaw per byte of L-R differential (IMU regression, 1,468 ticks), so the 18-byte direct cap allowed about 8 deg/s while a person crossing at 3 m sweeps about 25 deg/s. The operator walked out of the right frame edge at t = 32 s.
3. The velocity PID wound up to its +21 byte clamp whenever the persistence decay, the accel ramp, the obstacle throttle or the slew limiter cut the command, and dumped it as a surge when the limit lifted.
4. Last 30 s (low sun): the depth corridor reported phantom obstacles at 0.4–1.4 m with 2–7 percent valid pixels; the throttle went 0.9 → 0.0 → 0.4 → 0.8 while the operator was 3 m ahead. This is the "jumpy toward the end".
5. Speed: `VescConfig.max_erpm` 15000 (1.2 m/s) was an unverified cap; the VESC duty was 0.58–0.65 at 13,000 eRPM (about 22,000 eRPM per unit duty).

### D.2 Changes

- Config: `max_erpm` 15000 → 20000 (1.6 m/s, every mode, RC manual included; the derived `speed_loop_mps_per_byte`, `trail_speed_scale_mps_per_byte` and `slip_cmd_diff_per_byte` follow); `detect_min_bbox_width` 0.09 → 0.05; `pid_lateral_kp` 0.4 → 0.8; `max_steer_offset_byte` 25 → 40; `direct_mode_max_steer_byte` 18 → 40.
- `SpeedLayer`: the closed-loop target is the forward byte that reached the motors on the previous tick (`update_telemetry(emitted_forward_byte=...)`), never more than the open-loop request; logged as `speed_loop.target_byte`.
- Persistence decay: `steer_hold_grace_s` (0.30 s hold before decay) and `steer_hold_decay_speed_floor` 0.5.
- Direct pursuit slows into turns: `direct_turn_speed_knee_norm` 0.30, `direct_turn_speed_min_scale` 0.35; logged as `follow_me.turn_speed_scale`.
- Depth corridor: k-th smallest valid depth with `corridor_min_support_px` 400 of support and `corridor_persistence_polls` 2; logged `obstacle.corridor_valid_pct`, `obstacle.corridor_support_px`.
- Tool: `tools/analyze_follow_me_log.py` (timeline, jumpiness with cut attribution, detection-filter histograms, steering, yaw regression, speed-cap use, obstacle flicker).
- Tests: the golden tracking-bytes test and the slip-compensator tests pin the pre-retune tuning (they guard code paths, not tuning); the top-speed test follows `max_erpm`.

### D.3 Not validated

The retune has not been driven. The field test (walk, brisk walk, two 90 degree turns at 3 m, walk out to 6 m) is in the doc. CAUTION: RC manual is 33 percent faster too.

## E. Open issues from the UI audit

- #45 + #46 nav page: STOP is a fixed 72 px control outside the sheet (same id `btnStop`, wired to the same handler), visible while a run is active, paused or launching; the sheet collapses on the MANUAL → WAYPOINT_NAV transition; the nav status line (state, waypoint, distance) is always visible under the toolbar. At 100 percent fill, STOP appears first, GO fades over 250 ms, and a 4 s toast says "Robot is moving. Releasing does NOT stop it. Use STOP."
- #47 nav page: the JS gate is `gpsFix === 4` (backend requires RTK fixed); validation runs on pointerdown before the hold starts, errors block with an in-page notice, warnings show a non-blocking banner; no `confirm()` in the launch path.
- #48 nav page: phone loads collapsed, wide screens load half; half is 40vh; the handle follows the finger (translateY), snaps on release, and a tap toggles collapsed/half.
- #51 nav page: Clear All and Reverse Route are red-outlined, Reverse is two-tap; list controls are 44 px; the small numbers are 12 px.
- #50 teleop: hide/pagehide send neutral and stop the drive loop; the deadman is an inline banner next to ARM ("Deadman tripped: the page was hidden or the connection paused. Press and hold ARM to resume.") instead of a full-screen lock; the loop restarts on re-arm, or on return if the session is still armed. The server cannot tell "tab hidden" from "connection lost" (both are a stale heartbeat), so one message covers both.
- #52 dashboard: MJPEG streams are blanked while the tab is hidden and restored on return; the reconnect loop pauses while hidden.
- #53 property map: `viewport-fit=cover` and safe-area insets on the top bar, canvas and calibration panel.
- #49 arbitration: traced (comment on the issue): one source per tick, nav beats teleop by an `elif`, teleop is never told; the real gap is the hand-back to MANUAL after a mission while a teleop session is still armed. Fix not yet implemented.

## F. 19:53 CDT: could not drive forward into the garage (confirmed from the logs)

Kevin (2026-09-19, about 19:55 CDT): the robot would not drive forward and had to be backed into the garage. Logs: `arm_20260919_184719.log` (tail) and `arm_20260919_195329.log`, plus the service journal.

What the log says, per second from 19:52:30 to 19:54:31: `obstacle.depth_p5_mm` 367-429 (`distance_m` 0.4, `throttle_scale` 0.0) while `depth_p50_mm` was 5,000-7,300 and `depth_valid_pct` 10-13. Both sticks forward at 19:53:32-19:53:47 (ch1/ch2 1992-2104) produced motor bytes at neutral; reverse and pivots from 19:53:50 worked. Camera health was fine (depth age under 0.25 s, no reconnect, no `oak_health` event); no YOLO "Safety STOP" line. Nothing in the code changed: the deployed SHA was 9838203 from the 18:44 waypoint run through this event.

Diagnosis: a real object 0.37 m in front would pull the corridor median down with it; a 5th percentile at the minimum measurable depth (`min_depth_mm` 350) with a far median is max-disparity noise, which always reads "closest possible" and therefore stays steady to the centimetre. The OAK-D Lite has no IR projector; its stereo depth is passive and degrades after sunset. Obstacle avoidance gates forward motion only, which is why backing in worked. The noise population was at least 5 percent of the valid pixels (about 560 of about 11,000), so the section D support floor (400 px) alone does not reject it.

Fixes (commit after 97e469c): near support is the larger of 400 px and `corridor_min_support_frac` 0.02 of the corridor (about 2,000 px; a 5 cm pole at 0.4 m covers about 11,000); `obstacle.corridor_near_px` (valid pixels claiming to be inside `slow_distance_m`) is logged so the next phantom can be sized; and in MANUAL the corridor stop is a floor (`manual_obstacle_min_scale` 0.15, about 0.24 m/s at full stick), so an operator can always creep forward past a phantom. The YOLO person/animal stop tier (distance forced to 0.0) stays absolute in every mode; autonomous modes keep the hard stop.

Also seen: the OAK chip temperature was 82-83 C throughout (the 1 Hz `oak.chip_temp_c`); worth watching on a hot day.

## Technical Names

IoU, eRPM, VESC, YOLOv8, NMS, PID, tracklet, grace hold, open-loop, pole pair, telemetry, `track_id`, `TrackletTracker`, `TargetTracker`, `RpmPlausibilityGate`, OAK-D Lite, BMI270, `OakImuReader`, ZUPT, SSE, GPS, RTK, `imu_source`, `invert_output`, chalk test, `heading_align`, DFRobot, Grok, Kevin.
