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

Tests: 703 at a0a2776 → 781 on the branch, all pass, 4 skipped. Changed tests and their reasons are in each commit message.

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

### B.5 Not done

- GPS course over ground and speed from the DFRobot receiver registers (needs a bench read first; a DFRobot audit is in progress).
- RTK stays float: fix 4 has never appeared in any log on the Pi since 2026-07-30. Runbook: `docs/rtk_float_investigation.md`; next step is Kevin's short-baseline test.
- The external 9DoF board is parked (`docs/external_imu_postmortem.md`).
- Auto-deploy: script and runbook are on `main` (5a3e41d). The install on the Pi (cron + one restart) is Kevin's step; the classifier blocks it from this session.

## C. Logging (commits 61939ce, 547e2e2, a36be2b, d761ba9)

1. Log the values already computed: VESC gate/temps/duty, heading-hold blend/gain-scale/saturation, GPS age, straight intent; a session header line (git SHA, gains, scales) at the top of each log file; the OAK health transition as an event line; the console line only at 1 per 5 s when not on a terminal (it wrote about 21 MB per hour into the journal).
2. Speed loop: open-loop byte, target and actual m/s, error, P/I/D, correction, closed flag, logged as `follow_me.speed_loop`.
3. Diagnostics demoted: `imu_pipeline` and `oak_camera_health` move to a 1 Hz "slow" line with the OAK chip temperature; duplicated `imu.oak_imu` counters and the flat `heading_offset_*` copies are dropped; the MCAP telemetry gains the corrected heading, GPS, VESC, charger inhibit and nav fields.
4. RTK GPS driver: every fix-quality transition logged at WARNING; UTC and geoid separation parsed; course and speed over ground read from the receiver's RMC sentence (VTG fallback) and logged, not fed to control; low-quality epochs published; 5 Hz poll deduplicated on the UTC second; LoRa mode verified and logged; health counters and bus reopen. `docs/rtk_gps_module.md`.

Tests: 781 → 867, all pass, 4 skipped. Note: `pi_app/tests/test_log_gating.py` uses bare pytest functions, which `unittest discover` does not collect; its tests have never run under the project test command.

## Technical Names

IoU, eRPM, VESC, YOLOv8, NMS, PID, tracklet, grace hold, open-loop, pole pair, telemetry, `track_id`, `TrackletTracker`, `TargetTracker`, `RpmPlausibilityGate`, OAK-D Lite, BMI270, `OakImuReader`, ZUPT, SSE, GPS, RTK, `imu_source`, `invert_output`, chalk test, `heading_align`, DFRobot, Grok, Kevin.
