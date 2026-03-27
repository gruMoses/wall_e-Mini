# WALL-E Mini Unified Roadmap & TODO

Last updated: 2026-03-15  
Scope: Single source of truth for active engineering work and prioritized enhancements.

## Current Status

- [x] OAK-D performance optimization phases (P1-P4) implemented and validated
- [x] IMU fidelity program (IMU-0..IMU-5) implemented and validated
- [x] Conservative IMU default enabled (`oak_nmni_enabled=True`)
- [ ] Program-level process hardening still open (strict baseline repeat policy + per-phase rerun discipline)

---

## P0 Open Program Items (Do Next)

- [ ] Complete strict baseline matrix requirement (3x each scenario):
  - idle
  - control-loop only
  - OAK pipeline only
  - full stack nominal
  - full stack stress
  - 20+ min thermal soak
- [ ] Enforce rerun discipline after each future phase/change.
- [ ] Publish one canonical validation report template that all future feature work follows.

---

## Active Engineering Backlog

### P1 (High Impact, Near Term)

- [ ] Wheel speed feedback via VESC telemetry read-back in `VescCanDriver`
  - parse status CAN frames and expose RPM/speed API
- [ ] Slew rate limiter for track commands (mode-aware, accel/decel asymmetric)
- [ ] Terrain roughness governor (accel RMS + high-pass + speed scaling)
- [ ] Tilt protection governor (roll/pitch-based speed reduction + hard stop at extreme tilt)

### P2 (Medium-High Impact)

- [ ] **Trail-following steering: heading-noise contaminates breadcrumbs**
  - Root cause: `camera_to_world` projects `(x_cam, z_cam)` through the instantaneous IMU heading. At 3 m range, ±5° of heading noise creates ±0.26 m lateral scatter in crumb world-y positions. A 20° heading drift manufactures a ~1 m phantom curve in the trail that pure pursuit follows instead of the person's actual path.
  - [x] **Option 1 — Log-replay simulator**
    Implemented: `tools/replay_follow_me_log.py`. Reads structured JSON logs, replays each FOLLOW_ME segment through `FollowMeController` with `time.monotonic` mapped to log timestamps, optional GPS replay, and compares replayed `speed_offset` / `steer_offset` to logged values. Use `--list-runs`, `--run N`, `--csv`, `--no-gps`. Large logs can take tens of seconds to scan.
  - **Option 2 — 2D physics simulator.**
    Add differential-drive kinematics (motor bytes → heading rate + forward velocity). Define synthetic person trajectories (straight + left turn, right turn, U-turn, walk around obstacle). Inject configurable IMU noise. Generate synthetic camera detections from known person position. Run `FollowMeController` in the loop, plot robot vs person path, and score tracking accuracy. Enables parameter sweeps and regression tests on scenarios we haven't physically run.
  - **Option 3 — Noise-immune trail crumbs (the algorithmic fix).**
    Use the simulator to validate before deploying. Candidate approaches:
    - EMA-filtered heading for crumb projection (smooth out high-frequency noise).
    - Lay crumbs based on person bearing angle rather than full world projection (heading-noise-immune for lateral position).
    - Use the trail's own accumulated direction as the heading reference instead of the noisy IMU, at least for the lateral component.
    - Reject crumbs whose lateral position jumps more than expected given the person's camera-frame movement between frames.

- [ ] **OAK camera staleness recovery (USB disconnect / no auto-reconnect)**
  - Root cause: when OAK-D USB disconnects (power/cable/hub), device re-enumerates but the running process keeps the old device handle; depth/detection queues stop receiving frames → `oak_camera_health` goes stale; only recovery today is restarting the service.
  - Add staleness-based recovery: if `oak_camera_health.is_stale` and e.g. `depth_recv_age_s > 60`, optionally restart the OAK reader thread (stop pipeline, reopen device, restart pipeline) so the process reconnects to the device without a full service restart.
  - Log the first transition to stale (e.g. once per run) with `depth_recv_age_s` and a note that USB disconnect is likely.
  - Optional: watch dmesg or udev for OAK USB disconnect and trigger OAK restart or service restart.

- [ ] Backlash compensation
  - measurement pre-step + compensator strategy (pulse/dead-zone inverse/hysteresis)
- [ ] Follow Me smoothing bundle
  - distance dead zone, steering EMA, comfort approach speed
- [ ] Unified speed-governor architecture (`final_scale = min(all_limiters)`)
- [ ] Stuck detection + recovery routine

### P3 (Medium Impact)

- [ ] Depth predictive slowdown (time-to-collision + continuous braking curve)
- [ ] Gain scheduling (speed/mode/surface indexed PID sets)
- [ ] RTK GPS hardening
  - reconnection/backoff, quality transition handling, startup convergence policy

### P4 (Longer Horizon)

- [ ] Inertial navigation aiding (GPS + IMU + odometry fusion)
- [ ] Terrain classification (stretch)
- [ ] Creative/demo features (LED ring, sound, voice, auto-docking)

---

## Completed Highlights (Condensed)

- [x] OAK depth offload, queue-policy tuning, recorder load reductions
- [x] Follow Me tracker continuity and docs alignment
- [x] IMU device timestamp propagation and controller cadence alignment
- [x] IMU telemetry precision restoration
- [x] IMU bounded ingestion mode + safe fallback controls
- [x] Optional drift/noise features implemented; conservative defaults validated

---

## Validation & Handoff References

- IMU final handoff: `docs/imu_fidelity_completion_summary_20260302.md`
- IMU phase validation docs:
  - `docs/imu0_instrumentation_snapshot_20260302.md`
  - `docs/imu1_timestamp_validation_20260302.md`
  - `docs/imu2_status_precision_validation_20260302.md`
  - `docs/imu3_bounded_ingestion_implementation_20260302.md`
  - `docs/imu4_cadence_alignment_validation_20260302.md`
  - `docs/imu5_optional_features_implementation_20260302.md`
- Performance baseline/report docs:
  - `docs/performance_baseline_report_20260301.md`
  - `docs/performance_p1_results_20260301.md`
