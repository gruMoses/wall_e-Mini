# WALL-E Mini Unified Roadmap & TODO

Last updated: 2026-03-03  
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
