# WALL-E Mini — Ideas & Enhancements Tracker

Last updated: 2026-03-02
Status: Living document. Items are grouped by theme and prioritized by estimated impact/effort.

Legend: `[x]` done · `[-]` in progress · `[ ]` not started · `[?]` needs investigation

---

## 1. IMU Pipeline — Fix the 15 Hz Bottleneck (Critical)

The single biggest performance issue discovered during code review.

**Problem**: The BMI270 on the OAK-D Lite produces data at 100 Hz, but
`_poll_imu()` runs inside the obstacle-avoidance loop whose cadence is set by
`config.obstacle_avoidance.update_rate_hz = 15.0`. So the controller only
receives fresh IMU state ~15 times/second, even though the main loop runs at
~50 Hz and the PID config expects 80 Hz. Additionally, `_poll_imu()` drains
the queue and keeps only the last sample, discarding ~85% of readings.

**Sub-items**:

- [ ] **Decouple IMU polling from depth polling.** Options:
  - (A) Dedicated IMU thread inside `OakDepthReader` with its own tight sleep
    (e.g. 5 ms for 200 Hz).
  - (B) Tighter inner loop: poll IMU at high frequency between depth polls
    (`while not depth_ready: poll_imu(); sleep(5ms)`).
  - (C) Use DepthAI `Script` node to push IMU samples via a separate
    non-blocking output queue that the host drains independently.
- [ ] **Average or fuse discarded samples** instead of keeping only the last one.
  For gyro, integrate all samples into a running yaw delta. For accel, average.
- [ ] **Use device-side timestamps** (`imuPacket.acceleroMeter.getTimestamp()`)
  instead of `time.monotonic()` on the host for accurate dt computation.
- [ ] **Align configured rate with actual rate.** `ImuSteeringConfig.update_rate_hz`
  is 80 but actual delivery is ~15 Hz. Either fix the pipeline to deliver 80+ Hz
  or set the config to match reality (affects PID derivative term scaling).

Impact: **Very High** — this is the root cause of sluggish/oscillatory heading hold.
Effort: Medium (option A is ~50 lines of new thread code).

---

## 2. PID Heading Hold Improvements

Findings from the code review of `imu_steering.py` and `controller.py`.

- [ ] **Low-pass filter on D-term.** Currently raw yaw rate feeds Kd with no
  smoothing, making it vulnerable to gyro noise spikes. A first-order EMA
  (alpha ~0.3) or 2nd-order Butterworth at 10-15 Hz is standard practice.
- [ ] **Integral behavior in deadband.** The integral still accumulates while
  the heading error is within the 0.9° deadband (only P-term is zeroed).
  This causes a slow wind-up followed by a sudden correction jump when the
  error crosses the threshold. Fix: freeze integral inside deadband, or apply
  a continuous smooth deadband function.
- [ ] **Thread safety gap.** Line 226 of `imu_steering.py` writes
  `self.state.last_steering_input` outside the RLock. Wrap it.
- [ ] **Telemetry precision bug.** `controller.py` `get_imu_status()` rounds
  ALL float fields to `int`, destroying PID telemetry resolution. Use 2-3
  decimal places instead.
- [ ] **Derivative-on-measurement vs derivative-on-error.** Current code uses
  derivative-on-measurement (good), but confirm sign convention is consistent
  across all code paths.

Impact: High (smoother, more predictable heading hold).
Effort: Low-Medium.

---

## 3. Backlash Compensation

Discussed conceptually — no code changes made yet.

**Problem**: Gear train / belt / coupler play means that when steering direction
reverses, there is a dead zone where the motor moves but the wheels don't.
This manifests as limit cycling in PID heading hold and sloppy feel in manual.

**Sub-items**:

- [ ] **Measure backlash with IMU.** During auto-tune pre-step:
  1. Spin motors slowly in one direction until yaw rate is steady.
  2. Reverse motor slowly; measure how far the motor must travel before
     the IMU detects yaw rate in the new direction.
  3. Record this angle (or motor command duration) as the backlash amount.
- [ ] **Compensate in PID output.** Options:
  - (A) Dead-zone inverse: on direction reversal, inject an extra command
    pulse equal to the measured backlash, then resume normal PID.
  - (B) Backlash-aware integral: freeze integral accumulation during the
    backlash zone so it doesn't wind up during the dead travel.
  - (C) Hysteresis band: widen the deadband asymmetrically when direction
    has recently reversed.
- [ ] **Add backlash measurement to auto-tuner** as a pre-step before PID
  relay tuning begins.
- [ ] **Store backlash value in config** (`ImuSteeringConfig.backlash_deg` or
  `backlash_byte`).

Impact: Medium-High (eliminates limit cycling at heading hold).
Effort: Medium.

---

## 4. Wheel Speed Feedback

**Opportunity**: The Arduino RC input channels (D2-D6) are no longer used for
RC pulse reading. These are interrupt-capable pins that can be repurposed for
quadrature encoder inputs. Additionally, the VESC inherently tracks actual eRPM
and can report it back over CAN.

**Current state**: Neither encoder support nor VESC telemetry read-back exists
in the codebase. The Arduino firmware (`arduino_rc_reader.ino`) only reads RC
pulses and drives H-bridge PWM. The `VescCanDriver` only sends RPM commands
(arbitration ID `0x300 + can_id`) and never reads CAN responses.

**Sub-items**:

- [ ] **VESC telemetry read-back (easiest path).** VESC sends status frames on
  CAN automatically:
  - `0x900 + id`: eRPM (32-bit), current (16-bit), duty (16-bit)
  - `0x0E00 + id`: voltage, temp
  - Add a CAN receive thread in `VescCanDriver` that parses these frames and
    exposes `get_rpm()` / `get_speed()`.
- [ ] **Arduino encoder support (alternative).** If using the Arduino H-bridge
  path instead of VESC:
  - Repurpose D2/D3 (INT0/INT1) for left/right encoder A-channel
    (single-channel counting gives speed, not direction — but direction is
    known from motor command).
  - Report counts in the existing CSV serial protocol, e.g.
    `CH1,CH2,CH3,CH4,CH5,ENC_L,ENC_R\n`.
  - Parse in `ArduinoRCReader` and expose via `get_wheel_speeds()`.
- [ ] **Closed-loop speed control.** Once actual wheel speed is known:
  - PID speed controller: commanded RPM vs actual RPM per side.
  - Replaces the current open-loop byte→RPM mapping.
  - Better straight-line tracking without needing IMU correction.
- [ ] **Slip detection.** Compare commanded RPM to actual RPM:
  - `slip_ratio = (commanded - actual) / max(commanded, 1)`
  - If `slip_ratio > threshold` (e.g. 0.3), reduce commanded speed.
  - Critical for outdoor operation on grass/mud/gravel.
- [ ] **Odometry.** Wheel encoder ticks + track width → dead reckoning:
  - Fuse with IMU heading for better position estimate.
  - Feed into waypoint navigation when GPS is degraded.
  - Use for Follow Me breadcrumb path tracking.

Impact: **Very High** (enables closed-loop speed, slip detection, odometry).
Effort: Medium (VESC path) to High (encoder path).

---

## 5. Terrain Roughness Detection & Speed Governing

Use the BMI270 accelerometer to detect rough terrain and automatically slow down.

**Sub-items**:

- [ ] **Roughness metric.** Compute RMS of Z-axis accelerometer over a sliding
  window (e.g. 0.5s). Thresholds (approximate, need field calibration):
  - Smooth pavement: < 0.05 g RMS
  - Moderate (gravel): 0.05 – 0.12 g RMS
  - Rough (dirt/roots): 0.12 – 0.20 g RMS
  - Dangerous (stairs/drop): > 0.20 g RMS
- [ ] **Speed governor.** Output a 0.0–1.0 speed scale factor based on
  roughness. Apply as a multiplier to the commanded speed before motor output.
- [ ] **High-pass filter** the accel signal first to remove gravity and slow
  tilting, isolating only vibration energy.
- [ ] **Configurable thresholds** in a new `TerrainConfig` dataclass.
- [ ] **Telemetry logging** of roughness metric for tuning.

Impact: High (prevents mechanical damage, smoother ride on rough terrain).
Effort: Low-Medium (pure software, no new hardware).

---

## 6. Tilt Protection

Use the BMI270 roll/pitch estimates (already computed in `OakImuReader`) to
detect dangerous tilt angles and limit speed or stop.

- [ ] **Forward/backward pitch limit.** If pitch exceeds threshold (e.g. 25°),
  reduce max speed or stop. Prevents tipping on steep slopes.
- [ ] **Roll limit.** If roll exceeds threshold (e.g. 30°), cut throttle to
  prevent rollover on side slopes.
- [ ] **Gradual speed reduction** as tilt approaches the limit, not a hard
  cliff. E.g. linear ramp from 100% at 15° to 0% at 30°.
- [ ] **Emergency stop** at extreme tilt (e.g. 45°) with a latch requiring
  manual reset.

Impact: Medium-High (safety critical for outdoor slopes).
Effort: Low.

---

## 7. Slew Rate Limiter (Smooth Acceleration/Deceleration)

FRC-proven technique for eliminating jerky starts and stops.

- [ ] **Limit rate-of-change of motor commands** between consecutive control
  loop ticks. E.g. max 5 byte-units per tick at 50 Hz = 250 byte-units/sec
  full ramp time ~1 second.
- [ ] **Asymmetric rates.** Deceleration limit should be higher (faster) than
  acceleration for safety — the robot should be able to stop quickly.
- [ ] **Per-mode tuning.** Follow Me mode may want gentler slew than manual RC.
- [ ] **Configurable in `Config`**: `slew_accel_per_s`, `slew_decel_per_s`.

Impact: Medium (dramatically better feel, less mechanical stress).
Effort: Very Low (~20 lines).

---

## 8. Speed Governor Architecture (Min-of-All-Limiters)

Unify all speed-limiting systems into a clean architecture.

- [ ] **Each subsystem outputs a 0.0–1.0 scale factor independently:**
  - Terrain roughness governor
  - Tilt governor
  - Obstacle proximity governor (already exists partially)
  - Battery voltage governor (future)
  - Slip detection governor
  - Thermal governor (if motor/ESC temp available from VESC)
- [ ] **Final speed = commanded_speed × min(all_governors).**
- [ ] **Log which governor is the active limiter** for debugging/telemetry.
- [ ] **Governor override for emergency stop** (0.0 from any governor = full stop).

Impact: Medium (clean architecture, easy to add new limiters).
Effort: Low-Medium.

---

## 9. Follow Me Improvements

Enhancements to make person-following feel smooth and intelligent.

- [ ] **Feedforward speed matching.** Estimate target person's speed from
  successive depth readings. Apply as a feedforward term so the robot
  anticipates instead of purely reacting.
- [ ] **Breadcrumb path following.** Record the path the person walked as a
  sequence of (x, y) waypoints (using wheel odometry + IMU heading). If the
  target is temporarily lost, follow the breadcrumb trail instead of stopping.
- [ ] **Dead zone on following distance.** Don't adjust speed when within
  ± tolerance of the target distance (e.g. ±0.2 m). Prevents oscillation
  at steady-state follow.
- [ ] **EMA steering smoothing.** Apply exponential moving average to steering
  commands to reduce jitter from noisy person detection bounding boxes.
  (alpha ~0.3–0.5)
- [ ] **Comfort-based approach speed.** Limit approach speed when close to the
  person; reduce speed proportionally as distance decreases below
  `follow_distance_m`.
- [ ] **OAK-D ObjectTracker** (mentioned in performance_optimization_todo.md P3).
  Use on-device object tracking for person continuity across frames, reducing
  host-side target switching/jitter.

Impact: High (Follow Me is a flagship feature).
Effort: Medium.

---

## 10. Gain Scheduling

Adapt PID gains based on operating conditions.

- [ ] **Speed-indexed gains.** Different Kp/Ki/Kd at different speeds:
  - At crawl speed: high Kp for precise heading, low Kd.
  - At cruise speed: moderate Kp, higher Kd for stability.
  - Store as a lookup table with linear interpolation.
- [ ] **Mode-based gains.** Different gain sets for:
  - Manual RC (responsive, driver-in-the-loop)
  - Follow Me (smooth, less aggressive corrections)
  - Autonomous waypoint nav (precise, tighter tolerances)
- [ ] **Surface-adaptive gains.** If terrain roughness is known:
  - Rough terrain: reduce Kp to avoid amplifying vibration.
  - Smooth terrain: allow tighter heading hold.

Impact: Medium.
Effort: Medium.

---

## 11. Stuck Detection & Recovery

Detect when the robot is stuck and take corrective action.

- [ ] **Detection signals** (combine for confidence):
  - Motor commanded but IMU shows no yaw change AND no acceleration.
  - Wheel speed feedback (if available) shows stall or near-zero.
  - Obstacle depth shows object getting no closer despite forward command.
  - Current draw anomaly from VESC telemetry (high current + no movement).
- [ ] **Recovery behavior:**
  - Back up briefly, turn a random angle, retry.
  - After N retries, stop and signal for help (LED flash, buzzer, telemetry alert).
- [ ] **Timeout.** If stuck for > N seconds with no recovery, safe-stop.

Impact: Medium (essential for autonomous modes).
Effort: Medium.

---

## 12. Depth-Based Predictive Slowdown

Use the OAK-D depth field to slow down before reaching obstacles, not just at
fixed distance thresholds.

- [ ] **Time-to-collision (TTC).** Compute `TTC = distance / closing_speed`.
  Slow down when TTC < threshold (e.g. 2 seconds), not just when distance <
  threshold. This accounts for approach speed.
- [ ] **Lateral obstacle scanning.** Expand the depth ROI to check left/right
  of the current path. If the robot is turning toward an obstacle, start
  slowing earlier.
- [ ] **Smooth braking curve.** Instead of discrete slow/stop zones, use a
  continuous speed limit: `max_speed = f(TTC)`.

Impact: Medium-High (much better obstacle avoidance feel).
Effort: Medium.

---

## 13. Inertial Navigation Aiding (GPS + IMU + Wheel Odometry)

Fuse multiple sensors for better position estimation in autonomous modes.

- [ ] **Complementary filter or EKF** fusing:
  - GPS lat/lon (1 Hz, high absolute accuracy with RTK)
  - IMU heading (100 Hz, no drift if fused with GPS course-over-ground)
  - Wheel odometry (high rate, drifts over distance but good short-term)
- [ ] **GPS dropout bridging.** When RTK fix is lost (quality < 4), continue
  navigating on IMU + odometry for short periods. Set a maximum coast time
  (e.g. 10 seconds) before safe-stop.
- [ ] **Course-over-ground for yaw correction.** When moving at sufficient
  speed, use GPS heading to correct gyro drift (eliminates need for
  magnetometer in outdoor environments).
- [ ] **Fence / geofence.** Use GPS to define an operational boundary. If the
  robot approaches the fence, reduce speed and eventually stop.

Impact: Medium (needed for robust waypoint navigation).
Effort: High.

---

## 14. RTK GPS — Known Gotchas

Findings from forum/documentation research on the DFRobot KIT0198 (LC29HBS base
+ LC29HDA rover, LoRa link).

- [ ] **I2C 255-byte buffer limit.** The DFRobot I2C wrapper reads at most
  255 bytes per register read. Current code reads only 29 bytes per poll (OK),
  but if we ever need to read NMEA sentences via I2C, we must split reads.
- [ ] **No accuracy reporting.** The module does not expose GST or PQTMEPE
  accuracy estimate sentences via I2C. We only get fix quality (0-5) and HDOP.
  Cannot distinguish between a 2 cm RTK fix and a 10 cm one.
- [ ] **LoRa link fragility.** The 915 MHz LoRa correction link has limited
  range (~300 m LOS) and can lose corrections near buildings/trees. Monitor
  fix quality transitions (4→1) and degrade gracefully.
- [ ] **Warm-start to RTK fix time.** Allow 60-120 seconds after power-on for
  the rover to converge from SPP → DGPS → RTK float → RTK fixed.
  Do not begin autonomous navigation until quality ≥ 4.
- [ ] **No I2C bus reconnection.** Current `rtk_gps.py` opens the I2C bus once;
  if the bus errors out, the reader thread exits permanently. Add reconnection
  logic with exponential backoff.
- [ ] **Base station antenna placement.** Base needs clear sky view; even
  partial obstruction degrades correction quality. Document antenna requirements.

Impact: Medium (reliability of GPS waypoint navigation).
Effort: Low-Medium.

---

## 15. Gyro Drift Mitigation

Techniques to reduce yaw drift in the pure-gyro heading integration.

- [ ] **NMNI (No Motion No Integration).** When the IMU detects the robot is
  stationary (accel magnitude ≈ 1g, gyro < threshold), stop integrating gyro
  to prevent static drift. Already partially exists as concept in `OakImuReader`
  but not fully implemented.
- [ ] **ZUPT (Zero-Velocity Update).** When wheel speed shows zero (requires
  encoder/VESC feedback), force gyro bias recalibration on-the-fly.
- [ ] **Continuous bias estimation.** Track the running mean of gyro output
  during known-stationary periods and subtract it from live readings.
- [ ] **GPS course-over-ground correction** (see item 13) — the best long-term
  drift mitigation for outdoor operation.

Impact: Medium-High (heading hold degrades over minutes without drift mitigation).
Effort: Low (NMNI/ZUPT) to Medium (GPS fusion).

---

## 16. Terrain Classification (Future / Stretch)

Use accelerometer vibration signatures to identify surface type.

- [ ] **FFT frequency analysis** of accelerometer Z-axis:
  - Pavement: low-frequency, low-amplitude
  - Gravel: mid-frequency broadband
  - Grass: low-frequency, higher amplitude
  - Each surface has a characteristic spectral fingerprint.
- [ ] **Lightweight ML classifier.** Train on labeled accel data from different
  surfaces. Run inference on Pi (small model, < 1ms per classification).
- [ ] **Adapt behavior by terrain type.** Different max speeds, PID gains,
  and slew rates per classified surface.

Impact: Low-Medium (cool but not essential).
Effort: High.

---

## 17. Creative / Fun Ideas

- [ ] **Personality animations.** Idle head movements, excited wiggles when
  detecting a person, "confused" head tilt when obstacle blocks all paths.
- [ ] **Sound effects.** Play sounds through a connected speaker for different
  states (following, obstacle detected, stuck, low battery).
- [ ] **LED status ring.** Addressable LED strip showing mode (manual=blue,
  follow=green, autonomous=yellow, error=red), battery level, GPS status.
- [ ] **Voice commands** via Bluetooth mic. "Wall-E, follow me" / "Wall-E, stop" /
  "Wall-E, go home" (return to GPS home point).
- [ ] **Auto-docking.** Use ArUco markers or AprilTags (detectable by OAK-D) to
  guide the robot to a charging station.

Impact: Low (fun factor, demo appeal).
Effort: Varies.

---

## Priority Summary

| Priority | Item | Impact | Effort |
|----------|------|--------|--------|
| **P0** | Fix 15 Hz IMU bottleneck (#1) | Very High | Medium |
| **P0** | PID heading hold fixes (#2) | High | Low-Med |
| **P1** | Wheel speed feedback via VESC (#4) | Very High | Medium |
| **P1** | Slew rate limiter (#7) | Medium | Very Low |
| **P1** | Terrain roughness governor (#5) | High | Low-Med |
| **P1** | Tilt protection (#6) | Med-High | Low |
| **P2** | Backlash compensation (#3) | Med-High | Medium |
| **P2** | Follow Me improvements (#9) | High | Medium |
| **P2** | Speed governor architecture (#8) | Medium | Low-Med |
| **P2** | Gyro drift mitigation (#15) | Med-High | Low-Med |
| **P3** | Depth predictive slowdown (#12) | Med-High | Medium |
| **P3** | Stuck detection & recovery (#11) | Medium | Medium |
| **P3** | Gain scheduling (#10) | Medium | Medium |
| **P3** | RTK GPS hardening (#14) | Medium | Low-Med |
| **P4** | Inertial nav aiding (#13) | Medium | High |
| **P4** | Terrain classification (#16) | Low-Med | High |
| **P4** | Creative / fun (#17) | Low | Varies |

---

## Cross-References

- Performance optimization work: `docs/performance_optimization_todo.md`
- IMU steering details: `docs/imu_steering.md`
- OAK-D integration plan: `docs/oak_d_lite_integration_plan.md`
- Hardware mapping: `docs/hardware_mapping.md`
- VESC CAN protocol: `docs/can_vesc.md`
