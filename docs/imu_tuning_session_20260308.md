# IMU Steering Tuning Session — 2026-03-08

## Context

OAK-D Lite camera is mounted **sideways** (roll ≈ -90°). The onboard BMI270 IMU
provides gyro + accelerometer but no magnetometer, so heading is integrated from
gyro (relative to startup orientation). The system uses a PID heading-hold
controller (`ImuSteeringCompensator`) that applies differential byte corrections
to the left/right motor outputs.

---

## Problem 1: Heading stuck at 0° after calibration

**Symptom:** Robot drove erratically; heading never changed from 0°.

**Root cause:** `OakImuReader.read()` used the body-frame Z-axis gyro (`gz`)
directly for yaw integration. With the camera on its side, body-Z points
horizontally — it no longer measures yaw. The NMNI filter (0.3 dps threshold)
then zeroed out the tiny residual signal completely.

**Fix (attempt 1 — Euler kinematics):** Replaced `yaw_rate = gz` with the
textbook Euler equation:

```
ψ̇ = gy·sin(φ)/cos(θ) + gz·cos(φ)/cos(θ)
```

This blew up because pitch ≈ ±90° puts `cos(θ)` near zero — a singularity.
Vibration pushed pitch across the boundary and produced yaw rates of 18,000+ dps.

**Fix (final — gravity projection):** Project body-frame angular velocity onto
the accelerometer-derived gravity direction:

```
ω_yaw = (ω · a) / |a|²
```

This avoids the Euler singularity entirely. Works at any camera orientation.

**Key file:** `pi_app/hardware/oak_imu.py`, method `read()`.

**Lesson:** When the sensor frame differs significantly from the world frame,
body-axis gyro components don't map 1:1 to world-frame rates. Euler kinematic
equations have singularities; the gravity-projection method is singularity-free.

---

## Problem 2: Violent oscillation after fixing heading

**Symptom:** Robot swerved left-right-left when trying to drive straight.

**Root cause (multi-factor):**

1. **Auto-calibrated PID gains were wildly aggressive.** The calibration wizard
   had run while the heading was broken (stuck at 0°), producing nonsense gains:
   Kp=37.7, Ki=35.6, Kd=10.0, max_correction=220. These turned the controller
   into a bang-bang oscillator.

2. **Vibration noise in the gravity projection.** The raw accelerometer includes
   driving vibration (bumps, pavement texture). Since the gravity-projection uses
   the instantaneous accel vector, vibration spikes corrupt the gravity direction
   estimate, producing noisy yaw rates (jumps of 20-40 dps between consecutive
   samples). This noise feeds into both heading integration and the D-term.

3. **Speed-dependent effective gain.** On a differential-drive robot, the same
   byte correction produces more angular velocity at higher wheel speed. Gains
   tuned at moderate speed become dangerously aggressive at full throttle.

### Fix 2a: Accelerometer EMA smoothing

Added an exponential moving average (alpha=0.12) to the accelerometer values
**before** the gravity projection. Raw accel is still used for roll/pitch
(the complementary filter already smooths those). The smoothed accel removes
vibration while preserving the true gravity direction.

**Key file:** `pi_app/hardware/oak_imu.py` — `_ax_ema`, `_ay_ema`, `_az_ema`.

### Fix 2b: D-term EMA

Enabled the existing `dterm_ema_alpha` config (set to 0.3). This smooths the
yaw rate signal before the derivative term uses it, so the D-term responds to
actual turning trends rather than individual noisy samples.

**Key file:** `config.py` — `dterm_ema_alpha: float = 0.3`.

### Fix 2c: PID gain retuning

Multiple rounds of manual tuning, guided by log analysis:

| Parameter       | Auto-cal (broken) | Round 1  | Round 2 | Final   |
|-----------------|-------------------|----------|---------|---------|
| `kp`            | 37.7              | 3.59     | 1.2     | **0.7** |
| `ki`            | 35.6              | 0.35     | 0.15    | **0.08**|
| `kd`            | 10.0              | 0.92     | 0.4     | **0.5** |
| `max_correction`| 220               | 50       | 35      | **25**  |

**Lesson:** Never trust auto-calibrated gains if the sensor data was broken
during calibration. Start with very conservative gains and increase cautiously.
Ki is the most dangerous gain for oscillation — keep it low.

### Fix 2d: Speed-dependent gain scheduling

Added `gain_schedule_enabled` and `gain_schedule_ref_speed_byte` to config.
The controller attenuates PID corrections at high wheel speed:

```
scale = ref_speed / max(actual_speed, ref_speed)
```

Where `ref_speed = 50` bytes-from-neutral (the speed range where gains were
tuned). At full throttle (128 bytes), corrections are scaled to 39%.

**Key file:** `pi_app/control/controller.py` — applied just before
`_apply_steering_correction()`.

**Results:**

| Condition           | Before scheduling | After scheduling |
|---------------------|-------------------|------------------|
| Moderate speed (50) | ±5° error         | ±2° error        |
| Full speed (128)    | ±28° oscillation  | ±5-8° error      |
| Corrections at full | saturating ±25    | gentle ±3-4      |

**Lesson:** Fixed-gain PID on a differential-drive robot is inherently
speed-dependent. Gain scheduling (or velocity-normalized corrections) is
essential for consistent behavior across the throttle range.

---

## Problem 3: Follow Me steering oscillation

**Symptom:** In Follow Me mode, the robot swung violently side to side until it
lost the person from the camera frame.

**Root cause:** The Follow Me `steering_gain` (2.02) is too high. When the
person drifts 1m off-center, the L-R motor differential reaches 100+ bytes,
creating a violent spin. The robot overshoots, the person appears on the
opposite side, and each cycle amplifies until the target leaves the frame.

**Sequence from logs:**
1. Person at x=+0.1m → gentle tracking (diff=+8)
2. Person drifts to x=-0.2m → robot corrects (diff=-29)
3. Overshoot → person swings to x=+0.7m → diff=+57
4. Overshoot → person at x=-1.1m → diff=-100 (violent spin)
5. Person at x=+1.2m → diff=+111 → person drops out of frame (np=0)
6. Stale x_m keeps robot spinning → tracking lost

**Status:** Not yet fixed. The IMU heading hold correctly backs off during
Follow Me steering (the `correction_zero_at_steering` blend works), so this is
purely a Follow Me controller issue.

**Next step:** Reduce `steering_gain` from 2.02 to ~1.0 and consider increasing
`steering_derivative_gain` for more damping. May also need the same kind of
speed-dependent attenuation applied to Follow Me steering output.

---

## Current Config Snapshot

```python
# IMU PID (config.py → ImuSteeringConfig)
kp = 0.7
ki = 0.08
kd = 0.5
max_correction = 25
deadband_deg = 0.9
dterm_ema_alpha = 0.3
gain_schedule_enabled = True
gain_schedule_ref_speed_byte = 50.0

# Follow Me (config.py → FollowMeConfig)
steering_gain = 2.0231          # TOO HIGH — needs ~1.0
steering_derivative_gain = 0.8542
steering_ema_alpha = 0.3

# OAK IMU (config.py → ImuSteeringConfig)
oak_nmni_enabled = True
oak_nmni_threshold_dps = 0.3
```

---

## Files Modified Today

| File | Changes |
|------|---------|
| `pi_app/hardware/oak_imu.py` | Gravity-projection yaw rate; accel EMA smoothing |
| `pi_app/control/controller.py` | Speed-dependent gain scheduling |
| `config.py` | PID gains, D-term EMA, gain schedule params |

---

## Next Steps

1. **Follow Me steering gain** — Reduce `steering_gain` to ~1.0, test. Consider
   speed-dependent attenuation for Follow Me steering differential too.
2. **Re-run calibration wizard** — Now that the IMU data is clean, the auto-tuner
   should produce sane gains. Compare with manual values.
3. **Steady-state offset at speed** — At full throttle, the gain-scheduled
   corrections are so gentle that ~5° steady-state error persists. Could add a
   small integral boost or separate slow-speed / high-speed integral gains.
4. **Test on grass/dirt** — Pavement was the test surface today. Softer surfaces
   will have different traction and vibration characteristics.
5. **Long-duration drift** — The gyro-only heading will drift over minutes.
   Acceptable for short runs but may need periodic re-zeroing for longer
   autonomous missions.
