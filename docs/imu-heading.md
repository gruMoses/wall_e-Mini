Moved out of CLAUDE.md on 2026-11-14 (text kept as it was; it was the "IMU" section). Also read `docs/heading_tuning.md` and `docs/gps_heading_alignment.md`.

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

