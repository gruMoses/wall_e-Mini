"""
Configuration file for WALL-E Mini robot control system.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GpsHeadingAlignConfig:
    """One-shot GPS alignment for the gyro-only IMU.

    The displacement lock below wants a forward straight manual run.
    The COG lock (``cog_lock_enabled``) does not need a straight path.
    Once locked, the offset is kept for the life of the process, including
    disarm. A service restart or an IMU frame break clears it.
    """
    enabled: bool = True
    # Calibrated for slower forward manual runs.
    min_distance_m: float = 0.8       # displacement required in history window
    min_speed_mps: float = 0.12       # trust COG once robot is clearly moving
    min_fix_quality: int = 4          # RTK fixed only
    # Retained for config compatibility; post-lock refinement is disabled.
    alpha: float = 0.1
    history_seconds: float = 8.0      # longer window stabilizes low-speed COG
    # Initial lock is permitted only during an explicit manual straight run
    # and while body yaw rate remains below this threshold.
    max_lock_yaw_rate_dps: float = 3.0

    # ── Per-epoch course-over-ground lock (2026-09-19) ─────────────────────
    # The receiver reports its own Doppler course over ground each epoch
    # (GpsReading.cog_deg). Pairing each epoch's course with the IMU heading
    # at that same instant gives an offset sample that does not depend on a
    # straight path or a straight command: wobble cancels sample by sample.
    # Lock when enough samples agree. Works from the RC or the phone; the only
    # requirements are RTK fixed, moving forward at walking pace, and not
    # pivoting. A single mixed stick and uneven ground made the displacement
    # method above a coin flip (Kevin, 2026-09-19).
    cog_lock_enabled: bool = True
    cog_min_speed_mps: float = 0.3        # Doppler course is noisy below this
    cog_max_yaw_rate_dps: float = 6.0     # ~1 deg of GPS-latency error per sample
    cog_min_samples: int = 6              # epochs (1 Hz) that must agree
    cog_max_spread_deg: float = 8.0       # circular std of the samples
    cog_window_s: float = 20.0            # samples older than this expire
    # Lock persistence (2026-09-19, Kevin's decision): the offset is kept
    # across disarm / re-arm within one service run and re-verified against
    # live course over ground whenever the robot drives forward at RTK fixed.
    # If the samples agree with each other but disagree with the frozen offset
    # by more than this, the lock moves to the new value with a WARNING. The
    # lock is dropped on an IMU frame discontinuity (OAK reconnect / cum reset).
    cog_verify_max_error_deg: float = 15.0


@dataclass(frozen=True)
class ImuSteeringConfig:
    """Configuration for IMU-based steering compensation."""
    
    # Enable/disable IMU steering
    enabled: bool = True
    
    # PID gains for heading control (tuned for WAYPOINT_NAV DRIVE steering)
    kp: float = 1.2       # Proportional gain (heading error to steering correction)
    ki: float = 0.08      # Integral gain (slow bias removal; high values cause oscillation)
    kd: float = 0.5       # Derivative gain (yaw rate damping)

    # Control parameters
    max_correction: int = 35      # Maximum steering correction in byte units (0-255)
    deadband_deg: float = 0.9    # Minimum heading error to trigger correction (degrees)
    max_integral: float = 30.0   # Maximum integral term to prevent windup
    # Steering-output inversion. Default flipped True -> False on 2026-09-19:
    # it existed only to compensate an inverted heading. The OAK reader now
    # publishes a clockwise-positive compass heading (see OakImuReader.read),
    # and controller.py applies a positive correction as left+ / right- = turn
    # right. With a CW-positive heading a rightward drift gives
    # error = target - heading < 0 -> negative correction -> steer left. No
    # inversion is needed. Note the D-term (imu_steering.py: d_term =
    # -kd * yaw_rate) was ANTI-damping under the old double flip, because
    # yaw_rate was CW-positive while heading was CCW-positive; it is true
    # damping now. Knob kept for bench A/B on a re-mounted IMU.
    invert_output: bool = False
    # Steering neutral detection (hysteresis) to lock heading until commanded turn
    steering_neutral_enter: float = 0.08  # |steering_input| below this enters neutral
    steering_neutral_exit: float = 0.15   # |steering_input| above this exits neutral
    # Manual/BT heading-hold gate: if steering intent stays below this while
    # moving, treat as "go straight" even if channel bytes are imperfectly matched.
    manual_hold_max_steering: float = 0.18
    # Safety cap for heading-hold authority while driving via BT/web remote.
    # Prevents aggressive uncommanded pivots if target or heading jumps.
    manual_bt_max_correction: float = 12.0
    neutral_dwell_s: float = 0.0          # Optional dwell before locking target (0 = immediate)
    # Straight-intent detection for dual-throttle skid steer
    straight_equal_tolerance_us: int = 120     # |ch1_us - ch2_us| <= tol -> straight intent
    straight_min_throttle_us: int = 80        # max(|ch1-1500|,|ch2-1500|) to qualify as moving
    # Relative tolerance to allow proportional mismatch at higher throttle
    straight_relative_tolerance_pct: float = 0.35
    # Optional per-side bias applied only during straight intent (bytes)
    #straight_bias_left_byte: int = -20
    #straight_bias_right_byte: int = 20
    # Hysteresis time to keep straight intent latched despite brief mismatch (seconds)
    straight_disengage_hysteresis_s: float = 0.80
    # Steering-blend: corrections scale down as absolute steering_input grows; zero at this magnitude
    correction_zero_at_steering: float = 0.50
    
    # Debug and logging
    log_steering_corrections: bool = False  # Enable debug logging of steering corrections
    
    # Timing
    update_rate_hz: float = 60.0  # Controller-side IMU update cap (aligned with default OAK IMU poll)
    # OAK-D IMU ingestion controls (IMU-3 + lossless producer yaw):
    # Yaw is integrated from EVERY drained packet on the producer regardless of
    # mode (see oak_imu_yaw_producer / docs/heading_tuning.md). These knobs are
    # retained for poll cadence and metrics compatibility only.
    # - "latest" / "bounded": historical snapshot selection (no longer drops yaw samples)
    oak_imu_packet_mode: str = "latest"
    oak_imu_max_packets_per_poll: int = 4
    # Optional dedicated OAK IMU polling cadence (separate from depth poll loop).
    oak_imu_poll_hz: float = 60.0
    # Optional OAK yaw drift mitigations (IMU-5). NMNI enabled by default after validation.
    oak_nmni_enabled: bool = True
    oak_nmni_threshold_dps: float = 0.3
    # Stationary gyro-bias tracking + ZUPT (zero-velocity update), added
    # 2026-09-19. Field evidence: parked, the heading wound +0.9 deg/s; the
    # robot JSON log showed gy_body_dps (bias-subtracted body rate about Y) at
    # -0.03 right after a GOOD 3 s boot calibration, then -0.46 / -0.83 /
    # -1.07 / -1.25 over the next 37 min, plateauing near -1.2 deg/s. That is
    # thermal gyro-bias drift with nothing tracking it. NMNI at 0.3 deg/s
    # cannot gate it, and the old reader-side bias_adapt only ran when
    # |rate| < 0.3 deg/s, so it could never engage once the drift passed that.
    # (oak_bias_adapt_enabled / oak_bias_adapt_alpha were removed with it.)
    oak_stationary_bias_tracking_enabled: bool = True
    # Rolling window length for the stationary detector.
    oak_stationary_window_s: float = 1.0
    # Per-axis raw gyro std gate: below this the window looks like noise, not motion.
    oak_stationary_gyro_std_dps: float = 0.3
    # Accel-norm std gate: catches bumps/vibration that gyro std alone can miss.
    # Tune this one on the robot — it is the most mounting-dependent threshold.
    oak_stationary_accel_std_g: float = 0.03
    # ABSOLUTE bound on the raw window mean per axis (not a residual against
    # the current bias estimate — that was chicken-and-egg: it could never
    # learn a bias larger than the bound itself). The std gates do the real
    # work; this only rejects a steady slow turn that would otherwise look
    # quiet AND passes a fresh wheels-stopped witness (see
    # ImuYawProducer.set_motion_witness / docs/heading_tuning.md — the witness
    # is what actually rejects a genuine slow turn now). Raised 2.0 -> 5.0 on
    # 2026-09-19: a raw mean above 5 deg/s while the wheels are stopped and the
    # sensor is quiet is not bias; a 2.6 deg/s hot bias must stay learnable.
    oak_stationary_max_rate_dps: float = 5.0
    # Bias relaxation time constant toward the stationary window mean.
    oak_stationary_bias_tau_s: float = 15.0
    # Freeze yaw integration entirely while provably stationary.
    oak_zupt_enabled: bool = True
    # Derive yaw_axis_sign from gravity at calibration (2026-09-19): the
    # gyro_y channel is assumed CW-positive because BMI270 +Y points DOWN on
    # the current, validated mount — a MOUNTING fact, not a software
    # constant. If the camera is ever re-mounted upside down, +Y points UP
    # and +gy silently becomes counter-clockwise. True: calibrate_gyro reads
    # gravity and flips yaw_axis_sign if needed. False: yaw_axis_sign stays
    # +1 and calibrate_gyro only logs what it would have done (check-only).
    # See OakImuReader._resolve_yaw_axis_sign_from_gravity /
    # docs/heading_tuning.md "Mount orientation".
    oak_yaw_axis_sign_auto: bool = True
    # OAK IMU yaw-rate source:
    # - "auto": lock onto dominant gyro axis while turning (can pick the wrong axis
    #   under vibration — chalk 90/180 with pi_app.cli.oak_yaw_chalk_test before
    #   trusting auto; prefer pinned gyro_x/y/z once measured)
    # - "gyro_x" / "gyro_y" / "gyro_z": force specific axis for diagnostics/production
    # - "gravity_projected": project gyro onto gravity vector
    # Field-validated 2026-07-12 with the hardened chalk harness. Pin gyro_y;
    # "auto" was unstable. Scale is neutral (1.0): host-side packet loss was the
    # under-report root cause — never mask sample loss with an empirical multiplier
    # like the old auto/0.46 pair. See docs/heading_tuning.md.
    oak_yaw_rate_source: str = "gyro_y"
    # Field-calibrated yaw-rate scale for OAK IMU heading integration.
    # Only meaningful for the axis actually selected when the scale was fitted.
    oak_yaw_rate_scale: float = 1.0
    # Optional IMU lever-arm mitigation toggle for A/B testing.
    # Keep disabled for the known-good setup above.
    oak_use_gravity_projected_yaw_rate: bool = False
    # Optional derivative-term EMA filtering (0.0 disables).
    dterm_ema_alpha: float = 0.3

    # Speed-dependent gain scheduling: at higher wheel speed, each byte of
    # correction produces more turning, so we attenuate the PID output.
    # scale = ref / max(speed, ref)  where speed = max distance-from-neutral.
    gain_schedule_enabled: bool = True
    gain_schedule_ref_speed_byte: float = 50.0

    # Fallback behavior
    fallback_on_error: bool = True  # Use RC control if IMU fails
    calibration_timeout_s: float = 5.0  # Timeout for IMU calibration


@dataclass(frozen=True)
class RcMapConfig:
    """RC mapping configuration for throttle channels (CH1/CH2).

    When RC pulse exceeds these thresholds, output saturates to full-scale.
    """
    forward_full_us: int = 1950  # >= maps to 255
    reverse_full_us: int = 1050  # <= maps to 0
    # Stick expo (2026-09-27, Kevin: small movements near the centre really
    # slow, the edge "just go"): y = (1 - e) x + e x^3 on each track's
    # deflection after the pulse-to-byte map. RC MANUAL only; the phone
    # teleop and the autonomous modes do not use it. At 0.6: quarter stick
    # gives 0.11 of full (linear 0.25), half stick 0.28 (0.50), three
    # quarters 0.55 (0.75), full stick unchanged. The straight-intent and
    # arming checks still read the raw pulses. 0.0 = linear.
    stick_expo: float = 0.6


@dataclass(frozen=True)
class VescConfig:
    # VESC expects electrical RPM (eRPM)
    # 20000 since 2026-09-19 (was 15000 -- an arbitrary cap, see
    # docs/gearing_memo.md). On the 18:50 follow-me run the wheels reached
    # ~13,000 eRPM at a VESC duty of only 0.58-0.65 (~22,000 eRPM per unit
    # duty), so 20000 still leaves ~10 percent of duty for the eRPM loop to
    # regulate under load. Top speed 1.2 -> 1.6 m/s in EVERY mode, RC manual
    # included. FollowMeConfig.speed_loop_mps_per_byte,
    # trail_speed_scale_mps_per_byte and slip_cmd_diff_per_byte scale with it.
    max_erpm: int = 20000

    # CAN IDs for each motor
    left_can_id: int = 2
    right_can_id: int = 1

    # Low-voltage watchdog — MOTOR CUTOFF only (no OS shutdown). Contract of
    # 2026-11-10 (docs/status_audit_2026-11-10.md section 3). When the TRUSTED
    # pack voltage (see pack_voltage_*) stays in [floor, threshold) for
    # voltage_shutdown_delay_s we ramp the motors to zero and raise a
    # recoverable pack-low latch. The latch must trip BEFORE the BMS does: the
    # BMS pack undervoltage trip is 40.0 V and its cell trip 3.000 V
    # (docs/battery_settings.md). A BMS trip opens the discharge FET, so drive
    # AND braking stop at once and the Pi halts 5 s later on UPS mains loss.
    # 44.0 V = 13 x 3.38 V. Worst case from the audit: 44.0 - 0.1 V reading
    # offset - 0.3 V drain over 30 s - 2.6 V sag (40 A x 65 mOhm) = 41.0 V at
    # the BMS, 1.0 V above its trip. 43.0 V leaves no margin. The old 39.0 V
    # was below the BMS trip and could never fire. 13S confirmed 2026-04-01.
    voltage_shutdown_threshold_v: float = 44.0
    # Plausibility floor: a reading BELOW this means the pack is disconnected /
    # switched off / sensor garbage. This 13S pack's BMS trips at 40.0 V, so it
    # can never genuinely sit this low — a reading below the floor is a normal
    # bench event (main battery switched off while the Pi runs on its UPS), NOT
    # a dying pack. Below the floor we stop motors and log once, but never latch
    # and never treat it as a low-voltage event (2026-07-10 false-trigger fix).
    voltage_shutdown_floor_v: float = 30.0
    # 30s sustain: a genuinely dying pack sags slowly, so a full 30s continuous
    # in-band reading is required before cutting motors (rejects transients).
    # (Was 10.0; contract changed 2026-07-10.) Not agreed (audit section 3,
    # item 6): a Grok review wanted 3-5 s. With today's VESC limits (40 A per
    # side, part A of docs/battery_settings.md not applied) a hard launch at
    # low charge sags the bus 2-3 V for a few seconds, so a 3-5 s delay can
    # latch on acceleration alone. Keep 30 s until part A is applied and the
    # bus-voltage logs show the real sag near 44 V.
    voltage_shutdown_delay_s: float = 30.0
    # A per-VESC bus reading (STATUS_5) older than this is not used. Same
    # 0.5 s as the telemetry staleness cut-off (vesc._BUS_STALE_S); each VESC
    # sends STATUS_5 at about 48 Hz, so 0.5 s is more than 20 missed frames.
    pack_voltage_max_age_s: float = 0.5
    # Two fresh readings that differ by more than this are UNTRUSTED: no latch,
    # no release, reported in telemetry. The pack voltage used is the MINIMUM of
    # the fresh raw readings (the low reader is the left one, within 0.1 V of
    # the BMS). The DIFFERENCE is taken between the two readings averaged with
    # pack_voltage_compare_tau_s, not between the latest raw frames.
    # History: 1.5 V (raw) until 2026-11-11; the stand test that evening
    # (docs/stand_test_2026-11-11.md) showed that the latest frames of the two
    # VESCs can be 0-40 ms apart while the free-wheel speed loop swings the bus
    # about 1 V in 20 ms, so the raw pair reached 2.1 V although readings taken
    # at the same moment differ by only 0.8-1.3 V (the right VESC's offset).
    # 3.0 V (raw) was the same-night stopgap (6ec19b9). With the average, the
    # stand data gave a gap of 0.96-1.22 V (0.88-1.53 V with the swings scaled
    # x2.5), so 2.0 V is the limit again. A left reader that reads high still
    # lets min() fall to the right reading (about 1 V high): the latch can be
    # up to 1 V late. That is true at any limit.
    pack_voltage_max_disagree_v: float = 2.0
    # Time constant (s) of the exponential average used only for the
    # disagreement check. Both readings use the same one, so a swing that one
    # VESC reports a frame earlier than the other cancels. 0 = compare the
    # latest raw readings (the behaviour before 2026-11-11). 0.25 s is about
    # 12 frames per VESC; a reader that really goes wrong shows in the
    # average within about 1 s (4 time constants).
    pack_voltage_compare_tau_s: float = 0.25
    # When ONLY the right VESC's reading is fresh it is used minus this. That
    # VESC reads 0.9-1.0 V high (battery_settings.md finding 7), so the
    # uncorrected value could hold a 43 V pack above 44 V. 1.0 V takes the
    # high end (latches earlier, never later). Set to 0.0 once the right VESC's
    # voltage reading is recalibrated in VESC Tool.
    pack_voltage_right_alone_offset_v: float = 1.0
    # RESTRICTED state (2026-11-10 follow-up): untrusted data never latches,
    # but a pack voltage that stays NOT CROSS-CHECKED for this long (stale, a
    # disagreement, a below-floor reading, or only one usable reader because
    # the other VESC is silent or frozen; a lone reader stuck high cannot be
    # caught) means the robot no longer knows its pack. Then autonomous modes
    # go to zero at once and MANUAL is capped at pack_low_limp_max_byte_offset. It is
    # NOT the latch: no low-voltage claim. A lone reader still latches.
    # 10 s: a CAN reopen or a VESC reboot is 1-3 s; ten times a frame gap is
    # still a short exposure at walking pace.
    pack_voltage_untrusted_limit_s: float = 10.0
    # RESTRICTED ends after this many seconds of cross-checked (both fresh,
    # agreeing) readings with zero throttle, or on a disarm while they are
    # cross-checked. 2 s = about 100
    # frames per VESC, the same as the sustain grace.
    pack_voltage_trust_confirm_s: float = 2.0
    # FROZEN reader: a fresh reading that has not changed at all for this long
    # while the other reader changed within pack_voltage_frozen_other_moved_s
    # is not used (the other one is used alone). 0 = OFF (the default).
    # OFF since 2026-11-10 (review): the readings step in 0.1 V, and at rest
    # one VESC often holds one value while the other flickers between two.
    # A replay of the real code over every armed log of 2026-11-06 to
    # 2026-11-08 and the parked night of 2026-11-10 found 1,600 "frozen"
    # episodes and 404 RESTRICTED episodes (8,000 s) with 20 s, among them
    # 5 min of armed MANUAL capped at the crawl speed; with 0 there were none.
    # Without this rule a frozen reader is only PARTLY covered: the guard uses
    # the LOWER reading, so a reader frozen high has no effect while the
    # other stays within pack_voltage_max_disagree_v of it. Once the pack
    # falls further the pair disagrees: untrusted, RESTRICTED after
    # pack_voltage_untrusted_limit_s (autonomous modes zero, MANUAL at the
    # crawl cap), but NO latch and no 42 V floor. The backstop then is the
    # BMS cell trigger (bms_cell_low_trip_v) while the BMS link is up, and the
    # BMS's own cut-off. Accepted (Grok 01a1cc96, 2026-11-11): latching on the
    # lower reading of a disagreeing pair would let one VESC stuck low strand
    # the robot on a healthy pack. A lone reader never releases a latch.
    pack_voltage_frozen_s: float = 0.0
    pack_voltage_frozen_other_moved_s: float = 2.0
    # Seeded countdown after a service start (default ON): if the first
    # cross-checked reading (both VESCs fresh and agreeing; lone-reader frames
    # before it do not count) is already below the threshold, the countdown is
    # pulled forward so the latch comes pack_low_seed_confirm_s later, not after a
    # fresh voltage_shutdown_delay_s of full drive. The resting voltage at
    # start carries no load sag, so below the threshold there means a low
    # pack; 2 s rejects a single bad first frame.
    pack_low_seed_on_start: bool = True
    pack_low_seed_confirm_s: float = 2.0
    # The sustain countdown PAUSES on a sample that is not trusted and in the
    # band (a disagreeing frame, a stale gap, a reading at or above the
    # threshold) and resets only after this many seconds of such samples in a
    # row. Without it one noisy frame at 29 s restarts the full 30 s, and a
    # pack hovering at the threshold under load could never latch. 2 s is
    # about 100 frames per VESC: a real recovery, not noise.
    pack_low_sustain_grace_s: float = 2.0
    # Release: the trusted pack voltage at or above threshold + this, with zero
    # commanded throttle on both tracks, in MANUAL. 2.0 V (46.0 V) is the old
    # anti-chatter band, kept (audit section 3, item 3). A disarm also releases.
    pack_low_release_hysteresis_v: float = 2.0
    # On a trip the motors ramp to zero over this time instead of a step to
    # RPM 0 from up to 1.31 m/s (ground speed, wheel_ground_scale 0.815). The ramp runs in VescCanDriver.set_tracks()
    # every control tick (~30 Hz), which also refreshes the VESC ~1 s command
    # timeout. A requested stop (stick to neutral, person stop) still goes to
    # zero at once: the ramp only limits how much speed is KEPT.
    pack_low_ramp_s: float = 1.0
    # The disarm that clears a pack-low latch ALWAYS starts a post-latch
    # session: MANUAL only (FOLLOW_ME / WAYPOINT_NAV go to zero at once), a fresh
    # countdown, a fresh cell count. It ends when the trusted pack is back at
    # threshold + hysteresis with zero throttle (or on a service restart).
    # Without limp, MANUAL drives at full speed and latches again after
    # voltage_shutdown_delay_s below 44 V. Limp home adds to that session:
    # speed capped at pack_low_limp_max_byte_offset, and the 44 V / 3.30 V
    # triggers replaced by the floors below. ON by the owner's decision of
    # 2026-11-10 (Kevin wants a slow MANUAL crawl home after a disarm on a
    # low pack). Autonomous modes never limp: they go to zero at once in the
    # session either way.
    pack_low_limp_enabled: bool = True
    # 32 bytes of 128 = about 0.33 m/s on the ground at max_erpm 20000 (0.40 m/s
    # in the old eRPM units, before wheel_ground_scale 0.815; walking pace). At that
    # speed the pack current is about 10 A, so the sag is about 0.7 V.
    pack_low_limp_max_byte_offset: int = 32
    # Hard floor in limp: 40.0 V BMS trip + 0.7 V limp sag + 0.1 V reading
    # offset + 1.2 V margin = 42.0 V (3.23 V per cell).
    pack_low_limp_floor_v: float = 42.0
    # The limp floor needs a shorter sustain than 30 s: limp current is low, so
    # the sag is small and a short delay does not trip on acceleration.
    pack_low_limp_delay_s: float = 5.0
    # Second trigger from the Daly BMS: lowest cell at or below this voltage on
    # bms_cell_low_consecutive_polls consecutive FRESH polls latches until a
    # disarm. 3.30 V is 0.30 V above the 3.000 V BMS cell trip (audit item 5).
    bms_cell_low_trip_v: float = 3.30
    # Limp replaces the cell trigger with this floor: 3.000 V trip + about
    # 0.05 V limp sag per cell + 0.10 V margin.
    bms_cell_limp_floor_v: float = 3.15
    # A BMS sample older than this is ignored (never a stop: a dark BLE link
    # must not stop the robot). The service polls every 8 s
    # (BmsConfig.bms_poll_interval_s) and a poll takes 1-3 s, so 20 s accepts
    # the latest poll and a late one, and nothing older. One MISSED poll can
    # pass 20 s; that breaks a run of fresh polls, which delays a cell trip
    # by about one poll (it never makes a false one).
    bms_cell_max_age_s: float = 20.0
    # Two consecutive fresh polls (about 16 s) so one sample taken during a
    # current spike, or one bad frame, cannot latch.
    bms_cell_low_consecutive_polls: int = 2
    # A cell reading below this is a parse error, not a cell: ignored.
    bms_cell_plausible_min_v: float = 2.5
    # A post-latch session that followed a BMS cell trip ends only when
    # bms_cell_low_consecutive_polls consecutive NEW fresh polls show the
    # lowest cell at or above this (the same bar as the trip), in addition to the
    # 46 V pack release (Grok 01a1cc96: a weak cell on a pack still above
    # 46 V ended limp on the first re-arm). 3.50 V x 13 = 45.5 V, so a
    # balanced pack at 46 V (3.54 V per cell) passes. A dark BMS keeps the
    # session (MANUAL only, limp cap) until a fresh poll or a service restart.
    bms_cell_release_v: float = 3.50

    # Wheel geometry + drivetrain for eRPM -> wheel speed (m/s) conversion.
    # wheel_radius_m: centre of wheel axle to contact patch (14.5in wheel dia).
    # motor_poles: total magnetic poles; pole_pairs = motor_poles // 2.
    # drive_gear_ratio: motor revs per wheel rev.
    #   6:1 gearbox * (20/14) chain stage * (40/10) chain stage = 34.2857:1
    wheel_radius_m: float = 0.18415
    motor_poles: int = 14
    drive_gear_ratio: float = 34.2857142857
    # Ground metres per nominal kinematic wheel metre, MEASURED (2026-11-10,
    # docs/wheel_speed_scale_2026-11-10.md). The three constants above read
    # 1.23x the GPS ground speed on straight steady drives (per-drive medians
    # 1.18-1.26 above 0.8 m/s, 1.24 pooled over 181 m of long segments, no
    # dependence on motor current at 2-4 A; March 2026 RTK on gravel gave
    # 1.256 too). The data cannot say whether radius, gearing or pole count
    # is wrong, so the geometry stays as labelled and this lumped factor
    # carries the correction. Uncertainty about +-0.03. Byte 254 = 20000 eRPM
    # = 1.31 m/s on the ground (not 1.61). Only the m/s VALUES change: the
    # VESC still runs the same eRPM for the same byte.
    wheel_ground_scale: float = 0.815

    # Set False to disable all closed-loop telemetry features (pure open-loop fallback).
    vesc_telemetry_enabled: bool = True

    # ── RPM plausibility gate (pi_app/control/rpm_plausibility.py) ──────────
    # "Commanded to move but eRPM reads ~0" for a sustained window means the
    # readback is dead (the 2026-06-11 lunge/stall cycle) or the wheel is
    # physically stalled. Either way the velocity PID must NOT act on it, so the
    # controller nulls rpm/actual_speed_mps (open-loop fallback, same path as
    # stale telemetry) until real RPM returns. Independent of the PID gains.
    rpm_plausibility_enabled: bool = True
    rpm_plausibility_min_cmd_bytes: int = 12   # |byte − 126| ≥ this is "non-trivial" (~1400 eRPM)
    rpm_plausibility_min_erpm: int = 150       # |eRPM| below this counts as "not moving"
    rpm_plausibility_window_s: float = 0.5     # implausible for this long → trip (covers spin-up)
    rpm_plausibility_hold_s: float = 2.0       # stay tripped at least this long (no chatter)

    # ── IMU motion witness (pi_app/control/rpm_plausibility.wheels_stopped) ──
    # Much tighter than rpm_plausibility_min_erpm (150) on purpose: that floor
    # answers "is this RPM reading credible", not "are the wheels stopped". A
    # real slow pivot (~1.5 deg/s ≈ 133 eRPM on this drivetrain) must read as
    # NOT stopped, so the ZUPT witness never freezes real rotation. VESC eRPM
    # reads exactly 0 at rest; 30 eRPM ≈ 0.3 deg/s per wheel.
    rpm_witness_min_erpm: int = 30


@dataclass(frozen=True)
class ObstacleAvoidanceConfig:
    """Configuration for OAK-D Lite depth-based obstacle avoidance."""
    enabled: bool = True
    slow_distance_m: float = 1.5
    stop_distance_m: float = 0.4
    roi_width_pct: float = 0.80
    roi_height_pct: float = 0.5
    roi_vertical_offset_pct: float = 0.0  # 0.0 for next test run (was -0.20; shifted up causing sky/horizon hits)
    camera_height_m: float = 0.497
    robot_width_m: float = 0.820
    # MEASURED from this camera's factory EEPROM 2026-07-26 via
    # `python3 -m pi_app.cli.oak_intrinsics`: CAM_A @ 640x400 gives fx=456.89,
    # implied HFOV 70.01 deg (EEPROM spec FOV 68.794 deg).
    #
    # This was 81.0, which is the OAK-D Lite colour sensor's DIAGONAL FOV pasted
    # into a HORIZONTAL field. That produced fx=374.67 — 18% too small — which
    # shrank the corridor threshold (fx * robot_half_mm) by the same 18%, so the
    # mask guarded a ~0.67 m wide robot instead of the real 0.82 m and ignored
    # obstacles ~7 cm inside each edge of its own swept path.
    #
    # This value is now only a FALLBACK: OakDepthReader.get_intrinsics() (through
    # _intrinsics_for()) reads the per-unit calibration off the device and uses
    # that. It matters only if the EEPROM read fails.
    camera_hfov_deg: float = 70.0
    min_depth_mm: int = 350            # OAK-D Lite extended disparity minimum (~0.35m)
    min_valid_pct: float = 8.0         # ignore corridor if fewer than this % of pixels are valid
    # A real obstacle at 1 m and 0.3 m wide covers tens of thousands of corridor
    # pixels; 400 px ≈ a 20x20 blob — anything smaller is noise.
    corridor_min_support_px: int = 400
    # A single-poll phantom cannot lower the corridor distance; a genuine
    # approaching obstacle is delayed by at most (this - 1) polls (~66 ms at 15 Hz).
    corridor_persistence_polls: int = 2
    # Near support as a FRACTION of the corridor pixel count; the larger of
    # this and corridor_min_support_px applies. 2026-09-19 19:53 (after
    # sunset, garage approach): p5 sat at 367-429 mm for two minutes while
    # the corridor median was 5-7 m and 10-13 percent of pixels were valid --
    # max-disparity noise reading the minimum measurable depth. That noise was
    # at least 5 percent of the VALID pixels (~560 px), so the 400 px floor
    # alone does not catch it. A real obstacle at stop range covers far more:
    # even a 5 cm pole at 0.4 m is ~57 px wide x the 200 px ROI = 11,000 px.
    # 0.02 of the ~102k px corridor = ~2,000 px.
    corridor_min_support_frac: float = 0.02
    # Speckle filter (2026-09-26, 45 raw frames in logs/depth_snapshots on
    # the robot). False stereo matches on repeated structures (iron gate
    # bars) put 1,000-2,000 near pixels in the corridor as sparse specks at
    # random depths (0.35-1.5 m) and reported a 0.70-0.81 m obstacle that
    # never came closer. A near pixel (<= slow_distance_m) stays only when
    # at least corridor_speckle_min_neighbours near pixels in its
    # corridor_speckle_window_px square lie within one 100 mm depth bin of
    # it. A blob-size rule was tried first and rejected: woven wire fence
    # looks like speckle at the pixel level, and with 30 percent of its
    # pixels removed (worse light) every blob variant lost the fence at 1 m.
    # This rule, over 252 frames (the 45 plus random and clumped dropout to
    # 70 and 60 percent), lost no real obstacle the old code saw and left no
    # gate phantom for any count from 12 to 22 at window 13; 16 is the
    # middle. Real obstacles kept at their range: gate bars 1.0 m, trash can
    # 0.87 m, wire fence ~1.0 m and 1.74 m (the old code read the fence's
    # own speckle as 0.58-0.73 m), sunlit siding (tape 1.346 m). The
    # corridor's density and support checks use the pixels before this
    # filter, and if too few pixels remain to measure, the unfiltered
    # reading stands. 0 disables.
    corridor_speckle_window_px: int = 13
    corridor_speckle_min_neighbours: int = 16
    # The filter runs only when the near pixels reach the support floor
    # (below it the reading is beyond slow_distance_m either way) and do not
    # exceed this (a near region this large can only be a real surface; the
    # unfiltered reading is the cautious one, and it bounds the cost: ~7 ms
    # per frame on the Pi 5 at ~2-12k near px, ~20 ms at ~90k).
    corridor_speckle_max_near_px: int = 20000
    # Person mask (2026-09-26): the corridor masks each person box so the
    # followed person does not count twice (persons already set the obstacle
    # distance through the stop tier). The whole-box mask also hid anything
    # between the robot and the person inside that outline. Now only pixels
    # at or beyond (person z - this margin) are masked; nearer pixels stay
    # in the corridor. A person with an unknown range is not masked. 0.0
    # restores the whole-box mask.
    person_mask_depth_margin_m: float = 0.5
    # MANUAL mode: the corridor stop is a floor, not a wall. The RC/phone
    # operator can always creep forward at this fraction of the stick (0.15
    # = ~0.24 m/s at full stick), so a phantom obstacle cannot strand the
    # robot (2026-09-19 19:53: could not drive into the garage, had to back
    # in). The YOLO person/animal stop tier (distance forced to 0.0) stays
    # absolute. 0.0 restores the hard stop.
    manual_obstacle_min_scale: float = 0.15
    # MANUAL throttle curve (2026-09-27 17:18-17:26, loading electric mesh
    # fencing): netting draped over the lens read as a solid 0.50 m obstacle
    # for 8 minutes, and the old MANUAL law (1.0 at slow_distance_m, linear
    # to 0 at stop_distance_m, then the 0.15 floor) held full stick to a
    # 15 percent crawl. Kevin's numbers: half throttle at the netting's
    # 0.50 m, the floor only from 6 in closer. MANUAL is now piecewise
    # linear through 1.0 at slow_distance_m, 0.5 at
    # manual_half_throttle_distance_m, and manual_obstacle_min_scale at
    # manual_floor_distance_m (0.35 m is also the camera's closest valid
    # depth, min_depth_mm). FOLLOW_ME and WAYPOINT_NAV keep the old law, and
    # the person/animal stop tier (distance 0.0) stays absolute.
    # manual_half_throttle_distance_m = 0.0 restores the old MANUAL law;
    # manual_obstacle_min_scale = 0.0 also does (with its hard stop).
    manual_half_throttle_distance_m: float = 0.50
    manual_floor_distance_m: float = 0.35
    # Netting mute (2026-10-08). Electric-fence mesh loaded on the robot
    # hangs over the lens and reads as a solid wall at 0.40-0.50 m (60 raw
    # frames: 97-98 percent of valid corridor pixels at 0.35-0.65 m), so
    # the MANUAL curve held Kevin at 0.3-0.4 throttle for 81 m. Depth alone
    # cannot tell the mesh from a wall, and an automatic "it moves with us"
    # latch was rejected in review (a close person YOLO misses, wheel slip
    # in front of a real wall, or anything being pushed satisfies the same
    # test). So the mute is EXPLICIT: the operator taps the phone with the
    # netting already over the lens (POST /api/manual/netting_mute), armed
    # and MANUAL, with the dense near field in view, or the tap is refused
    # (no waiting state: it cannot arm itself on the next close surface).
    # While engaged, MANUAL ignores the depth corridor only as long as the
    # field holds; a finite reading beyond netting_mute_max_distance_m
    # ends the mute on that tick (a fence that appears when the netting
    # lifts is never ignored), and only a dropout gets a hold of
    # netting_mute_absent_release_s. It also drops on any person or
    # stop/slow-tier animal detection, on disarm, on leaving MANUAL, after
    # netting_mute_timeout_s, or on a second tap; a new tap is required
    # after any drop. The YOLO 0.8 m stop (distance 0.0, its own channel)
    # and a stale depth are never muted. FOLLOW_ME and WAYPOINT_NAV never
    # use it. Review (Grok 2026-10-08): the cap sits ON the data, not above
    # it, because a real wall inside the cap is indistinguishable.
    netting_mute_enabled: bool = True
    netting_mute_timeout_s: float = 900.0          # Kevin's hauls run ~10 min
    netting_mute_absent_release_s: float = 0.5     # dropout hold only (~7 depth frames)
    # The netting band from the data: the 2026-10-08 drive read 0.40-0.50 m
    # for 2,229 of 2,235 ticks, the 2026-09-27 netting 0.50 m. Sunlit
    # siding reads 0.71-0.75 m, a wire fence 0.86 m.
    netting_mute_max_distance_m: float = 0.55
    # Share of the valid corridor pixels inside slow_distance_m: >= this at
    # the tap (the drive's median was 0.95) ...
    netting_mute_min_near_share: float = 0.85
    # ... and >= this to stay engaged (the share sat below 0.85 for 36
    # percent of the drive, in runs up to 69 s; below 0.70 for 0.1 percent,
    # 0.33 s at most).
    netting_mute_hold_near_share: float = 0.70
    update_rate_hz: float = 15.0
    stale_timeout_s: float = 0.5
    stale_policy: str = "stop"   # fail-safe: stop when depth data is stale
    manual_stale_throttle_scale: float = 0.10  # MANUAL mode: slow (not stop) when depth is stale
    safety_stop_radius_m: float = 0.8  # YOLO "stop" tier detections within this radius force min_distance->0 (applied in oak_depth depth poll)
    # Size backstop (2026-09-20, arm_20260920_154844.log): a genuinely close
    # person with no usable stereo still stops the robot. Shoulders 0.5 m at
    # 0.8 m subtend 0.45 of the 70 deg frame. A "stop"-tier detection whose
    # depth_status is not "ok" and whose bbox width >= this forces the hard
    # stop exactly like z < safety_stop_radius_m. 0.0 disables.
    safety_stop_bbox_width: float = 0.40
    # Person/animal stop HOLD (pi_app/control/person_stop_hold.py; the full
    # rules are in its docstring). The stop above had no memory: inside
    # ~0.4 m the box is clipped by the frame edge, the depth sample fails,
    # the corridor sees past the person, and one such frame released it
    # (2026-11-08 14:27:40, FOLLOW_ME, ~0.66 s with Kevin 0.3-0.9 m away; the
    # clipped box sampled the background at 2.6-2.7 m). The hold keeps
    # distance 0.0 after a trigger in FOLLOW_ME / WAYPOINT_NAV until the
    # object that caused it (one "chain") is seen beyond the release range,
    # the arm switch is off (live RC), or, after a drop to MANUAL, the stick
    # is let go. No timer releases it. Forward only: pivots and reverse pass.
    # A second object near the chain, a lost chain or two candidates make
    # the hold operator-only (accepted). False restores the stateless stop.
    person_stop_hold_enabled: bool = True
    # False: no hold ENGAGES in MANUAL; the stop there stays as it was.
    # Kevin's rule of 2026-09-27 (human input can carry a little extra risk;
    # the operator stops the robot by letting go of the stick), and the
    # netting: mesh over the lens reads a wall at 0.45 m and never shows
    # clear, so a MANUAL hold would need an arm-switch cycle each time the
    # camera sees Kevin close. A hold carried into MANUAL from an autonomous
    # mode stays applied until no drive command is given for
    # person_stop_hold_neutral_hold_s. Do NOT set True without Kevin's word.
    person_stop_hold_in_manual: bool = False
    # 0.2 m beyond safety_stop_radius_m, so range noise at the radius
    # cannot chatter the hold. After a release the stateless 0.8 m stop
    # still acts on every poll; FOLLOW_ME commands no forward inside 1.4 m.
    person_stop_hold_release_m: float = 1.0
    # 5 fresh polls = ~0.33 s at 15 fps. On 2026-11-08 14:27:40.6 YOLO lost
    # the side-clipped operator for 2 logged frames (~0.2 s) while the
    # corridor read 2.9-4.7 m.
    person_stop_hold_release_polls: int = 5
    # A person inside min_depth_mm gives invalid pixels, so the corridor is
    # accepted as clear only with this much valid support (8 percent, the
    # corridor's own floor, is too low). 2026-11-08 14:27: open-field
    # FOLLOW_ME read 0.5-18 percent, the clear yard after the stop 43-73.
    person_stop_hold_min_valid_pct: float = 20.0
    # A box within this of the left or right frame edge counts as clipped:
    # its range is never trusted (2026-11-08: "ok" 2.72 m at x1 = 0.001) and
    # its area is only a lower bound. YOLO clamps a clipped edge to 0 or 1;
    # on the 2026-11-07/08 drives the clipped sequences read 0.000-0.011.
    # 0.05 (32 px) is ~4x that and costs 41 of ~3,100 logged "ok" sightings
    # at >= 1.0 m (those 0.02-0.05 from an edge).
    person_stop_hold_edge_margin: float = 0.05
    # The chain (the object that caused the hold) continues on a box of the
    # same group (person / animal; YOLO calls one goat horse, sheep or cow)
    # seen at most this long after its last sighting and within this
    # normalized centre-x distance. The gap covers the 0.12-0.35 s detection
    # gaps of the 2026-09-20 runs with margin; the gate covers a full-rate
    # pivot (32 bytes x ~0.7 deg/s per byte = ~22 deg/s, 13 deg = 0.19 of
    # the frame in 0.6 s) plus a walking person. Past the gap the chain is
    # lost and the hold becomes operator-only.
    person_stop_hold_chain_gap_s: float = 0.6
    person_stop_hold_chain_gate: float = 0.30
    # The chain is resolved after this many fresh polls IN A ROW with its
    # own box seen beyond (~0.2 s at 15 fps), never on one box.
    person_stop_hold_resolve_polls: int = 3
    # Plausibility of a box continuing the chain (pi_app/control/
    # person_stop_hold.py, PersonStopHold._plausible). Between two trusted
    # ranges |dz| <= speed * dt + margin; from the implied reference (no
    # trusted range yet: the stop radius) z <= 0.8 + speed * dt + margin.
    # The apparent area may shrink or grow only as 1/z^2 allows over that
    # reach, times/divided by area_factor for YOLO box jitter. Speed: a
    # walk is ~1.4 m/s and neither autonomous mode reverses during a hold.
    # Logs 2026-11-07/08 (three drives, 2,198 pairs of consecutive
    # single-object trusted sightings <= 0.6 s apart): max |dz| - 1.5 * dt
    # = +0.002 m (p99.9 -0.010), so the 0.25 m margin is all range-noise
    # headroom; with these numbers no pair failed the area test, and the
    # worst pair needed an area factor of 1.07 (1.5 used). Rejected by
    # these bounds (the named review cases): 2.0 m 0.6 s after a measured
    # 0.5 m (reach 1.15 m), a carry-in 2.8 m 1.0 s after a 0.5 m or an
    # unmeasured seed (reach 1.75 m; 2.25 / 2.55 m).
    person_stop_hold_plausible_speed_mps: float = 1.5
    person_stop_hold_plausible_margin_m: float = 0.25
    person_stop_hold_plausible_area_factor: float = 1.5
    # Carry-in: the first poll in FOLLOW_ME / WAYPOINT_NAV engages the hold
    # when the last stateless trigger (any mode) is at most this old. The
    # mode level reaches the vision thread up to one control tick (33 ms) +
    # one depth poll (67 ms) late; 1.0 s also covers a start with a person
    # already blind-close whose trigger flickers with the detection gaps
    # (<= 0.35 s). Short so a MANUAL trigger minutes before a start (Kevin
    # beside the robot) does not block it. The carried-in box keeps its
    # REAL sighting time; this is also its continuation gap.
    person_stop_hold_carry_in_s: float = 1.0
    # The control loop writes the hold's levels every tick (30 Hz). Older
    # than this, the reader assumes autonomous, no operator release and the
    # stick not neutral, and logs it once at WARNING.
    person_stop_hold_levels_max_age_s: float = 1.0
    # MANUAL "stick let go" for a hold carried into MANUAL: BOTH commanded
    # tracks (before the obstacle layer, RC or phone) within this many bytes
    # of neutral (6 bytes = ~25 us of stick, the auto-disarm deadband) ...
    person_stop_hold_neutral_band_byte: float = 6.0
    # ... continuously for this long. Reverse or a pivot is a drive command.
    # A stick swept from forward to reverse crosses the band in one or two
    # 33 ms ticks; 0.25 s (~8 ticks) needs a real let-go.
    person_stop_hold_neutral_hold_s: float = 0.25


@dataclass(frozen=True)
class FollowMeConfig:
    """Configuration for autonomous person-following mode."""
    enabled: bool = True
    # 1.5 -> 1.2 on 2026-09-26: Kevin asked for a 4 ft stop (the 1.5 m
    # setting came to rest at 1.6 m, "a good 5 ft"). Forward speed is zero
    # inside follow_distance_m + speed_dead_zone_m (1.4 m); the robot coasts
    # to about 1.3 m. The top-clipped-box exemption in DetectionFilter keeps
    # the operator in the filter at this range (a standing adult overflows
    # the 42 deg frame inside ~2.3 m); the person stop tier is at 0.8 m.
    follow_distance_m: float = 1.2        # desired following distance in metres
    min_distance_m: float = 0.5
    # 6.0 -> 8.0 on 2026-09-26 (arm_20260926_125554.log): the operator
    # walked away at ~1.55 m/s, the robot was pinned at its 1.38 m/s cap,
    # and at 6.1-6.6 m DetectionFilter dropped him on range alone (YOLO
    # 0.83-0.89, depth_status "ok", same track id, width 0.058-0.077), so
    # the robot stopped. The torso sampler already reads to 9.0 m
    # (person_depth_sample_max_m). detect_min_bbox_width 0.05 now binds
    # first, near 7-7.5 m for a front-facing adult.
    max_distance_m: float = 8.0
    # Identity gates decide WHOM to follow; they must never hide a closer
    # range from the speed command (2026-09-20 15:49: tracker held 3.0 m
    # and rejected a true 1.2 m edge reading as an "occluder", then the
    # speed layer drove at the held 3.0 m). When True, speed uses
    # min(target.depth_m, nearest raw in-range person). False restores
    # pre-fix behaviour (speed from the locked target only).
    nearest_person_speed_limit_enabled: bool = True
    # Only a range closer than the speed law's current depth (smoothed
    # target.depth_m) by more than this engages the limit. Comparing against
    # raw_depth_m disabled the cap exactly when DepthFilter was still holding
    # a far range after the tracker accepted a close one (H3, 2026-09-20).
    nearest_person_margin_m: float = 0.3
    # Depth-unknown + geometry says near: a standing adult overflows the
    # 42 deg vertical frame inside ~2.3 m, so bbox height at/above this
    # with depth_status != "ok" forces the speed law to follow_distance_m
    # (zero forward). 0.0 disables.
    close_unknown_bbox_height: float = 0.85
    # Top speed 110 -> 128 bytes on 2026-09-26 (same log as max_distance_m):
    # 110 capped follow-me at 1.38 m/s while the robot was 3-6 m behind a
    # walking operator. 128 is byte 254 = max_erpm 20000 = 1.61 m/s (nominal
    # kinematics; about 1.31 m/s on the ground, see wheel_ground_scale), the
    # same top speed as full RC stick. The VESC reached 17,200 eRPM at duty
    # 0.71 on a full 53.5 V pack, so 20,000 eRPM needs ~0.82 there; below
    # ~46 V it saturates at max duty (about 1.5 m/s), as full stick does.
    # max_speed_error_m 1.5 -> 1.75 keeps the approach gain at
    # 128 / 1.75 = 73.1 bytes per metre (was 110 / 1.5 = 73.3), so the
    # speed law has the validated slope; follow_distance_m 1.2 only shifts
    # it 0.3 m closer (each command equals the old law's command 0.3 m
    # farther out, within 0.3 percent). Speed pinned at the cap also leaves no L/R headroom
    # above 254, so at top speed the mixer clips the outer track and a
    # small steer correction gives about half the yaw of the same
    # correction at 110 (turn_speed_scale still slows real turns).
    max_speed_error_m: float = 1.75   # distance error at which max speed is reached — tighter = more aggressive closing
    max_follow_speed_byte: int = 128
    # Legacy direct-pursuit PD gains (preserved; not used by new PID steering path)
    steering_gain: float = 0.50
    steering_derivative_gain: float = 0.06  # calibrated from Phase 2 plant model
    steering_ema_alpha: float = 0.3        # smooths x_m before derivative (0=heavy, 1=none)
    detection_confidence: float = 0.45    # minimum YOLO confidence to accept a detection
    lost_target_timeout_s: float = 3.5  # tolerate medium turn/occlusion dropouts without full stop

    # ── Layer 1: Detection filter ─────────────────────────────────────────────
    min_bbox_area: float = 0.0015        # minimum bbox area (fraction of frame); rejects tiny far detections
    # Geometric rejection rules — tuned from video analysis of a real follow-me run where
    # YOLO (conf 0.90-0.92) boxed a vertical sliver of the tracked person at the extreme
    # right frame edge (xmin 0.90-0.94, width 0.06-0.10) after they were lost, causing
    # spurious re-acquisition.  Legitimate person detections in that recording never
    # exceeded xmin 0.60 and were wider (≥ 0.10) and tall (implied_h 1.68–3.02 m).
    # Set any value to 0 (or ≤ 0) to disable that individual check.
    detect_edge_margin: float = 0.15       # reject xmin > (1-margin) or xmax < margin; 0 = disabled
    # 0.09 -> 0.05 on 2026-09-19: the 18:50 run lost the operator at 4.3 m --
    # a 0.45 m wide person is 0.086 of the 70 deg frame at 4.4 m -- with YOLO
    # still at 0.86-0.90 confidence, and every loss stopped the robot. The
    # edge rule above already rejects the frame-edge slivers this was added
    # for; 0.05 keeps a front-facing adult to ~7-7.5 m, inside
    # max_distance_m 8.0 (measured 0.058-0.062 at 6.4-6.6 m on 2026-09-26).
    detect_min_bbox_width: float = 0.05   # reject normalized bbox width < this; 0 = disabled
    detect_min_person_height_m: float = 1.20  # reject implied physical height < this (m); 0 = disabled
    # MEASURED from the factory EEPROM 2026-07-26 (`pi_app.cli.oak_intrinsics`):
    # CAM_A at the 640x352 YOLO input frame gives fy=456.89 -> VFOV 42.13 deg.
    #
    # This was 65.3, derived as 2*atan((240/320)*tan(81/2)) from the same wrong
    # 81 deg diagonal-as-horizontal value. Because implied_h scales with
    # tan(vfov/2), that inflated EVERY implied height by
    # tan(32.65)/tan(21.065) = 1.66x.
    #
    # Consequence: Rule 3 in DetectionFilter ("reject short ground blobs
    # (animals)") nominally rejects anything under detect_min_person_height_m
    # = 1.20 m, but at 1.66x inflation the real cutoff was 1.20/1.66 = 0.72 m.
    # A large dog clears 0.72 m. That is the same failure mode the sticky-lock
    # knobs above were added to defend against ("a chicken YOLO labels person").
    # At 42.13 deg the 1.20 m gate is finally the gate it claims to be, and a
    # 1.75 m adult still clears it across the whole 0.5-6.0 m follow range.
    #
    # NOTE: this is the VFOV of the DETECTION frame (640x352), not of the RGB
    # preview (640x480 -> 54.88 deg). If the NN input size changes, remeasure.
    detect_camera_vfov_deg: float = 42.13

    # ── Layer 2: Target tracker ───────────────────────────────────────────────
    target_ema_alpha: float = 0.5        # EMA smoothing on normalized horizontal offset (0=heavy, 1=none)
    target_persistence_s: float = 2.0   # hold last known position this long before declaring target lost
    # Sticky-target-lock knobs (defend against misclassified intruders, e.g. a
    # chicken YOLO labels "person"). Plain config — intentionally NOT autotuned.
    # SWITCH GRACE: when the committed target is momentarily absent from this
    # frame's candidates, HOLD it (coast, signal not-fresh) for this long instead
    # of switching to a different/closer candidate. Only after grace expires may a
    # new target be acquired.
    target_switch_grace_s: float = 1.5
    # ACQUIRE FLOOR: acquiring a NEW target requires confidence >= this (a higher
    # bar than the per-frame detection_confidence=0.45 filter). An already-committed
    # target stays followed even if its confidence later drops below this.
    target_acquire_confidence: float = 0.65
    # SUSTAINED ACQUISITION: a challenger must qualify (>= acquire_confidence) for
    # this many consecutive frames before it becomes the committed target. Filters
    # flickering high-conf blips.
    target_acquire_min_frames: int = 3
    # SUSTAINED HAND-OFF (2026-09-19): when the committed target is absent and a
    # DIFFERENT floor-clearing candidate is present, that challenger must stay the
    # best qualifier for this long (and target_acquire_min_frames) before it takes
    # the lock. Previously the hand-off was immediate, so a closer person stepping
    # in front of the operator (occluding them for one frame) stole the lock on
    # that frame. 0.0 restores the immediate hand-off. Must be < switch_grace_s
    # to matter — after grace expires the ordinary lost → re-acquire path runs.
    target_switch_min_s: float = 1.0
    # DEPTH CONTINUITY: a candidate only counts as "my committed target" if its
    # depth is within (continuity_m + rate_mps × seconds since last seen) of the
    # held depth. This is what tells a closer person at the operator's x apart
    # from the operator, and what rejects a tracklet id that got transferred onto
    # an occluder. 0.6 m base: a person cannot close 0.6 m between two 30 Hz
    # frames; a stereo-depth glitch that large costs one not-fresh tick, no more.
    # 0.0 disables the gate (pure id / x continuity, the pre-2026-09-19 rule).
    target_depth_continuity_m: float = 0.6
    target_depth_continuity_rate_mps: float = 1.5
    # DEPTH-UNKNOWN COAST (2026-09-20): when the stereo sampler says it does
    # not know the range (depth_status != "ok", z_m = 0), the tracker may
    # CONTINUE an existing lock on bbox x and hold the last known depth.
    # After this many seconds of unknown range the frame is treated as
    # not-fresh so persistence decay stops the robot. A candidate that HAS
    # a range and fails the depth-continuity gate is still rejected.
    target_depth_coast_max_s: float = 1.0

    # ── Host-side tracklet layer (IoU + constant-velocity Kalman) ─────────────
    # Assigns stable track_ids to person detections on parse paths that the OAK
    # device does not already track (host-side YOLOv8 NeuralNetwork parse and the
    # raw SpatialDetectionNetwork path). Lets follow_me lock onto a specific person
    # across frames / brief occlusions instead of just "closest by depth".
    tracklet_iou_threshold: float = 0.3  # min IoU (predicted bbox vs detection) to match a tracklet
    tracklet_min_hits: int = 3           # consecutive-ish hits before a tracklet is confirmed (reports its id)
    tracklet_max_age: int = 15           # frames a tracklet survives unmatched before deletion (~1 s @ 15 fps)
    # DEPTH GATE on association (2026-09-19): a detection may only be matched to
    # a tracklet if its depth is within (gate_m + growth × frames unmatched) of
    # the tracklet's last depth. Stops an occluder's box (bigger, overlapping,
    # closer) from inheriting the operator's id by IoU alone. 0.0 disables.
    tracklet_depth_gate_m: float = 0.75
    tracklet_depth_gate_growth_m_per_frame: float = 0.10  # ≈1.5 m/s of allowed closing at 15 fps
    # CENTRE-DISTANCE FALLBACK (2026-09-20 run, arm_20260920_145718.log): the
    # box is narrow at range (width 0.10 of the frame at 4 m), detections
    # arrive with 0.12-0.35 s gaps, and the robot yaws at 20-35 deg/s, so
    # predicted vs new detection IoU is 0 and TrackletTracker mints a new
    # id for the same person. 35 deg/s × 0.35 s = 12 deg = 0.175 of the
    # 70 deg frame. A second association pass matches unmatched CONFIRMED
    # tracklets by centre distance. 0.0 on both knobs disables the fallback.
    tracklet_center_gate_min: float = 0.20
    tracklet_center_gate_width_mult: float = 1.5

    # ── Layer 3: Lateral PID steering ────────────────────────────────────────
    # Error = normalized horizontal offset (-1.0 to +1.0); output scales to ±max_steer_offset_byte.
    # Plant from the heading derivative vs L-R (2026-09-19 and 2026-09-20):
    # 0.65-0.71 deg/s per L-R byte, ~0.45 s lag, r = 0.89-0.93. The 0.22
    # deg/s figure used on 2026-09-19 was an artifact of zero-filled
    # yaw-rate samples (64% of imu.yaw_rate_dps reads were 0.0 while
    # gy_body_dps was 18-20 deg/s; the producer still integrated heading).
    # That retune raised kp 0.4 -> 0.8, max_steer_offset_byte 25 -> 40,
    # direct_mode_max_steer_byte 18 -> 40 and the robot wove with growing
    # amplitude. 2026-09-20: kp 0.5, max 32, slew 0.25 (offline sim on a
    # 0.70 deg/s/byte plant with 0.15-0.30 s dead time). Close-range
    # behaviour is still tame: steer_deadband_norm and steer_edge_knee
    # are unchanged.
    pid_lateral_kp: float = 0.5
    pid_lateral_ki: float = 0.0
    pid_lateral_kd: float = 0.2
    pid_lateral_integral_limit: float = 0.5  # anti-windup clamp (normalised units)

    # ── Depth EMA filter (stabilises stereo depth at long range) ────────────────
    depth_ema_alpha: float = 0.35          # EMA smoothing on raw depth (0=heavy, 1=none)
    depth_max_velocity_mps: float = 5.0    # reject readings implying > this speed (m/s)

    # ── Person stereo sampler (2026-09-20, arm_20260920_154844.log, git 41f4fdf)
    # OAK-D Lite is PASSIVE stereo: outdoors only 5-10 percent of depth pixels
    # are valid. The old inner-50% median with a 6.0 m clip and no support
    # floor produced three stop-the-robot failures:
    #   (1) operator at 5.8-5.9 m, bbox 0.05-0.07 of the frame wide, SAME
    #       track id: z_m jumped 5.8, 3.7, 3.1, 5.8, 4.1, 2.6, 5.8 because
    #       the person's own pixels exceeded the 6000 mm clip and the median
    #       landed on a few stray pixels;
    #   (2) bbox at the right frame edge (x=[0.82, 1.00], often full-frame
    #       height ymin=0.00 ymax=1.00) read 1.2, 1.0, 0.8 m. The operator
    #       confirmed he really WAS about 1 m away (2026-09-20 15:49): those
    #       stereo values were true, not a border artifact. 1.2 m is outside
    #       safety_stop_radius_m 0.8, so the robot kept driving.
    #   (3) bbox near the left edge (x=[0.08, 0.27]) read 1.5-1.7 m then
    #       2.8 m one frame later.
    # DetectionFilter still applies min_distance_m / max_distance_m later;
    # the sampler must not discard the person's own pixels at 0.3-0.5 m
    # (H2a) or at 6.0-6.5 m.
    person_depth_sample_min_m: float = 0.30      # NOT min_distance_m; stereo floor on this camera is ~0.35 m
    person_depth_sample_max_m: float = 9.0
    person_depth_min_valid_px: int = 12          # absolute support floor
    person_depth_min_valid_frac: float = 0.02    # fraction of torso-ROI pixels
    person_assumed_height_m: float = 1.75        # standing adult; z_height_m diagnostic only, never a source of z
    # Impossible-FAR veto (H2b, safe direction only: discarding a far range
    # cannot hide a close person). A standing adult overflows the 42.13 deg
    # vertical frame inside ~2.3 m. When the box is clipped at BOTH top and
    # bottom (ymin <= 0.01 and ymax >= 0.99) and z_stereo_m is above this,
    # the median is background seen past a close person: depth_status
    # "far_veto", z_m = 0.0 (z_stereo_m still reported). Downstream treats
    # any status other than "ok" as unknown. Documented here because
    # follow_me.PersonDetection's comment list is owned by another file.
    person_depth_fullheight_max_m: float = 3.5
    # Bimodal ROI (H2c). If p75-p25 exceeds max(this, 0.35 * z_stereo_m)
    # the ROI holds two surfaces and the median is not a measurement:
    # depth_status "ambiguous", z_m = 0.0. This is the 5.8 -> 3.7 -> 2.6 m
    # flap at long range.
    person_depth_max_spread_m: float = 1.0
    # Height-consistency veto is OFF (0.0 disables). z_height_m is still
    # computed and logged as a diagnostic. Do not enable this without field
    # evidence of a real too-close stereo artifact on a box that is NOT
    # clipped top/bottom. The 2026-09-20 15:49 incident (true ~1.2 m
    # readings at the frame edge, robot kept driving) showed that a veto
    # which discards a true close range is a safety hazard; its only
    # supporting evidence is gone.
    person_depth_height_veto_ratio: float = 0.0

    # ── Layer 4: Speed (depth-based, closed-loop when VESC telemetry available) ─
    speed_dead_zone_m: float = 0.2       # ±dead_zone around follow_distance_m → speed = 0 (no oscillation)

    # Velocity PID — closes the speed loop using measured wheel RPM.
    # Error = target_wheel_speed_mps − actual_speed_mps, where
    #   target_wheel_speed_mps = open_loop_bytes × speed_loop_mps_per_byte
    #   actual_speed_mps       = avg |eRPM| → m/s (controller.py, same kinematics)
    # Output (m/s) is converted back to a byte correction with the SAME scale and
    # added to the open-loop feed-forward. Falls back to open-loop whenever
    # actual_speed_mps is None (no VESC, stale frames, or the RPM plausibility
    # gate in VescConfig tripped).
    #
    # HISTORY: gains were zeroed 2026-06-11 because dead RPM readback (always 0)
    # made the loop integrate a bogus "not moving" error → lunge/stall cycle.
    # Bench 2026-06-11 (tools/vesc_rpm_bench.py) then proved ERPM readback works
    # (−1 % / −0.4 % steady-state error at 1500 eRPM, 0 parse errors).
    # RE-ENABLED 2026-09-19 with two guards that make the old failure impossible:
    #   1. VescConfig.rpm_plausibility_*: commanded-but-0-eRPM → telemetry nulled
    #      → open-loop. The PID never sees the bogus zero for more than window_s.
    #   2. speed_pid_max_correction_mps clamps the loop's authority to ±0.163 m/s
    #      (±15.9 bytes; 0.20 m/s in the pre-2026-11-10 units), so even a wrong error can only nudge, never lunge.
    #
    # SCALE: the old loop mixed two scales — target from the GPS-measured GROUND
    # speed (trail_speed_scale 0.0075 m/s/byte, gravel scrub included) against a
    # measured WHEEL speed. That is a permanent ~20 % "too fast" error that the
    # integrator would pull the throttle down against. speed_loop_mps_per_byte is
    # the kinematic wheel speed per byte (VescConfig max_erpm / poles / gearing /
    # radius; docs/gearing_memo.md §a) — the same numbers that produce
    # actual_speed_mps — so target and feedback agree by construction.
    # test_rpm_plausibility pins it to the VescConfig derivation.
    #
    # GAINS: the VESC already runs its own eRPM loop (CAN_PACKET_SET_RPM), so in
    # free running the wheel tracks the command and this outer loop sees ~0 error.
    # It only works when the VESC cannot reach the commanded eRPM (load, current
    # limit, soft ground). Gentle gains, no derivative (eRPM at 20 Hz poll is too
    # noisy to differentiate). kp 0.6: a 0.1 m/s shortfall adds 0.06 m/s ≈ 6 bytes.
    # ki 0.15 with a 1.0 m/s·s integral clamp: ≤ 0.15 m/s of integral authority.
    speed_kp: float = 0.6
    speed_ki: float = 0.15
    speed_kd: float = 0.0
    # The two clamps below were tuned in the old (over-reading) wheel units on
    # 2026-09-19/20: 1.0 m/s·s and 0.20 m/s. On 2026-11-10 both were scaled by
    # VescConfig.wheel_ground_scale (0.815) so that they are worth the same
    # motor bytes as before and the validated behaviour is unchanged:
    #   integral authority ki × limit / k = 0.15 × 0.815 / 0.010232 = 11.9 bytes
    #   correction ceiling          0.163 / 0.010232               = 15.9 bytes
    # test_wheel_ground_scale.py::TestIdenticalMotorBytes pins this.
    speed_integral_limit: float = 0.815  # anti-windup clamp (m/s·s); 1.0 × 0.815 (was 50 — absurd with the clamp)
    speed_pid_max_correction_mps: float = 0.163  # |closed-loop correction| ceiling: 0.20 × 0.815 (= 15.9 bytes)
    # 0.012555 -> 0.010232 on 2026-11-10: the same derivation now includes
    # VescConfig.wheel_ground_scale (0.815). The loop is ratiometric (target
    # and feedback share this scale, kp and ki are m/s per m/s), so with the
    # two clamps scaled the follow-me bytes do not change at all.
    speed_loop_mps_per_byte: float = 0.010232325  # = 0.012555 × 0.815 exactly (1.310 m/s / 128), so every byte matches the old loop — see SCALE above

    # ── Slip detection & compensation ─────────────────────────────────────────
    # Hard off-switch. When False the slip compensator is a TRUE no-op: it returns
    # (speed, steer) unchanged — no throttle reduction, no steer feed-forward. This
    # is the shipped default after the 2026-06-13 runaway (below) so robot behavior
    # is byte-identical with slip out of the picture. Flip to True only after Kevin
    # tunes the commanded-vs-actual detector on hardware.
    slip_compensation_enabled: bool = False
    # Detection now compares the ACTUAL rpm differential against the differential
    # the EMITTED steer command should have produced (commanded-vs-actual), instead
    # of acting on raw |left_rpm − right_rpm|. On a skid-steer robot every turn
    # creates an rpm differential, so the old raw-diff test read a commanded turn as
    # "slip" and injected more steer → bigger differential → runaway. The new test
    # subtracts the commanded differential, so a deliberate turn no longer registers.
    slip_threshold_rpm: float = 200.0       # |slip_diff| (actual−expected) eRPM to declare slip
    slip_throttle_reduction: float = 0.15   # throttle scale-back fraction (0–1), gated by enable flag
    # Rough proportional model mapping emitted steer (bytes) → expected L−R eRPM
    # differential: expected_diff = slip_cmd_diff_per_byte * commanded_steer. It is
    # deliberately crude — its only job is to cancel the commanded component so a
    # turn isn't mistaken for slip. Calibrate on hardware; over-estimating is safe
    # (it just makes the detector less sensitive), under-estimating less so.
    slip_cmd_diff_per_byte: float = 53.3
    # The "going straight" guard must hold for this many consecutive ticks (on the
    # EMITTED/commanded steer, not the pre-correction PID value) before slip can
    # act — so a transient never triggers and the guard releases the instant a real
    # turn begins.
    slip_straight_persist_ticks: int = 3
    # Dedicated ceiling for any slip steer feed-forward, well below the global
    # ±max_steer_offset_byte and the direct cap (both 32 since 2026-09-20; they
    # were 25 and 18 when this was written). The TOTAL
    # post-slip steer is additionally clamped to direct_mode_max_steer_byte so slip
    # can never push the emitted steer past the direct cap.
    slip_max_steer_byte: float = 12.0
    # DISABLED 2026-06-13: zeroed after a Follow-Me steer runaway. VESC RPM
    # telemetry went live this day (was always 0), activating slip comp for the
    # first time. The OLD steer feed-forward was positive feedback on a skid-steer
    # robot — any turn creates an RPM differential, which injected more steer,
    # which increased the differential — so it pinned steer to the ±max and lost
    # the target. The detector was rewritten to commanded-vs-actual (above) and the
    # whole compensator gated behind slip_compensation_enabled (default False).
    # This gain stays meaningful (slip steer term = slip_diff * gain) but irrelevant
    # while disabled. See fm_trials 1781382327 and the steer=25-on-direct signature.
    slip_feedforward_gain: float = 0.0      # steer correction per eRPM slip_diff (bytes/RPM)

    # ── Motor output rate ─────────────────────────────────────────────────────
    follow_output_rate_hz: float = 15.0  # motor commands at this Hz, decoupled from 30 fps vision
    # Allow continued blind trail pursuit longer than short target-drop timeout.
    # This is the key behavior needed to keep moving around corners after LOS loss.
    lost_target_trail_pursuit_max_s: float = 3.0
    max_steer_offset_byte: float = 32.0          # 25 -> 40 on 2026-09-19 (bad 0.22 plant); 40 -> 32 on 2026-09-20, see pid_lateral_kp (was 15 before that)
    # Search-rotation steer magnitude (bytes) while pivoting in place to reacquire
    # a lost target. Was previously read via a phantom getattr fallback (30.0)
    # that had no backing field and EXCEEDED the caps of the time
    # (max_steer_offset_byte 25, direct_mode_max_steer_byte 18). Since 2026-09-20
    # both caps are 32, so the search pivot (20) is now SOFTER than direct
    # pursuit. That is by design: search is gentler than pursuit. The value has
    # been 20 since 2026-07-05 and was the live value in the validated runs of
    # 2026-09-20. The (slew-capped) emission gate still clamps everything.
    search_steer_cap_byte: float = 20.0
    # Gain on the trail-tangent/last-known-position bias blended into blind
    # trail-pursuit steer as elapsed lost-time grows (anticipates the turn the
    # person was making). Was previously read via a phantom getattr fallback
    # with no backing field; 3.0 matches that fallback exactly, so behavior is
    # unchanged — it's now a real, discoverable config knob.
    lost_steer_bias_gain: float = 3.0

    # Trail-following Pure Pursuit (breadcrumb path instead of direct pursuit)
    # OFF since 2026-11-10 (Kevin's decision). GpsOdometry keeps a math-angle
    # heading, but its camera_to_world and PurePursuitController use the
    # clockwise dead-reckoning frame, so trail pursuit steers the wrong way
    # after a pivot. Its heading is also the boot-relative gyro, not true
    # north (docs/status_audit_2026-11-10.md, section 8). GPS odometry is
    # selected whenever the GPS has a fix, so this hit trail pursuit beyond
    # 5.2 m, blind trail pursuit after a lost target, and the search-pivot
    # direction. None of those ran in any armed log up to 2026-11-08.
    # With False, the trail, both odometry objects and pure pursuit are not
    # built: FOLLOW_ME is direct pursuit only, and a lost target gets the
    # zero-forward search pivot toward the last camera position, then a stop.
    # Do NOT set True again before GpsOdometry is repaired, has frame tests,
    # and has a field test.
    trail_follow_enabled: bool = False
    # m/s per motor byte above neutral (trail odometry, pure-pursuit lookahead).
    # 0.0075 calibrated on gravel via RTK GPS at max_erpm 15000 (2026-03-28),
    # scaled 4/3 for 20000 on 2026-09-19. Checked 2026-11-10 against the GPS
    # track on 30 s windows of the 2026-11-06/08 drives (RTK float): best fit
    # 0.0100-0.0105, per-drive fits 0.0095-0.0109, so 0.0100 stands. (The VESC
    # eRPM speed, not this byte scale, reads 20-25 % above GPS.)
    trail_speed_scale_mps_per_byte: float = 0.0100
    # Which odometry builds the follow-me trail: "dead_reckon" (gyro heading +
    # motor-byte speed, boot-relative frame) or "gps" (GPS fix + gyro with a
    # course-over-ground offset, true north-east frame). One source only: the
    # frames differ. "dead_reckon" since 2026-11-10: on 30 s windows of the
    # 2026-11-06/08 drives its end-to-end error was 0.85 m median (5 % of the
    # path); GPS odometry instead carries 1.8-2.1 m steps at every fix-quality
    # change, a 1 Hz position, and no pose at all until a straight forward
    # run has taught it the heading offset. Both obey the frame contract in
    # pi_app/control/odometry.py.
    trail_odometry_source: str = "dead_reckon"
    trail_max_points: int = 100
    trail_min_spacing_m: float = 0.3
    trail_max_age_s: float = 30.0
    trail_consume_radius_m: float = 0.4
    trail_max_step_m: float = 3.0             # reject impossible breadcrumb jumps (robot+person both moving)
    trail_max_speed_mps: float = 30.0         # effectively disabled — GPS 1Hz jumps cause false rejections; max_step_m=3.0 catches real outliers
    pursuit_wheelbase_m: float = 0.28             # track width wheel-to-wheel
    direct_pursuit_distance_m: float = 4.0        # switch to trail pursuit sooner for better path tracking
    direct_pursuit_lateral_m: float = 1.0         # allow larger lateral offset before switching to trail mode
    min_trail_points_for_pursuit: int = 2

    # Adaptive lookahead: lookahead = clamp(speed_mps * time_s, min_m, max_m)
    pursuit_lookahead_time_s: float = 0.8
    pursuit_lookahead_min_m: float = 0.5
    pursuit_lookahead_max_m: float = 2.5

    # Trail path smoothing (Savitzky-Golay)
    trail_smoothing_enabled: bool = True
    trail_smoothing_window: int = 5               # must be odd, >= 3
    trail_smoothing_poly_order: int = 2           # must be < window

    # Curvature-based velocity scaling
    pursuit_curvature_scaling_enabled: bool = True
    pursuit_curvature_alpha: float = 5.0          # higher = more deceleration in turns
    pursuit_min_speed_byte: float = 15.0          # floor speed in tight turns
    pursuit_lookahead_curvature_points: int = 10  # look ahead for pre-deceleration (increased for gravel speed)
    pursuit_max_accel_byte_per_s: float = 50.0    # smooth speed transitions

    # ── Steer hold/decay during detection dropout ─────────────────────────
    steer_hold_decay_s: float = 1.0  # seconds to decay held steer to 0 after losing fresh detection (speed-aware: shrinks at high speed)
    # Hold last fresh speed/steer unchanged this long after a dropped frame so a
    # single 15 fps miss does not already scale speed by 0.78 (then the accel
    # ramp needs ~0.5 s to recover).
    steer_hold_grace_s: float = 0.30
    # Replaces the previous literal 0.3 floor on the speed-aware decay window.
    steer_hold_decay_speed_floor: float = 0.5

    # ── SafetyLayer acceleration cap ─────────────────────────────────────────
    max_speed_accel_byte_per_s: float = 150.0  # max speed ramp-up rate (bytes/s) inside Follow Me SafetyLayer

    # ── Direct pursuit steering cap ──────────────────────────────────────────
    direct_mode_max_steer_byte: float = 32.0   # 18 -> 40 on 2026-09-19 (bad 0.22 plant); 40 -> 32 on 2026-09-20, see pid_lateral_kp; the mixer clips at 254 anyway
    # Direct-pursuit forward-speed reduction while the person is off-centre.
    # The mixer clips at 254. True yaw authority is 0.65-0.71 deg/s per L-R
    # byte (~0.45 s lag) from the heading derivative; the 0.22 figure was
    # zero-filled yaw-rate samples. Below the knee, scale is 1.0 (byte-identical).
    direct_turn_speed_knee_norm: float = 0.30
    direct_turn_speed_min_scale: float = 0.35

    # ── Steer deadband and slew limiter ──────────────────────────────────────
    steer_deadband_norm: float = 0.04   # |x_err| below this → treat error as 0 (suppresses gait-wobble chasing; ~2-3% frame width)
    steer_slew_per_tick: float = 0.25   # 0.1 -> 0.25 on 2026-09-20: 8 bytes per 15 Hz tick at max 32; less actuator lag was the cheapest stability win in the sim

    # ── Edge-boost steering gain (direct PID path only) ───────────────────────
    # Amplifies the PID input proportionally as the person drifts toward the
    # frame edge, so the robot fights harder to recentre before losing the lock.
    # Only the DIRECT path is affected; trail pursuit is unchanged.
    # Gain formula: edge_gain = 1 + boost * clamp01((|x_err| - knee) / (1 - knee))
    # At |x_err| <= knee  → edge_gain = 1.0  (center behavior byte-identical)
    # At |x_err| = 1.0    → edge_gain = 1 + boost
    # Set boost=0.0 to disable entirely (full no-op, back-compat).
    steer_edge_boost: float = 1.5   # extra proportional gain at the frame edge (0 disables; error x(1+boost) at |x|=1)
    steer_edge_knee: float = 0.4    # |normalized_x| below this gets NO boost (preserves gentle, anti-wobble center)

    # ── Re-acquisition / search / mode-switch timing ─────────────────────────
    reacq_slew_window_s: float = 0.5           # ramp steer from 0 → full over this window after a detection dropout
    search_mode_delay_s: float = 1.5           # wait this long after trail exhaustion before entering search mode
    trail_exhausted_remaining: int = 3         # trail point count below which trail is considered exhausted
    mode_switch_dwell_s: float = 1.5           # minimum dwell time before switching between trail/direct pursuit modes

    # Trail/direct steering blend: when person is off-center in trail mode,
    # blend in direct pursuit steering so robot reacts to WHERE the person IS.
    trail_direct_blend_start_m: float = 5.0  # effectively disabled — was causing corner cutting
    trail_direct_blend_full_m: float = 10.0  # effectively disabled — was causing corner cutting

    # Person-position bias: blend live detection into trail pursuit steering
    # so robot reacts to where the person IS, not just the historical path.
    # 0.0 = pure trail, 1.0 = pure direct PID. 0.35 = 35% toward live person.
    trail_person_bias_weight: float = 0.35

    # GPS-based trail odometry (preferred over dead reckoning when RTK fix available)
    gps_cog_min_speed_mps: float = 0.5        # min speed for GPS COG heading to be trusted
    gps_heading_alpha: float = 0.85            # complementary filter: higher = trust gyro more
    gps_cog_min_delta_m: float = 0.05          # min position change to compute COG


@dataclass(frozen=True)
class SlewLimiterConfig:
    """Final-stage motor command slew limiter configuration."""
    enabled: bool = True

    # Mode-aware asymmetric limits in byte-units per second.
    # MANUAL is intentionally quicker than autonomous modes.
    manual_accel_bps: float = 250.0
    manual_decel_bps: float = 350.0
    follow_me_accel_bps: float = 200.0
    follow_me_decel_bps: float = 250.0
    waypoint_nav_accel_bps: float = 140.0
    waypoint_nav_decel_bps: float = 220.0

    # Hard-stop behavior: bypass slew limiter when a stop-critical governor is active.
    bypass_on_hard_stop: bool = True
    hard_stop_scale_threshold: float = 0.0
    # If bypass is disabled, this fast decel cap can still be used by caller logic.
    emergency_decel_bps: float = 2000.0

    # First armed command after neutral/disarm can either snap to target or ramp from neutral.
    snap_first_command: bool = True
    snap_first_follow_me: bool = False  # Follow Me always ramps from neutral


@dataclass(frozen=True)
class OakRecordingConfig:
    """Configuration for activity-triggered OAK-D recording (video + MCAP)."""
    enabled: bool = True
    recording_dir: str = "logs/oak"

    # Trigger: record whenever obstacle avoidance is active or Follow Me is on
    pre_buffer_s: float = 2.0         # ring-buffer seconds kept before trigger fires
    post_event_linger_s: float = 3.0  # keep recording N seconds after last trigger
    obstacle_trigger_scale: float = 0.95

    # H.265 video (on-device encoding, near-zero CPU cost)
    video_enabled: bool = True
    video_bitrate_kbps: int = 3000

    # MCAP annotated snapshots + telemetry
    mcap_enabled: bool = True
    mcap_image_fps: float = 3.0       # annotated RGB snapshot rate
    mcap_depth_fps: float = 1.0       # colorized depth snapshot rate
    mcap_telemetry_hz: float = 5.0
    # If true, MCAP image snapshots are recorded only for follow/person contexts.
    mcap_images_follow_only: bool = True
    # Local preview generation budget (used by web viewer + MCAP image capture path)
    preview_rgb_fps: float = 10.0
    preview_depth_fps: float = 8.0

    # Storage management
    max_total_mb: int = 4000
    max_age_days: int = 3


@dataclass(frozen=True)
class OakWebViewerConfig:
    """Configuration for the live web viewer served from the Pi."""
    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = 8080
    rgb_stream_fps: float = 10.0
    depth_stream_fps: float = 8.0
    telemetry_hz: float = 8.0


@dataclass(frozen=True)
class GpsConfig:
    """Configuration for DFRobot GNSS-RTK rover module (I2C)."""
    enabled: bool = True
    i2c_bus: int = 1
    i2c_addr: int = 0x20
    # Renamed from update_rate_hz 2026-09-19 (Commit D): the GPS module
    # itself updates at ~1 Hz. Polling at 5 Hz and deduping on the UTC
    # second (not the data-flush register) reliably catches every epoch
    # without the old poll-rate aliasing. See rtk_gps.py's _poll().
    poll_hz: float = 5.0
    stale_timeout_s: float = 3.0
    # How often (in epochs, not poll ticks) to read the RMC/VTG sentence for
    # course/speed over ground. 1 = every epoch.
    cog_read_every: int = 1
    read_nmea_sentences: bool = True
    # min_quality removed 2026-09-19 (Commit D): the reader used to silently
    # drop any epoch below this quality. Every reading now publishes with
    # its real fix_quality; consumers already gate themselves
    # (WaypointNavController.accepts_fix_quality, GpsHeadingAligner's
    # fix_quality != min_fix_quality check), so a blanket reader-side drop
    # just hid low-quality fixes from anything that might want to see them
    # (e.g. this logging work). Grepped for other readers first: none.


@dataclass(frozen=True)
class WaypointNavConfig:
    """Configuration for autonomous GPS waypoint navigation."""
    enabled: bool = True
    waypoint_file: str = "waypoints.json"
    arrival_radius_m: float = 1.0
    cruise_speed_byte: int = 40
    approach_speed_byte: int = 20
    slow_radius_m: float = 2.0
    # Quality 4 means RTK fixed and is matched exactly; quality 5 is RTK float.
    # Other configured values retain minimum-threshold behavior.
    min_rtk_quality: int = 4
    stale_timeout_s: float = 3.0
    # State-machine thresholds (see WaypointNavController)
    align_threshold_deg: float = 12.0      # |heading_err| below this -> ALIGN->DRIVE
    recovery_threshold_deg: float = 25.0   # |heading_err| above this -> DRIVE->ALIGN
    # ALIGN pivot is proportional to the heading error (2026-09-19): full
    # pivot_yaw_cmd at pivot_full_error_deg and above, floor pivot_yaw_min near
    # the window. A fixed 0.5 pivot measured ~74 deg/s and overshot 25-30 deg,
    # past recovery_threshold_deg, so ALIGN/DRIVE oscillated left-right.
    pivot_yaw_cmd: float = 0.35            # max normalized yaw command during ALIGN pivot
    pivot_yaw_min: float = 0.18            # floor so the tracks keep moving near the window
    pivot_full_error_deg: float = 90.0     # error at which the pivot reaches pivot_yaw_cmd
    motor_deadband_byte: int = 12          # minimum byte offset to overcome motor deadband
    # Heading guard (2026-11-10 status audit, item 3). A run whose heading
    # is older than this leaves WAYPOINT_NAV, and activation is refused.
    # Age = time since the compensator's last good IMU read plus the age of
    # the sample that read returned. 0.5 s matches OakImuReader's
    # _STALE_AGE_S: past it the reader freezes the heading, so a pivot
    # would turn with an unchanged heading error.
    heading_stale_s: float = 0.5
    # ALIGN (pivot in place) gives up after this long in one ALIGN episode
    # and leaves WAYPOINT_NAV. From the pivot rates: the field pivot was
    # ~74 deg/s at yaw 0.5, so ~52 deg/s at pivot_yaw_cmd 0.35 and ~27 deg/s
    # at the pivot_yaw_min 0.18 floor. The worst case, 180 deg, is ~1.7 s
    # (180->90 at full pivot) + ~1.2 s (proportional, 90->46 deg) + ~1.3 s
    # (floor, 46->12 deg) = ~4.1 s. 10 s is ~2.4x that, and still covers a
    # pivot at half those rates on grass (~8.2 s). Time paused for stale or
    # untrusted GPS inside ALIGN counts too.
    align_timeout_s: float = 10.0


@dataclass(frozen=True)
class GestureConfig:
    """Configuration for hand-gesture activation/deactivation of Follow Me."""
    enabled: bool = True
    activation_sequence: tuple = (3, 4, 3)   # finger counts to start Follow Me
    stop_gesture: str = "FIVE"               # open palm to stop Follow Me
    hold_frames: int = 12         # consecutive frames a gesture must be stable
    sequence_timeout_s: float = 3.0  # max seconds between sequence steps
    cooldown_s: float = 2.0       # ignore gestures briefly after activate/deactivate
    # True restores the old always-when-armed MediaPipe Hands poll.
    # False (default): skip Hands in FOLLOW_ME unless the gesture machine
    # is in phase ACTIVE (the only phase that honours FIVE). FOLLOW_ME from
    # the RC switch leaves the machine in IDLE, so Hands cannot act
    # (2026-09-20: 20-50 ms per loop on the shared vision thread).
    hand_poll_in_follow_me: bool = False


@dataclass(frozen=True)
class ArmsUpConfig:
    """Body-pose both-wrists-above-shoulders detector and garage bench twitch.

    The agreed trigger is BODY POSE (both wrists raised above the shoulders),
    not palms/fingers: the 640x480 stream cannot resolve a hand beyond ~0.5 m.

    ``enabled`` permits the feature (an AND gate). The bench twitch itself is
    a runtime latch (``Controller.request_twitch_test``), not a config flag.
    That latch is volatile, requires the robot armed in MANUAL at the moment
    of enabling, and clears on disarm, mode change, expiry, and budget. It
    must not be reused for the continuous back-up feature, which needs its
    own latch and review.

    22 bytes * ~0.0126 m/s per byte * 0.25 s is under 7 cm before ramp
    losses ("inches"). Reverse offset is clamped to [0, 30] bytes and
    duration to [0, 0.4] s at use.
    """
    enabled: bool = True
    min_visibility: float = 0.6
    wrist_above_shoulder_frac: float = 0.25
    hold_s: float = 0.4
    release_s: float = 0.3
    stale_s: float = 0.5
    # A hold counts only consecutive new samples this close together.
    # A longer gap, or an update() call this late, restarts the streak.
    max_sample_gap_s: float = 0.25
    min_hold_samples: int = 3
    # Wrist y below crop_y0 + this fraction of the crop height is "pinned
    # at the crop top" and is not a trigger.
    crop_edge_margin: float = 0.03
    pose_max_hz: float = 10.0
    # Asymmetric person-crop expand, fractions of the mapped box.
    # x is the total widening (split across both sides).
    crop_expand_x_frac: float = 0.20
    crop_expand_up_frac: float = 0.35
    crop_expand_down_frac: float = 0.05
    # Person-detection list older than this vs the preview frame is dropped.
    max_det_age_s: float = 0.3
    twitch_test_budget: int = 3
    twitch_test_max_s: float = 300.0
    twitch_reverse_byte: int = 22  # offset below neutral 126, applied to BOTH motors
    twitch_duration_s: float = 0.25
    twitch_cooldown_s: float = 3.0
    # Per-mode command must sit within this many bytes of neutral to start
    # or to keep a pulse. The robot must also have been emitting neutral
    # for twitch_min_still_s, and both track eRPMs must be present and at
    # or under twitch_max_still_erpm. A missing eRPM blocks the start.
    twitch_stick_neutral_band: int = 6
    twitch_min_still_s: float = 0.5
    twitch_max_still_erpm: float = 300.0


@dataclass(frozen=True)
class BmsConfig:
    """Configuration for Daly BMS Bluetooth communication (SPIM08HP)."""
    enabled: bool = True                   # enabled -- BMS MAC confirmed 2026-04-01
    bms_mac_address: str = "50:19:05:01:09:4E"  # JHB-50190501094E discovered 2026-04-01
    bms_poll_interval_s: float = 8.0       # seconds between full polls
    bms_timeout_s: float = 30.0            # BLE connect timeout; also fail-open threshold
    charger_inhibit_enabled: bool = True   # refuse drive commands when charging
    # Discharge FET safety: after arm, ignore FET=False for this long (normal BLE startup delay ~14s)
    bms_fet_grace_period_s: float = 20.0
    # After grace period, if discharge_fet_on stays False for this many seconds, force neutral
    bms_fet_safety_timeout_s: float = 2.0
    # Charge-detect debounce (2026-06-13 field bug): a momentary regen/noise blip on
    # pack_current_a (e.g. +1.3A while driving at full speed) must NOT be mistaken for
    # a real charger. Require current above this threshold, sustained across
    # charge_detect_min_consecutive_polls consecutive successful polls, before
    # is_charging() reports True and the controller inhibits drive.
    charge_detect_min_current_a: float = 2.0
    charge_detect_min_consecutive_polls: int = 2


@dataclass(frozen=True)
class PropertyMapConfig:
    """Configuration for the property map feature."""
    enabled: bool = True
    image_path: str = "property_map.jpg"
    calibration_path: str = "map_calibration.json"
    max_serve_width: int = 4096
    trail_max_points: int = 500


# COCO 80-class names, indexed by YOLOv8 label ID
_COCO_CLASS_NAMES: tuple = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
)


@dataclass(frozen=True)
class OakDetectionConfig:
    """Configuration for OAK-D object detection model.

    Supports YOLOv8n (80 COCO classes, anchor-free, ~28 FPS on OAK-D Lite)
    and MobileNet-SSD v2 (21 VOC classes) as a fallback.
    """
    # "yolov8n" for 80-class COCO detection; "mobilenet-ssd" for legacy VOC detection
    model_type: str = "yolov8n"
    # Local .blob path — empty string pulls from Luxonis model hub (uses model_type as name)
    model_path: str = "models/yolov8n_640x352.blob"
    # Network-level confidence threshold. Currently equal to the Follow Me threshold
    # (FollowMeConfig.detection_confidence); Follow Me applies its own post-filter on top.
    confidence_threshold: float = 0.45
    nms_threshold: float = 0.45
    # Input resolution — 640x352 (≈16:9) uses the full OAK-D Lite horizontal FOV
    input_width: int = 640
    input_height: int = 352
    # COCO class names indexed by YOLOv8 label ID (80 classes)
    coco_classes: tuple = _COCO_CLASS_NAMES
    # Detection-based obstacle safety tiers (YOLO COCO class IDs).
    # STOP: people and large animals — halt immediately
    stop_class_ids: tuple = (0, 15, 16, 17, 18, 19)   # person, cat, dog, horse, sheep, cow
    # SLOW: smaller moving objects — reduce speed
    slow_class_ids: tuple = (29, 32, 36, 37)           # frisbee, sports ball, skateboard, surfboard
    # LOG ONLY (2026-09-28): every detection of these classes goes into the
    # per-tick log as "animals" -- label, confidence, range, box, tier and
    # depth status -- with no behaviour change. Kevin has ducks, chickens,
    # geese (YOLO "bird", class 14, not in any tier yet), dogs and cats. The
    # data sizes the planned animal slow zone for FOLLOW_ME/WAYPOINT_NAV
    # (docs/NEXT_SESSION.md): how often and how far YOLO sees the flock, and
    # its false "bird" hits. () turns the field off.
    log_animal_class_ids: tuple = (14, 15, 16, 17, 18, 19)  # bird, cat, dog, horse, sheep, cow
    # 2026-09-20 field runs: person dets reach steering ~0.6 s old
    # (det_latency_s p50 0.80 s at det_fps 8 in follow-me, 0.61-0.66 s at
    # det_fps 11 idle; IMU yaw vs bbox-centre cross-correlation 0.60-0.65 s
    # on three runs). The YOLO NN input used depthai v3's default queue size
    # 3 with setBlocking(False): camera ~30 fps, NN ~10, so the NN always
    # worked a 3-deep FIFO (~300 ms) behind. 1 keeps only the newest frame.
    # 0 = leave the library default.
    nn_input_queue_size: int = 1
    # The vision loop slept 1/update_rate_hz unconditionally (67 ms) after
    # its work, so work 60 ms + sleep 67 ms = 8 Hz — the measured det_fps.
    # True = sleep only the remainder of the period; False = old sleep.
    vision_deadline_sleep: bool = True
    # _poll_depth ran before _poll_detections, ageing each detection by the
    # corridor computation. True = detections first. The corridor already
    # masks with last-published boxes and the safety-tier override already
    # reads last-published detections.
    poll_detections_first: bool = True
    # 0 = unspecified (today's requestOutput calls, no fps=). Reserved for
    # the next A/B; when > 0 pass fps= to every Camera.requestOutput in the
    # YOLO pipeline (colour NN, colour preview/hand, both mono outputs).
    camera_fps: float = 15.0
    # 2026-11-06: the OAK's USB 3 (SuperSpeed) link jams the RTK GPS; the
    # antenna is within 30 cm of the hub. Measured outdoors: camera idle,
    # 20-23 satellites with RTK float; OAK streaming at USB 3, 10-11
    # satellites and no RTK, and a GPS that restarts cannot acquire at all
    # (0 satellites for 4 min, shield on the cut extension repaired). OAK
    # streaming at USB 2: 32-34 satellites, RTK float. On this cable the plug
    # orientation picks the speed, so the cap is set here, not by the plug.
    # "HIGH" = USB 2 (480 Mbit/s), "SUPER" = USB 3, "" = depthai default.
    # Before "SUPER": re-test the satellite count with the camera streaming.
    max_usb_speed: str = "HIGH"
    # Log-only visual-inertial odometry (depthai BasaltVIO on the Pi,
    # 2026-11-06). False = the camera session exactly as before (UNCALIBRATED
    # IMU reports, no flip, no VIO node). True, on depthai >= 3.6.1 only: the
    # IMU node requests RAW (BasaltVIO diverges on UNCALIBRATED, see
    # docs/depthai_upgrade.md section 5) and OakDepthReader._poll_imu negates
    # y and z so the heading still gets the UNCALIBRATED (RDF) values. VIO
    # shares the mono outputs (camera_fps) and the 100 Hz IMU, logs to the
    # "vio" block, and never reaches the controller. After switching it on:
    # the hand-turn heading sign test, the boot gravity line, and an idle
    # oak.det_latency_s A/B (camera_fps is load-bearing for latency).
    vio_enabled: bool = False
    # Today's NeuralNetwork.setNumInferenceThreads value. Reserved for the next A/B.
    nn_inference_threads: int = 2
    # Today's NeuralNetwork.setNumShavesPerInferenceThread value.
    nn_shaves_per_thread: int = 4
    # 2026-09-25 17:43: CAM_A (the YOLO input and the preview) stalled after
    # 23 h. Mono and depth kept running at 15 fps. get_person_detections()
    # kept returning one frozen person (x -0.8 m, z 2.52 m, track 453) for
    # 40 min while get_health() already reported detections_stale. At 18:23
    # FOLLOW_ME treated that list as live and drove ~0.85 m/s in a left
    # circle for 44 s. Follow-me must not drive when the detection stream
    # is older than this. The stored list is left unchanged, so a stale
    # close person still forces the person/animal stop.
    vision_stale_s: float = 1.5
    # 2026-09-25 17:43: CAM_A (the YOLO input and the preview) stalled
    # after 23 h. Mono and depth kept running, and no exception was
    # raised, so the session never ended. End the session when no fresh
    # NN packet has arrived for this long and let the supervisor rebuild
    # it. 0 disables the watchdog.
    color_stall_restart_s: float = 5.0
    # No watchdog restart in the first seconds of a session.
    color_stall_warmup_s: float = 10.0
    # After this many watchdog restarts inside color_stall_window_s,
    # stop restarting and latch a fault. Depth stays up; follow-me keeps
    # refusing the stale stream. Restart the service to try again.
    color_stall_max_restarts: int = 3
    color_stall_window_s: float = 900.0
    # IMU stall watchdog. 2026-11-06 19:34:37: the BMI270 stream stopped
    # reaching the Pi in a healthy session, with no error. Depth (10-15 fps)
    # and detections (15 fps) kept running, so nothing ended the session,
    # and the heading stayed frozen for 38.8 h (two drives on 11-07) until
    # an unrelated colour-stall rebuild on 11-08 10:21:56 brought it back in
    # the same process. End the session when no IMU message has arrived for
    # imu_stall_restart_s while depth is fresh and the session is past
    # imu_stall_warmup_s, and let the supervisor rebuild it.
    # Why 10 s: the journal (2026-09-19..11-08) has 32,739 "IMU samples
    # stale" episodes that recovered by themselves. In 32,517 of the 32,591
    # same-process pairs the next episode started within 10 s, so each of
    # those drops was shorter than that; the 10-22..10-24 run, depth fresh,
    # dropped every ~4.4 s for 40 h, so each drop was shorter than 4.4 s.
    # 10 s is more than twice that (a rebuild costs ~8 s of depth and
    # detections), and it catches a dead stream in seconds instead of days.
    # 0 disables the watchdog.
    imu_stall_restart_s: float = 10.0
    # No IMU restart in the first seconds of a session (same as the colour
    # watchdog; the first IMU message of a session arrives within ~3.4 s).
    imu_stall_warmup_s: float = 10.0
    # The IMU watchdog acts only while depth is newer than this. When depth
    # is stale too, the whole device is stalled: that is the reconnect and
    # stale-depth path, not an IMU-only stall.
    imu_stall_depth_fresh_s: float = 1.5
    # An IMU stale this long is an "oak_imu_stall" event in the JSON log
    # (pi_app/app/imu_stall.py). Longer than every self-recovering drop of
    # the 10-22..10-24 run (each under ~4.4 s, one every ~4.4 s for 40 h,
    # which would otherwise write ~20,000 events a day), and half of
    # imu_stall_restart_s, so the stall is marked before the restart.
    imu_stall_event_s: float = 5.0
    # The IMU watchdog shares the colour watchdog's budget: at most
    # color_stall_max_restarts session restarts by EITHER watchdog inside
    # color_stall_window_s, so the two cannot multiply each other's
    # restarts. Past that the watchdog that wanted the restart latches its
    # own fault (color_stall_fault / imu_stall_fault) until its stream
    # delivers again.
    # Vision worker hang (2026-10-04 13:41 -> 2026-10-08 17:36). After a USB
    # hub glitch the colour-stall watchdog's third restart called
    # pipeline.stop() and depthai's teardown never returned. The worker
    # thread sat inside C++ for 4 days with pipeline_running still True, so
    # no supervisor ran, depth was 4 days stale, and MANUAL drove at
    # manual_stale_throttle_scale ("extremely slow") with nothing in the
    # journal but the status line. A thread stuck in depthai cannot be
    # interrupted from Python; the only recovery is a process restart
    # (the wall-e unit is Restart=on-failure). The worker heartbeat is the
    # newer of the session loop tick and the supervisor loop tick, so a
    # device-absent backoff (supervisor ticking every 10 s) is not a hang.
    # vision_hang_s: heartbeat older than this => hung (0 disables both).
    vision_hang_s: float = 30.0
    # Exit the process for a systemd restart after this long hung, and
    # only while DISARMED: an operator never loses MANUAL mid-drive, and
    # FOLLOW_ME / WAYPOINT_NAV are already halted by the stale depth.
    # 0 = log only, never exit.
    vision_hang_restart_s: float = 60.0
    # Journal reminder cadence while hung (the health transition line
    # prints once; this keeps shouting).
    vision_hang_log_interval_s: float = 60.0
    # Restart gate (Grok review 2026-10-08): cmd.is_armed also goes False on
    # a 1 s RC dropout with the switch still up, so the gate reads the RAW
    # ch3 level with a fresh RC link, blocks on a short dropout (the switch
    # may still be up), and allows after this long without RC (transmitter
    # off: RC-stale forces disarm and phone teleop needs RC armed, so nobody
    # can drive). The permitted state must also hold continuously for
    # vision_hang_restart_settle_s before the exit, so a switch flipped high
    # in the last second always wins.
    vision_hang_rc_off_s: float = 30.0
    vision_hang_restart_settle_s: float = 10.0
    # Self-restart budget: at most this many exits per window, remembered in
    # logs/vision_hang_restarts.json across restarts, so a hang that recurs
    # on every boot (hub flapping) ends in "log only, restart by hand"
    # instead of an endless 95 s restart loop. 0 = no cap.
    vision_hang_restart_budget: int = 3
    vision_hang_restart_window_s: float = 3600.0


@dataclass(frozen=True)
class SafetyConfig:
    """Arming watchdog separate from the RC ch3/ch5 level checks.

    2026-09-24 overnight-armed incident: the RC transmitter was left on
    with the arm switch (ch3) up for 23 hours.
    """

    # Seconds armed and idle before auto-disarm. 0 disables.
    auto_disarm_idle_s: float = 600.0
    # ch1 and ch2 must each sit within this many microseconds of 1500 µs
    # to count as idle.
    # Matches pi_app.control.mapping.DEADBAND_US: a stick beyond this moves
    # the robot, so it must not count as idle.
    auto_disarm_stick_deadband_us: int = 25
    # A fresh start (boot, deploy, crash restart) arms only after the arm
    # switch has been seen OFF. Without it, a restart after an auto-disarm
    # would re-arm on a switch that was left up. main.py passes this to the
    # Controller; a Controller built directly (unit tests) defaults to off.
    require_switch_off_at_startup: bool = True


@dataclass(frozen=True)
class Config:
    """Main configuration class."""
    
    # IMU steering configuration
    imu_steering: ImuSteeringConfig = ImuSteeringConfig()
    # RC mapping config
    rc_map: RcMapConfig = RcMapConfig()
    # Armed-idle auto-disarm (2026-09-24 overnight-armed incident).
    safety: SafetyConfig = SafetyConfig()
    # VESC config
    vesc: VescConfig = VescConfig()

    # OAK-D Lite obstacle avoidance
    obstacle_avoidance: ObstacleAvoidanceConfig = ObstacleAvoidanceConfig()

    # RTK GPS
    gps: GpsConfig = GpsConfig()

    # Waypoint navigation
    waypoint_nav: WaypointNavConfig = WaypointNavConfig()

    # GPS-derived heading alignment (locks from a forward manual run; the
    # offset is then kept for the life of the process, including disarm)
    gps_heading_align: GpsHeadingAlignConfig = GpsHeadingAlignConfig()

    # Follow Me person-tracking mode
    follow_me: FollowMeConfig = FollowMeConfig()

    # Hand-gesture Follow Me activation
    gesture: GestureConfig = GestureConfig()

    # Both-wrists-up body-pose detector (bench twitch; real reverse-follow later)
    arms_up: ArmsUpConfig = ArmsUpConfig()

    # Final motor-output slew limiter
    slew_limiter: SlewLimiterConfig = SlewLimiterConfig()

    # OAK-D recording (video + MCAP)
    oak_recording: OakRecordingConfig = OakRecordingConfig()

    # Live web viewer (MJPEG stream + recordings browser)
    oak_web_viewer: OakWebViewerConfig = OakWebViewerConfig()
    
    # IMU source: "auto" (try external I2C first, fall back to OAK-D),
    # "external" (I2C breakout only), "oak_d" (OAK-D onboard BMI270 only), "none"
    # Pinned to the OAK BMI270 on 2026-09-19. The external ISM330DHCX +
    # MMC5983MA path (pi_app/hardware/imu_reader.py) is unvalidated, has no
    # tests, and publishes the opposite yaw-rate sign convention to the OAK
    # path, so "auto" (external first) must not be able to hijack heading if
    # that board is plugged in. Re-enable only after the bench protocol in
    # docs/external_imu_postmortem.md.
    imu_source: str = "oak_d"

    # File paths
    imu_calibration_path: str = "imu_calibration.json"
    # Optional magnetometer axis map to align mag with accel/gyro frame.
    # Each element: 'x','-x','y','-y','z','-z'. None selects sensible defaults per sensor.
    imu_mag_axis_map: tuple[str, str, str] | None = ('x', 'y', '-z')
    # Heading convention: True = heading increases clockwise (compass style)
    imu_heading_cw_positive: bool = True
    # Whether to use the magnetometer for yaw/heading fusion
    imu_use_magnetometer: bool = False
    
    # OAK-D detection model (YOLOv8n by default, MobileNet-SSD fallback)
    oak_detection: OakDetectionConfig = OakDetectionConfig()

    # Property map overlay
    property_map: PropertyMapConfig = PropertyMapConfig()

    # Daly BMS Bluetooth communication
    bms: BmsConfig = BmsConfig()

    # Debug settings
    log_imu_data: bool = False

    # Disarmed-state telemetry logging cadence. The main loop writes the full
    # ~10Hz structured JSON telemetry line only while armed. While disarmed
    # (robot idle, which is most of its uptime) it drops to a periodic
    # heartbeat write every log_disarmed_heartbeat_s seconds, cutting
    # idle-time disk I/O by ~99%. Events (ARM/DISARM/EMERGENCY/etc.),
    # charger_inhibit flips, active emergencies, and mode transitions still
    # log immediately regardless of this interval.
    log_disarmed_heartbeat_s: float = 5.0


# Global configuration instance
config = Config()
