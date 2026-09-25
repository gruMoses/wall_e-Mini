"""
Configuration file for WALL-E Mini robot control system.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GpsHeadingAlignConfig:
    """One-shot GPS alignment for the gyro-only IMU.

    Lock history is accepted only from forward, straight manual-RC motion.
    Once locked, the offset is frozen until the armed session ends.
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

    # Low-voltage watchdog — EARLY WARNING + MOTOR CUTOFF only (no OS shutdown).
    # When pack voltage stays in the band [floor, threshold) for
    # voltage_shutdown_delay_s consecutive seconds we cut motors and raise a
    # recoverable pack-low latch (surfaced on telemetry). We do NOT halt the Pi:
    # the Pi runs on its own UPS, so over-discharge protection is delegated to
    # the pack BMS hard-cut (~37.7 V) -> UPS input-loss -> graceful-shutdown
    # chain, which auto-recovers when pack power returns.
    # 39.0 V = 13S Li-ion at 3.0 V/cell (safe cutoff; BMS hardware also cuts at ~2.9 V/cell).
    # BMS reports 13 cells — confirmed 13S (stale 14S comment corrected 2026-04-01).
    voltage_shutdown_threshold_v: float = 39.0
    # Plausibility floor: a reading BELOW this means the pack is disconnected /
    # switched off / sensor garbage. A 13S pack's BMS hard-cuts ~37.7 V, so it
    # can never genuinely sit this low — a reading below the floor is a normal
    # bench event (main battery switched off while the Pi runs on its UPS), NOT
    # a dying pack. Below the floor we stop motors and log once, but never latch
    # and never treat it as a low-voltage event (2026-07-10 false-trigger fix).
    voltage_shutdown_floor_v: float = 30.0
    # 30s sustain: a genuinely dying pack sags slowly, so a full 30s continuous
    # in-band reading is required before cutting motors (rejects transients).
    # (Was 10.0; contract changed 2026-07-10.)
    voltage_shutdown_delay_s: float = 30.0

    # Wheel geometry + drivetrain for eRPM -> wheel speed (m/s) conversion.
    # wheel_radius_m: centre of wheel axle to contact patch (14.5in wheel dia).
    # motor_poles: total magnetic poles; pole_pairs = motor_poles // 2.
    # drive_gear_ratio: motor revs per wheel rev.
    #   6:1 gearbox * (20/14) chain stage * (40/10) chain stage = 34.2857:1
    wheel_radius_m: float = 0.18415
    motor_poles: int = 14
    drive_gear_ratio: float = 34.2857142857

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
    # This value is now only a FALLBACK: oak_depth._depth_intrinsics_for() reads
    # the per-unit calibration off the device and uses that. It matters only if
    # the EEPROM read fails.
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
    # MANUAL mode: the corridor stop is a floor, not a wall. The RC/phone
    # operator can always creep forward at this fraction of the stick (0.15
    # = ~0.24 m/s at full stick), so a phantom obstacle cannot strand the
    # robot (2026-09-19 19:53: could not drive into the garage, had to back
    # in). The YOLO person/animal stop tier (distance forced to 0.0) stays
    # absolute. 0.0 restores the hard stop.
    manual_obstacle_min_scale: float = 0.15
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


@dataclass(frozen=True)
class FollowMeConfig:
    """Configuration for autonomous person-following mode."""
    enabled: bool = True
    follow_distance_m: float = 1.5        # desired following distance in metres
    min_distance_m: float = 0.5
    max_distance_m: float = 6.0
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
    max_speed_error_m: float = 1.5   # distance error at which max speed is reached — tighter = more aggressive closing
    max_follow_speed_byte: int = 110
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
    # for; 0.05 keeps a person to ~6 m (max_distance_m).
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
    #   2. speed_pid_max_correction_mps clamps the loop's authority to ±0.20 m/s
    #      (≈ ±21 bytes), so even a wrong error can only nudge, never lunge.
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
    speed_integral_limit: float = 1.0    # anti-windup clamp (m/s·s); was 50 — absurd with a 0.2 m/s clamp
    speed_pid_max_correction_mps: float = 0.20   # |closed-loop correction| ceiling (≈ 21 bytes)
    speed_loop_mps_per_byte: float = 0.012555    # kinematic WHEEL m/s per byte at max_erpm 20000 (1.607 m/s / 128) — see SCALE above

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
    # ±max_steer_offset_byte (25) and at/under the direct cap (18). The TOTAL
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
    # that had no backing field and EXCEEDED both max_steer_offset_byte (25) and
    # direct_mode_max_steer_byte (18). 20 keeps search comfortably under the
    # global cap (25) while still turning harder than the tighter direct-pursuit
    # cap (18), matching the pre-existing effective ceiling search operated
    # under once the (now also slew-capped) emission gate clamps everything.
    search_steer_cap_byte: float = 20.0
    # Gain on the trail-tangent/last-known-position bias blended into blind
    # trail-pursuit steer as elapsed lost-time grows (anticipates the turn the
    # person was making). Was previously read via a phantom getattr fallback
    # with no backing field; 3.0 matches that fallback exactly, so behavior is
    # unchanged — it's now a real, discoverable config knob.
    lost_steer_bias_gain: float = 3.0

    # Trail-following Pure Pursuit (breadcrumb path instead of direct pursuit)
    trail_follow_enabled: bool = True
    trail_speed_scale_mps_per_byte: float = 0.0100  # 0.0075 calibrated on gravel via RTK GPS at max_erpm 15000 (2026-03-28), scaled 4/3 for 20000 on 2026-09-19 — re-calibrate
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
    # The mixer clips at 245. True yaw authority is 0.65-0.71 deg/s per L-R
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
class PumpGestureConfig:
    """Log-only low two-hand "pump" detector.

    ``enabled`` turns the log on. This step has no motion path: the
    controller records the detector output and does not read it when it
    chooses motor bytes, mode, or follow-me behaviour.

    Thresholds are the first values from the 2026-09-24 box-width trial
    (docs/backup_pump_design.md). Every comparison the detector makes
    reads one of these fields.
    """
    enabled: bool = True
    min_range_m: float = 1.0
    max_range_m: float = 5.0
    # Normalised. A box with xmin <= margin or xmax >= 1 - margin touches
    # the side of the NN frame.
    edge_margin: float = 0.01
    min_body_w_m: float = 0.3
    max_body_w_m: float = 2.2
    rest_window_s: float = 3.0
    rest_min_samples: int = 8
    rest_min_span_s: float = 1.5
    freeze_ratio: float = 1.4
    out_ratio: float = 1.5
    edge_out_frac: float = 0.25
    max_centre_shift_m: float = 0.15
    min_out_frames: int = 3
    # An armed run may hold through the 1.4x-1.5x band on its way down,
    # but only this long after its last out frame.
    max_hold_s: float = 0.4
    peak_end_ratio: float = 1.4
    start_peaks: int = 2
    start_window_s: float = 3.0
    continue_window_s: float = 0.7
    no_detection_stop_s: float = 0.3
    max_active_s: float = 8.0
    cooldown_s: float = 2.0
    rearm_rest_ratio: float = 1.3
    rearm_rest_s: float = 1.0
    log_nearest_outside_follow_me: bool = True


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
    # Today's NeuralNetwork.setNumInferenceThreads value. Reserved for the next A/B.
    nn_inference_threads: int = 2
    # Today's NeuralNetwork.setNumShavesPerInferenceThread value.
    nn_shaves_per_thread: int = 4


@dataclass(frozen=True)
class Config:
    """Main configuration class."""
    
    # IMU steering configuration
    imu_steering: ImuSteeringConfig = ImuSteeringConfig()
    # RC mapping config
    rc_map: RcMapConfig = RcMapConfig()
    # VESC config
    vesc: VescConfig = VescConfig()

    # OAK-D Lite obstacle avoidance
    obstacle_avoidance: ObstacleAvoidanceConfig = ObstacleAvoidanceConfig()

    # RTK GPS
    gps: GpsConfig = GpsConfig()

    # Waypoint navigation
    waypoint_nav: WaypointNavConfig = WaypointNavConfig()

    # GPS-derived heading alignment (manual forward lock, then frozen)
    gps_heading_align: GpsHeadingAlignConfig = GpsHeadingAlignConfig()

    # Follow Me person-tracking mode
    follow_me: FollowMeConfig = FollowMeConfig()

    # Hand-gesture Follow Me activation
    gesture: GestureConfig = GestureConfig()

    # Both-wrists-up body-pose detector (bench twitch; real reverse-follow later)
    arms_up: ArmsUpConfig = ArmsUpConfig()

    # Log-only low two-hand pump detector. No motor path in this step.
    pump: PumpGestureConfig = PumpGestureConfig()

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
