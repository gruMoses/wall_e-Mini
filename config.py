"""
Configuration file for WALL-E Mini robot control system.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GpsHeadingAlignConfig:
    """Configuration for GPS-derived heading alignment of a gyro-only IMU.

    Runs every controller tick regardless of mode, so any consumer of
    IMU heading can ask for a true-north-referenced value.
    """
    enabled: bool = True
    # Calibrated for slower real-world drive patterns so lock/refinement can
    # occur during normal operation (not only fast straight runs).
    min_distance_m: float = 0.8       # displacement required in history window
    min_speed_mps: float = 0.12       # trust COG once robot is clearly moving
    min_fix_quality: int = 4          # RTK fixed only
    alpha: float = 0.1                # EMA factor for ongoing drift correction
    history_seconds: float = 8.0      # longer window stabilizes low-speed COG


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
    invert_output: bool = True   # Invert sign so positive heading error drives corrective turn direction
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
    # OAK-D IMU ingestion controls (IMU-3):
    # - "latest": consume newest packet only (lowest CPU, current default)
    # - "bounded": consume up to oak_imu_max_packets_per_poll packets each poll
    oak_imu_packet_mode: str = "latest"
    oak_imu_max_packets_per_poll: int = 4
    # Optional dedicated OAK IMU polling cadence (separate from depth poll loop).
    oak_imu_poll_hz: float = 60.0
    # Optional OAK yaw drift mitigations (IMU-5). NMNI enabled by default after validation.
    oak_nmni_enabled: bool = True
    oak_nmni_threshold_dps: float = 0.3
    oak_bias_adapt_enabled: bool = False
    oak_bias_adapt_alpha: float = 0.001
    # OAK IMU yaw-rate source:
    # - "auto": lock onto dominant gyro axis while turning (robust to mounting orientation)
    # - "gyro_x" / "gyro_y" / "gyro_z": force specific axis for diagnostics
    # - "gravity_projected": project gyro onto gravity vector
    # Known-good field setup (Apr 2026): source="auto", scale=0.46, gravity_projected=False.
    oak_yaw_rate_source: str = "auto"
    # Field-calibrated yaw-rate scale for OAK IMU heading integration.
    # Derived from in-place turn tests so displayed heading change matches reality.
    oak_yaw_rate_scale: float = 0.46
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
    max_erpm: int = 15000

    # CAN IDs for each motor
    left_can_id: int = 2
    right_can_id: int = 1

    # Low-voltage shutdown: trigger graceful OS shutdown when pack voltage stays
    # below threshold for voltage_shutdown_delay_s consecutive seconds.
    # 39.0 V = 13S Li-ion at 3.0 V/cell (safe cutoff; BMS hardware also cuts at ~2.9 V/cell).
    # BMS reports 13 cells — confirmed 13S (stale 14S comment corrected 2026-04-01).
    voltage_shutdown_threshold_v: float = 39.0
    voltage_shutdown_delay_s: float = 10.0

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
    camera_hfov_deg: float = 81.0
    min_depth_mm: int = 350            # OAK-D Lite extended disparity minimum (~0.35m)
    min_valid_pct: float = 8.0         # ignore corridor if fewer than this % of pixels are valid
    update_rate_hz: float = 15.0
    stale_timeout_s: float = 0.5
    stale_policy: str = "stop"   # fail-safe: stop when depth data is stale
    manual_stale_throttle_scale: float = 0.10  # MANUAL mode: slow (not stop) when depth is stale
    safety_stop_radius_m: float = 0.8  # YOLO "stop" tier detections within this radius force min_distance->0 (applied in oak_depth depth poll)


@dataclass(frozen=True)
class FollowMeConfig:
    """Configuration for autonomous person-following mode."""
    enabled: bool = True
    follow_distance_m: float = 1.5        # desired following distance in metres
    min_distance_m: float = 0.5
    max_distance_m: float = 6.0
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

    # ── Layer 2: Target tracker ───────────────────────────────────────────────
    target_ema_alpha: float = 0.5        # EMA smoothing on normalized horizontal offset (0=heavy, 1=none)
    target_persistence_s: float = 2.0   # hold last known position this long before declaring target lost

    # ── Host-side tracklet layer (IoU + constant-velocity Kalman) ─────────────
    # Assigns stable track_ids to person detections on parse paths that the OAK
    # device does not already track (host-side YOLOv8 NeuralNetwork parse and the
    # raw SpatialDetectionNetwork path). Lets follow_me lock onto a specific person
    # across frames / brief occlusions instead of just "closest by depth".
    tracklet_iou_threshold: float = 0.3  # min IoU (predicted bbox vs detection) to match a tracklet
    tracklet_min_hits: int = 3           # consecutive-ish hits before a tracklet is confirmed (reports its id)
    tracklet_max_age: int = 15           # frames a tracklet survives unmatched before deletion (~1 s @ 15 fps)

    # ── Layer 3: Lateral PID steering ────────────────────────────────────────
    # Error = normalized horizontal offset (-1.0 to +1.0); output scales to ±max_steer_offset_byte.
    pid_lateral_kp: float = 0.4
    pid_lateral_ki: float = 0.0
    pid_lateral_kd: float = 0.2
    pid_lateral_integral_limit: float = 0.5  # anti-windup clamp (normalised units)

    # ── Depth EMA filter (stabilises stereo depth at long range) ────────────────
    depth_ema_alpha: float = 0.35          # EMA smoothing on raw depth (0=heavy, 1=none)
    depth_max_velocity_mps: float = 5.0    # reject readings implying > this speed (m/s)

    # ── Layer 4: Speed (depth-based, closed-loop when VESC telemetry available) ─
    speed_dead_zone_m: float = 0.2       # ±dead_zone around follow_distance_m → speed = 0 (no oscillation)

    # Velocity PID — closes the speed loop using measured wheel RPM.
    # Error = target_speed_mps (derived from depth) − actual_speed_mps.
    # Output (m/s) is converted to a byte correction via trail_speed_scale_mps_per_byte.
    # Disabled automatically when VESC telemetry is unavailable (open-loop fallback).
    #
    # DISABLED 2026-06-11 (gains zeroed): the wheel-RPM telemetry feeding this loop
    # is dead/unreliable, so the PID integrates garbage and drives the lunge/stall
    # cycle (target_speed_mps is computed against a bogus actual_speed_mps). Zeroing
    # all three gains makes PIDController.compute() return 0.0 (verified: p+i+d all
    # collapse to 0, no divide-by-gain), so SpeedLayer.compute() falls back to the
    # pure open-loop throttle-vs-distance mapping (error * gain). Restore the 0.8/0.2/
    # 0.05 values once VESC eRPM telemetry is trustworthy again.
    speed_kp: float = 0.0
    speed_ki: float = 0.0
    speed_kd: float = 0.0
    speed_integral_limit: float = 50.0   # anti-windup clamp (m/s accumulated)

    # ── Slip detection & compensation ─────────────────────────────────────────
    # When |left_rpm − right_rpm| exceeds threshold while commanded straight,
    # throttle is reduced and a small steer feed-forward is injected.
    slip_threshold_rpm: float = 200.0       # eRPM differential to declare slip
    slip_throttle_reduction: float = 0.15   # throttle scale-back fraction (0–1)
    # DISABLED 2026-06-13: zeroed after a Follow-Me steer runaway. VESC RPM
    # telemetry went live this day (was always 0), activating slip comp for the
    # first time. The steer feed-forward is positive feedback on a skid-steer
    # robot — any turn creates an RPM differential, which injects more steer,
    # which increases the differential — so it pinned steer to the ±max and lost
    # the target. Throttle-reduction half left active. Re-enable only with a
    # commanded-vs-actual slip detector (not raw rpm_diff). See fm_trials
    # 1781382327 and the steer=25-on-direct-ticks signature.
    slip_feedforward_gain: float = 0.0      # steer correction per eRPM differential (bytes/RPM)

    # ── Motor output rate ─────────────────────────────────────────────────────
    follow_output_rate_hz: float = 15.0  # motor commands at this Hz, decoupled from 30 fps vision
    # Allow continued blind trail pursuit longer than short target-drop timeout.
    # This is the key behavior needed to keep moving around corners after LOS loss.
    lost_target_trail_pursuit_max_s: float = 3.0
    lost_target_search_steer_pct: float = 0.40  # fraction of max_follow_speed_byte for search turn
    max_steer_delta_per_s: float = 35.0          # steering differential slew limit (bytes/s)
    max_steer_offset_byte: float = 25.0          # increased from 15 — lets robot turn harder to keep person in FOV

    # Trail-following Pure Pursuit (breadcrumb path instead of direct pursuit)
    trail_follow_enabled: bool = True
    trail_speed_scale_mps_per_byte: float = 0.0075  # calibrated on gravel via RTK GPS: 0.0075 at offsets 67–123 (2026-03-28)
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

    # ── SafetyLayer acceleration cap ─────────────────────────────────────────
    max_speed_accel_byte_per_s: float = 150.0  # max speed ramp-up rate (bytes/s) inside Follow Me SafetyLayer

    # ── Direct pursuit steering cap ──────────────────────────────────────────
    direct_mode_max_steer_byte: float = 18.0   # max steer in direct pursuit (lower than trail to limit close-range overshoot)

    # ── Steer deadband and slew limiter ──────────────────────────────────────
    steer_deadband_norm: float = 0.04   # |x_err| below this → treat error as 0 (suppresses gait-wobble chasing; ~2-3% frame width)
    steer_slew_per_tick: float = 0.1    # max steer change per 15 Hz output tick, as fraction of max_steer_offset_byte

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
    update_rate_hz: float = 1.0
    min_quality: int = 1         # 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    stale_timeout_s: float = 3.0


@dataclass(frozen=True)
class WaypointNavConfig:
    """Configuration for autonomous GPS waypoint navigation."""
    enabled: bool = True
    waypoint_file: str = "waypoints.json"
    arrival_radius_m: float = 1.0
    cruise_speed_byte: int = 40
    approach_speed_byte: int = 20
    slow_radius_m: float = 2.0
    min_rtk_quality: int = 4     # require RTK fixed for autonomous nav
    stale_timeout_s: float = 3.0
    # State-machine thresholds (see WaypointNavController)
    align_threshold_deg: float = 12.0      # |heading_err| below this -> ALIGN->DRIVE
    recovery_threshold_deg: float = 25.0   # |heading_err| above this -> DRIVE->ALIGN
    pivot_yaw_cmd: float = 0.5             # normalized yaw command during ALIGN pivot
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
    # Network-level confidence threshold. Lower than Follow Me threshold so safety tiers
    # see all detections; Follow Me applies its own post-filter (FollowMeConfig.detection_confidence).
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

    # GPS-derived heading alignment (runs every tick regardless of mode)
    gps_heading_align: GpsHeadingAlignConfig = GpsHeadingAlignConfig()

    # Follow Me person-tracking mode
    follow_me: FollowMeConfig = FollowMeConfig()

    # Hand-gesture Follow Me activation
    gesture: GestureConfig = GestureConfig()

    # Final motor-output slew limiter
    slew_limiter: SlewLimiterConfig = SlewLimiterConfig()

    # OAK-D recording (video + MCAP)
    oak_recording: OakRecordingConfig = OakRecordingConfig()

    # Live web viewer (MJPEG stream + recordings browser)
    oak_web_viewer: OakWebViewerConfig = OakWebViewerConfig()
    
    # IMU source: "auto" (try external I2C first, fall back to OAK-D),
    # "external" (I2C breakout only), "oak_d" (OAK-D onboard BMI270 only), "none"
    imu_source: str = "auto"

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


# Global configuration instance
config = Config()
