"""
Autonomous person-following controller.

Architecture — five clean layers, each with a single responsibility:

  1. DetectionFilter  — raw PersonDetection list  → validated, normalised candidates
  2. TargetTracker    — candidates → single smoothed target (EMA, persistence, track-ID sticky)
  3. SteeringLayer    — normalized x-offset → lateral PID → steer_byte differential
  4. SpeedLayer       — depth error → proportional speed with dead zone
  5. SafetyLayer      — smooth acceleration, speed cap

  _mix_commands()     — (speed_offset, steer_offset) → (left_byte, right_byte)

  FollowMeController  — orchestrates all layers; also integrates the optional
                         trail-following Pure-Pursuit subsystem (unchanged logic,
                         plugs into the steering layer as an override).

Motor output is rate-limited to ~15 Hz (configurable), decoupled from the
30 fps vision pipeline.  Between output ticks the last command is held.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT
from pi_app.control.gps_odometry import GpsOdometry, GpsOdometryConfig
from pi_app.control.odometry import DeadReckonOdometry
from pi_app.control.pure_pursuit import PurePursuitController, PursuitConfig
from pi_app.control.trail import TrailConfig, TrailManager

NEUTRAL = CENTER_OUTPUT_VALUE


# ─────────────────────────────────────────────────────────────────────────────
# Public dataclass — API contract with oak_depth.py; do not rename fields.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class PersonDetection:
    """Single person detection from the OAK-D Lite spatial network."""
    x_m: float       # lateral offset from camera centre (+ = right)
    z_m: float       # forward distance in metres
    confidence: float
    bbox: tuple[float, float, float, float]  # xmin, ymin, xmax, ymax (normalised 0–1)
    track_id: int | None = None


# ─────────────────────────────────────────────────────────────────────────────
# Layer 1: Detection Filter
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class _FilteredDetection:
    """Validated, normalised detection ready for the tracker."""
    normalized_x: float    # horizontal offset: -1.0 (far left) … +1.0 (far right)
    x_m: float             # lateral offset in metres (for trail odometry)
    depth_m: float
    confidence: float
    bbox: tuple[float, float, float, float]
    track_id: int | None = None


class DetectionFilter:
    """Layer 1 — validates raw YOLO PersonDetections and normalises the output.

    Rejects detections that are:
    * below the confidence threshold
    * outside the valid depth range [min_depth_m, max_depth_m]
    * too small (bounding-box area below min_bbox_area as a frame fraction)
    """

    def __init__(
        self,
        conf_threshold: float,
        min_depth_m: float,
        max_depth_m: float,
        min_bbox_area: float,
    ) -> None:
        self._conf = conf_threshold
        self._min_depth = min_depth_m
        self._max_depth = max_depth_m
        self._min_area = min_bbox_area

    def process(self, raw: list[PersonDetection]) -> list[_FilteredDetection]:
        """Return validated and normalised detections."""
        out: list[_FilteredDetection] = []
        for det in raw:
            if det.confidence < self._conf:
                continue
            if not (self._min_depth <= det.z_m <= self._max_depth):
                continue
            area = (det.bbox[2] - det.bbox[0]) * (det.bbox[3] - det.bbox[1])
            if area < self._min_area:
                continue
            cx = (det.bbox[0] + det.bbox[2]) * 0.5
            normalized_x = (cx - 0.5) * 2.0  # map [0, 1] → [-1, +1]
            out.append(_FilteredDetection(
                normalized_x=normalized_x,
                x_m=det.x_m,
                depth_m=det.z_m,
                confidence=det.confidence,
                bbox=det.bbox,
                track_id=det.track_id,
            ))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# Layer 2: Target Tracker
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class _TargetState:
    """Current tracked target after EMA smoothing."""
    normalized_x: float    # EMA-smoothed horizontal offset
    x_m: float             # latest raw lateral offset in metres (for trail bias)
    depth_m: float         # latest raw depth
    confidence: float
    track_id: int | None
    last_seen_time: float  # time.monotonic() of most recent fresh detection


class TargetTracker:
    """Layer 2 — selects and smooths the follow target across frames.

    Selection strategy:
      * If the previously tracked ID is still visible, keep following it
        (track-ID sticky continuity).
      * Otherwise select the closest candidate by depth_m.

    EMA smoothing (alpha ~0.35) damps frame-to-frame jitter on the
    horizontal offset before it reaches the PID.

    Persistence: after the camera stops seeing the target, the last known
    state is held for up to ``persistence_s`` seconds before returning None.
    """

    def __init__(self, ema_alpha: float, persistence_s: float) -> None:
        self._alpha = ema_alpha
        self._persistence_s = persistence_s
        self._state: _TargetState | None = None

    def update(
        self,
        candidates: list[_FilteredDetection],
        now: float,
    ) -> _TargetState | None:
        """Update tracker; return current state or None when target is lost."""
        if candidates:
            best = self._select(candidates)
            self._apply_ema(best, now)
        else:
            # No fresh detection — check persistence window
            if self._state is None:
                return None
            if (now - self._state.last_seen_time) > self._persistence_s:
                return None
            # Return stale state; caller sees it and can handle gracefully
        return self._state

    def _select(self, candidates: list[_FilteredDetection]) -> _FilteredDetection:
        """Pick target: prefer last known track_id, else take closest by depth."""
        if self._state is not None and self._state.track_id is not None:
            for c in candidates:
                if c.track_id == self._state.track_id:
                    return c
        return min(candidates, key=lambda d: d.depth_m)

    def _apply_ema(self, det: _FilteredDetection, now: float) -> None:
        if self._state is None:
            smoothed_x = det.normalized_x
        else:
            smoothed_x = (
                self._alpha * det.normalized_x
                + (1.0 - self._alpha) * self._state.normalized_x
            )
        self._state = _TargetState(
            normalized_x=smoothed_x,
            x_m=det.x_m,
            depth_m=det.depth_m,
            confidence=det.confidence,
            track_id=det.track_id,
            last_seen_time=now,
        )

    def reset(self) -> None:
        self._state = None

    @property
    def last_seen_time(self) -> float:
        return self._state.last_seen_time if self._state is not None else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Reusable PID controller
# ─────────────────────────────────────────────────────────────────────────────

class PIDController:
    """Generic PID with anti-windup integral clamping.

    All terms work in the caller's units.  ``dt`` is capped at 100 ms
    internally to prevent integrator blowup after stale / first-tick calls.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = float("inf"),
        output_limit: float = float("inf"),
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral_limit = integral_limit
        self._output_limit = output_limit
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_tick: bool = True

    def compute(self, error: float, dt: float) -> float:
        """Return PID output for the given error and elapsed time."""
        dt = max(1e-6, min(dt, 0.1))  # cap to avoid first-call blowup

        p = self.kp * error

        self._integral += error * dt
        self._integral = max(
            -self._integral_limit, min(self._integral_limit, self._integral)
        )
        i = self.ki * self._integral

        d = 0.0
        if not self._first_tick:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        self._first_tick = False

        output = p + i + d
        return max(-self._output_limit, min(self._output_limit, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_tick = True


# ─────────────────────────────────────────────────────────────────────────────
# Layer 3: Steering (PID lateral control)
# ─────────────────────────────────────────────────────────────────────────────

class SteeringLayer:
    """Layer 3 — lateral PID control.

    Input:  normalized horizontal offset of target (-1.0 = full left,
                                                    +1.0 = full right)
    Output: steering differential in motor-byte units (positive = turn right;
            left motor gets +steer, right motor gets -steer).

    The PID error IS the normalised offset: zero when the target is centred.
    The raw PID output (normalised) is then scaled to ±max_steer_byte.
    """

    def __init__(self, pid: PIDController, max_steer_byte: float) -> None:
        self._pid = pid
        self._max = max_steer_byte

    def compute(self, normalized_x: float, dt: float) -> float:
        """Compute steer offset in motor bytes from lateral error."""
        raw = self._pid.compute(normalized_x, dt)
        return max(-self._max, min(self._max, raw * self._max))

    def reset(self) -> None:
        self._pid.reset()


# ─────────────────────────────────────────────────────────────────────────────
# Layer 4: Speed (depth-based)
# ─────────────────────────────────────────────────────────────────────────────

class SpeedLayer:
    """Layer 4 — depth-based speed control, optionally closed-loop via velocity PID.

    When the robot is within ±dead_zone_m of the desired following distance,
    output is zero so the robot holds position without oscillating.
    Outside the dead zone, open-loop speed is proportional to the distance error.

    If a velocity_pid and speed_scale_mps_per_byte are supplied, and
    actual_speed_mps is passed to compute(), the layer closes the speed loop:
      velocity_error = target_speed_mps − actual_speed_mps
      correction_byte = PID(velocity_error) / speed_scale_mps_per_byte
    Falls back gracefully to open-loop when actual_speed_mps is None.
    """

    def __init__(
        self,
        target_dist_m: float,
        dead_zone_m: float,
        speed_gain: float,          # bytes per metre of distance error
        min_dist_m: float,
        max_speed_byte: float,
        velocity_pid: PIDController | None = None,
        speed_scale_mps_per_byte: float = 0.0075,
    ) -> None:
        self._target = target_dist_m
        self._dead_zone = dead_zone_m
        self._gain = speed_gain
        self._min_dist = min_dist_m
        self._max_speed = max_speed_byte
        self._velocity_pid = velocity_pid
        self._speed_scale = max(speed_scale_mps_per_byte, 1e-9)

    def compute(
        self,
        depth_m: float,
        actual_speed_mps: float | None = None,
        dt: float = 0.05,
    ) -> float:
        """Return forward speed offset in motor bytes (0 = hold/stop).

        When actual_speed_mps is provided and a velocity_pid is configured,
        a closed-loop correction is applied on top of the open-loop output.
        Otherwise (or when actual speed is None) behaves as pure open-loop.
        """
        if depth_m <= self._min_dist:
            if self._velocity_pid is not None:
                self._velocity_pid.reset()
            return 0.0
        error = depth_m - self._target
        if abs(error) <= self._dead_zone:
            if self._velocity_pid is not None:
                self._velocity_pid.reset()
            return 0.0
        if error <= 0.0:
            return 0.0  # too close — stop (backing up not implemented here)
        open_loop = min(self._max_speed, error * self._gain)

        # Closed-loop velocity correction when telemetry is available
        if self._velocity_pid is not None and actual_speed_mps is not None:
            target_speed_mps = open_loop * self._speed_scale
            velocity_error = target_speed_mps - actual_speed_mps
            correction_mps = self._velocity_pid.compute(velocity_error, dt)
            correction_byte = correction_mps / self._speed_scale
            return max(0.0, min(self._max_speed, open_loop + correction_byte))
        return open_loop


# ─────────────────────────────────────────────────────────────────────────────
# Layer 5: Safety
# ─────────────────────────────────────────────────────────────────────────────

class SafetyLayer:
    """Layer 5 — safety constraints on the combined motor command.

    Enforces:
    * Maximum speed cap
    * Smooth acceleration (slew-rate limiting on speed)

    The final slew limiter in controller.py provides an additional
    motor-level ramp; this layer guards within follow_me itself.
    """

    def __init__(self, max_speed_byte: float, max_accel_bps: float) -> None:
        self._max_speed = max_speed_byte
        self._max_accel = max_accel_bps
        self._prev_speed: float = 0.0
        self._prev_time: float = 0.0

    def apply(self, speed: float, steer: float, now: float) -> tuple[float, float]:
        """Return (safe_speed, safe_steer) with accel limiting and speed cap."""
        speed = min(speed, self._max_speed)
        if self._prev_time > 0.0:
            dt = now - self._prev_time
            if dt > 0.0:
                max_increase = self._max_accel * dt
                speed = min(speed, self._prev_speed + max_increase)
        speed = max(0.0, speed)
        self._prev_speed = speed
        self._prev_time = now
        return speed, steer

    def reset(self) -> None:
        self._prev_speed = 0.0
        self._prev_time = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Motor mixer
# ─────────────────────────────────────────────────────────────────────────────

def _mix_commands(speed_offset: float, steer_offset: float) -> tuple[int, int]:
    """Combine speed and steering differentials into left/right motor bytes."""
    left = int(round(NEUTRAL + speed_offset + steer_offset))
    right = int(round(NEUTRAL + speed_offset - steer_offset))
    return (
        max(MIN_OUTPUT, min(MAX_OUTPUT, left)),
        max(MIN_OUTPUT, min(MAX_OUTPUT, right)),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main controller — orchestrates all layers
# ─────────────────────────────────────────────────────────────────────────────

class FollowMeController:
    """Orchestrates the five follow-me layers.

    Public API (unchanged from prior version so controller.py needs no edits):
      update_pose(heading_deg, motor_l, motor_r, timestamp)
      update_gps(lat, lon, fix_quality, timestamp)
      compute(detections) -> (left_byte, right_byte)
      get_status()        -> dict
    """

    _MIN_LOST_TARGET_SPEED = 5  # minimum forward bytes during trail / search recovery

    def __init__(self, config: FollowMeConfig) -> None:
        self._cfg = config

        # ── Layer 1: Detection filter ────────────────────────────────────────
        self._filter = DetectionFilter(
            conf_threshold=config.detection_confidence,
            min_depth_m=config.min_distance_m,
            max_depth_m=config.max_distance_m,
            min_bbox_area=getattr(config, "min_bbox_area", 0.0015),
        )

        # ── Layer 2: Target tracker ──────────────────────────────────────────
        self._tracker = TargetTracker(
            ema_alpha=getattr(config, "target_ema_alpha", 0.35),
            persistence_s=getattr(config, "target_persistence_s", 1.0),
        )

        # ── Layer 3: Steering (PID) ──────────────────────────────────────────
        max_steer = float(getattr(config, "max_steer_offset_byte", 25.0))
        self._steering = SteeringLayer(
            pid=PIDController(
                kp=getattr(config, "pid_lateral_kp", 1.8),
                ki=getattr(config, "pid_lateral_ki", 0.2),
                kd=getattr(config, "pid_lateral_kd", 0.7),
                integral_limit=getattr(config, "pid_lateral_integral_limit", 0.5),
                output_limit=1.0,  # normalised; SteeringLayer scales to bytes
            ),
            max_steer_byte=max_steer,
        )

        # ── Layer 4: Speed (depth-based, closed-loop when telemetry available) ──
        max_speed = float(config.max_follow_speed_byte)
        max_speed_err = float(getattr(config, "max_speed_error_m", 2.5))
        _speed_scale = float(getattr(config, "trail_speed_scale_mps_per_byte", 0.0075))
        _velocity_pid = PIDController(
            kp=float(getattr(config, "speed_kp", 0.8)),
            ki=float(getattr(config, "speed_ki", 0.2)),
            kd=float(getattr(config, "speed_kd", 0.05)),
            integral_limit=float(getattr(config, "speed_integral_limit", 50.0)),
        )
        self._speed = SpeedLayer(
            target_dist_m=config.follow_distance_m,
            dead_zone_m=float(getattr(config, "speed_dead_zone_m", 0.2)),
            speed_gain=max_speed / max(max_speed_err, 0.1),
            min_dist_m=config.min_distance_m,
            max_speed_byte=max_speed,
            velocity_pid=_velocity_pid,
            speed_scale_mps_per_byte=_speed_scale,
        )

        # ── Layer 5: Safety ──────────────────────────────────────────────────
        self._safety = SafetyLayer(
            max_speed_byte=max_speed,
            max_accel_bps=float(getattr(config, "max_speed_accel_byte_per_s", 150.0)),
        )

        # ── Motor output rate limiting ───────────────────────────────────────
        # Computation (PID, trail, steering) always runs at full call rate.
        # Only the emitted motor bytes are held at follow_output_rate_hz.
        output_hz = float(getattr(config, "follow_output_rate_hz", 15.0))
        self._output_interval_s = 1.0 / max(1.0, output_hz)
        self._last_output_time: float = 0.0
        self._cached_left: int = NEUTRAL
        self._cached_right: int = NEUTRAL
        self._prev_target_present: bool = False  # for transition detection
        self._prev_fresh_detection: bool = False  # True only when persons visible this frame
        self._reacq_time: float = 0.0             # monotonic() of last lost→tracking transition
        self._last_fresh_steer: float = 0.0       # steer from last frame with fresh detection
        self._last_fresh_steer_time: float = 0.0  # monotonic() of that frame

        # ── Telemetry state ──────────────────────────────────────────────────
        self._tracking: bool = False
        self._last_target_z: float | None = None
        self._last_target_x: float | None = None      # x in metres (for trail bias)
        self._last_distance_error: float | None = None
        self._last_speed_offset: float = 0.0
        self._last_steer_offset: float = 0.0
        self._last_num_detections: int = 0
        self._last_target_confidence: float = 0.0
        self._last_target_track_id: int | None = None
        self._last_valid_time: float = 0.0
        self._pursuit_mode: str = "direct"

        # ── VESC telemetry (for closed-loop speed and slip detection) ────────
        self._actual_left_rpm: int | None = None
        self._actual_right_rpm: int | None = None
        self._actual_speed_mps: float | None = None
        self._last_slip_active: bool = False

        # ── Trail-following subsystems (Pure Pursuit — preserved) ────────────
        self._trail_enabled = bool(getattr(config, "trail_follow_enabled", False))
        self._last_pursuit_mode: str = "direct"
        self._odometry: DeadReckonOdometry | None = None
        self._gps_odom: GpsOdometry | None = None
        self._trail: TrailManager | None = None
        self._pursuit: PurePursuitController | None = None
        self._pursuit_lookahead_x: float = 0.0
        self._pursuit_lookahead_y: float = 0.0
        self._trail_length: int = 0
        self._trail_distance_m: float = 0.0
        self._trail_rejected_jump_count: int = 0
        self._trail_rejected_speed_count: int = 0
        self._last_target_world_x: float | None = None
        self._last_target_world_y: float | None = None
        self._curvature_at_lookahead: float = 0.0
        self._speed_limited: bool = False
        self._odom_source: str = "none"
        self._last_mode_switch_time: float = 0.0

        if self._trail_enabled:
            self._odometry = DeadReckonOdometry(
                speed_scale=getattr(config, "trail_speed_scale_mps_per_byte", 0.0016)
            )
            self._gps_odom = GpsOdometry(GpsOdometryConfig(
                cog_min_speed_mps=getattr(config, "gps_cog_min_speed_mps", 0.5),
                heading_alpha=getattr(config, "gps_heading_alpha", 0.85),
                cog_min_delta_m=getattr(config, "gps_cog_min_delta_m", 0.05),
            ))
            self._trail = TrailManager(TrailConfig(
                max_trail_points=getattr(config, "trail_max_points", 100),
                min_spacing_m=getattr(config, "trail_min_spacing_m", 0.3),
                max_age_s=getattr(config, "trail_max_age_s", 30.0),
                consume_radius_m=getattr(config, "trail_consume_radius_m", 0.4),
                max_step_m=getattr(config, "trail_max_step_m", 1.2),
                max_speed_mps=getattr(config, "trail_max_speed_mps", 2.5),
                smoothing_enabled=getattr(config, "trail_smoothing_enabled", True),
                smoothing_window=getattr(config, "trail_smoothing_window", 5),
                smoothing_poly_order=getattr(config, "trail_smoothing_poly_order", 2),
            ))
            self._pursuit = PurePursuitController(PursuitConfig(
                lookahead_time_s=getattr(config, "pursuit_lookahead_time_s", 0.8),
                lookahead_min_m=getattr(config, "pursuit_lookahead_min_m", 0.5),
                lookahead_max_m=getattr(config, "pursuit_lookahead_max_m", 2.5),
                speed_scale_mps_per_byte=getattr(config, "trail_speed_scale_mps_per_byte", 0.0016),
                wheelbase_m=getattr(config, "pursuit_wheelbase_m", 0.28),
                max_steer_byte=float(getattr(config, "max_steer_offset_byte", 15.0)),
                max_speed_byte=max_speed,
                curvature_scaling_enabled=getattr(config, "pursuit_curvature_scaling_enabled", True),
                curvature_alpha=getattr(config, "pursuit_curvature_alpha", 5.0),
                min_speed_byte=float(getattr(config, "pursuit_min_speed_byte", 15.0)),
                lookahead_curvature_points=getattr(config, "pursuit_lookahead_curvature_points", 5),
                max_accel_byte_per_s=float(getattr(config, "pursuit_max_accel_byte_per_s", 50.0)),
            ))

    # ── Public API: odometry feeds ───────────────────────────────────────────

    def update_pose(
        self,
        heading_deg: float,
        motor_l: int,
        motor_r: int,
        timestamp: float,
    ) -> None:
        """Feed IMU heading and actual motor commands for dead-reckoning odometry.

        Called by controller.py every cycle before compute().
        """
        if self._odometry is not None:
            self._odometry.update(heading_deg, motor_l, motor_r, timestamp)
        if self._gps_odom is not None:
            self._gps_odom.update_gyro(heading_deg, timestamp)

    def update_gps(
        self,
        lat: float,
        lon: float,
        fix_quality: int,
        timestamp: float,
    ) -> None:
        """Feed GPS position for GPS-based trail odometry.

        Called by controller.py every cycle when a GPS reading is available.
        """
        if self._gps_odom is not None:
            self._gps_odom.update_gps(lat, lon, fix_quality, timestamp)

    def update_telemetry(
        self,
        left_rpm: int | None,
        right_rpm: int | None,
        actual_speed_mps: float | None,
    ) -> None:
        """Feed VESC telemetry for closed-loop speed control and slip detection.

        Called by controller.py in FOLLOW_ME mode before each compute() call.
        Passing all-None gracefully disables closed-loop / slip features.
        """
        self._actual_left_rpm = left_rpm
        self._actual_right_rpm = right_rpm
        self._actual_speed_mps = actual_speed_mps

    # ── Public API: main compute ─────────────────────────────────────────────

    def compute(self, detections: list[PersonDetection]) -> tuple[int, int]:
        """Compute (left_byte, right_byte) to follow the best-scored person.

        Vision detections may arrive at up to 30 fps; all control computation
        (PID, trail breadcrumbs, steering mode selection) runs on every call.
        Only the emitted motor bytes are held at follow_output_rate_hz
        (default 15 Hz) so the robot doesn't jerk at the vision frame rate.
        """
        now = time.monotonic()
        self._last_num_detections = len(detections)

        # ── Layers 1 + 2: filter and track (full rate) ───────────────────────
        filtered = self._filter.process(detections)
        target = self._tracker.update(filtered, now)

        # Telemetry: always update from tracker result
        target_present = target is not None
        if target_present:
            self._last_target_z = target.depth_m
            self._last_target_x = target.x_m    # metres, used by trail bias
            self._last_target_track_id = target.track_id
            self._last_target_confidence = target.confidence
            if filtered:
                self._tracking = True
                self._last_valid_time = now
        else:
            self._tracking = False

        # ── Trail breadcrumbs (full rate — accurate path needs every point) ───
        if self._trail_enabled and self._trail is not None and filtered and target_present:
            odom = self._pick_odometry()
            if odom is not None:
                wx, wy = odom.camera_to_world(target.x_m, target.depth_m)
                self._last_target_world_x = wx
                self._last_target_world_y = wy
                self._trail.add_point(wx, wy, now, speed_hint=self._last_speed_offset)
                pose = odom.pose
                self._trail.prune(pose.x, pose.y, pose.theta, now)
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count

        # ── Full control computation (full rate) ──────────────────────────────
        # dt for PID and safety is time since last compute(), not last output.
        dt = now - self._last_output_time if self._last_output_time > 0.0 else (
            1.0 / max(1.0, float(getattr(self._cfg, "follow_output_rate_hz", 15.0)))
        )

        # True only when there is a live detection this frame (not stale persistence data).
        fresh_detection = bool(filtered)

        if not target_present:
            self._prev_fresh_detection = False
            left, right = self._handle_lost_target(now)
        else:
            # Layer 4: Speed (closed-loop when VESC telemetry is available)
            speed = self._speed.compute(
                target.depth_m,
                actual_speed_mps=self._actual_speed_mps,
                dt=dt,
            )
            self._last_distance_error = target.depth_m - self._cfg.follow_distance_m

            # Layer 3: Steering — only when there is a FRESH detection this frame.
            # When the tracker is coasting on stale data (persistence window, persons=0)
            # hold the last fresh steer and decay it linearly to 0 over steer_hold_decay_s
            # so the robot continues turning during brief occlusions instead of driving straight.
            if not fresh_detection:
                elapsed_since_fresh = now - self._last_fresh_steer_time
                hold_decay_s = float(getattr(self._cfg, "steer_hold_decay_s", 1.5))
                decay = max(0.0, 1.0 - elapsed_since_fresh / hold_decay_s) if hold_decay_s > 0.0 else 0.0
                steer = self._last_fresh_steer * decay
            else:
                if not self._prev_fresh_detection:
                    self._reacq_time = now  # mark reacquisition start
                steer = self._compute_steering(target, speed, dt, now)
                # Ramp steer 0 → full over reacq_slew_window_s after a dropout to
                # prevent the spike on the first frames after reacquisition.
                reacq_window = max(
                    float(getattr(self._cfg, "reacq_slew_window_s", 0.5)), 0.05
                )
                if self._reacq_time > 0.0:
                    reacq_elapsed = now - self._reacq_time
                    if reacq_elapsed < reacq_window:
                        steer *= reacq_elapsed / reacq_window
                self._last_fresh_steer = steer
                self._last_fresh_steer_time = now

            self._prev_fresh_detection = fresh_detection

            # Slip detection & compensation (when VESC RPM telemetry available)
            speed, steer = self._apply_slip_compensation(speed, steer)

            # Layer 5: Safety
            speed, steer = self._safety.apply(speed, steer, now)

            self._last_speed_offset = speed
            self._last_steer_offset = steer
            left, right = _mix_commands(speed, steer)

        # ── Rate-limit motor OUTPUT only ──────────────────────────────────────
        # Always emit immediately on first call or state transition (target
        # gained / lost). Otherwise hold the last command for the remainder of
        # the output interval so motors aren't updated at the full vision rate.
        dt_output = now - self._last_output_time
        state_changed = target_present != self._prev_target_present
        self._prev_target_present = target_present

        if (self._last_output_time <= 0.0
                or dt_output >= self._output_interval_s
                or state_changed):
            self._cache_output((left, right), now)

        return self._cached_left, self._cached_right

    # ── Private helpers ──────────────────────────────────────────────────────

    def _cache_output(self, cmd: tuple[int, int], now: float) -> None:
        self._cached_left, self._cached_right = cmd
        self._last_output_time = now

    def _pick_odometry(self):
        """Return the best available odometry source (GPS preferred)."""
        if self._gps_odom is not None and self._gps_odom.has_fix:
            self._odom_source = "gps"
            return self._gps_odom
        if self._odometry is not None:
            self._odom_source = "dead_reckon"
            return self._odometry
        self._odom_source = "none"
        return None

    def _compute_steering(
        self,
        target: _TargetState,
        speed_offset: float,
        dt: float,
        now: float,
    ) -> float:
        """Choose between trail Pure-Pursuit and direct PID for steering.

        Trail pursuit is used when conditions are met (trail enabled, enough
        breadcrumbs, target is far / turning).  Falls back to Layer 3 PID
        direct pursuit otherwise.
        """
        odom = self._pick_odometry()
        mode_dwell_s = float(getattr(self._cfg, "mode_switch_dwell_s", 0.5))

        if (
            self._trail_enabled
            and self._pursuit is not None
            and odom is not None
            and self._trail is not None
        ):
            trail_pts = self._trail.get_smoothed_trail()
            self._trail_length = len(trail_pts)
            min_pts = int(getattr(self._cfg, "min_trail_points_for_pursuit", 2))
            direct_dist = float(getattr(self._cfg, "direct_pursuit_distance_m", 2.0))
            direct_lat = float(getattr(self._cfg, "direct_pursuit_lateral_m", 0.3))

            time_in_mode = now - self._last_mode_switch_time
            if self._last_pursuit_mode == "trail":
                use_direct = (
                    target.depth_m < direct_dist
                    and abs(target.normalized_x) < direct_lat
                    and time_in_mode >= mode_dwell_s
                )
            else:
                use_direct = not (
                    (target.depth_m > direct_dist * 1.3
                     or abs(target.normalized_x) > direct_lat * 1.5)
                    and time_in_mode >= mode_dwell_s
                )

            if len(trail_pts) >= min_pts and not use_direct:
                pose = odom.pose
                smoothed = self._trail.get_smoothed_trail()
                curvatures = TrailManager.compute_curvatures(smoothed)
                cmd = self._pursuit.compute(
                    pose, smoothed, speed_offset,
                    curvatures=curvatures, now=now,
                )
                if cmd is not None:
                    if self._last_pursuit_mode != "trail":
                        self._last_mode_switch_time = now
                        self._steering.reset()  # clear PID integrator on mode switch
                    self._pursuit_mode = "trail"
                    self._last_pursuit_mode = "trail"
                    self._pursuit_lookahead_x = cmd.lookahead_x
                    self._pursuit_lookahead_y = cmd.lookahead_y
                    self._curvature_at_lookahead = cmd.curvature_at_lookahead
                    self._speed_limited = cmd.speed_limited
                    steer = cmd.steer_byte

                    # Blend in direct PID steering when person is far off-centre.
                    # (blend thresholds are effectively disabled by large defaults.)
                    blend_start = float(getattr(self._cfg, "trail_direct_blend_start_m", 1.0))
                    blend_full = float(getattr(self._cfg, "trail_direct_blend_full_m", 3.0))
                    abs_x = abs(target.normalized_x)
                    if abs_x > blend_start:
                        direct_steer = self._steering.compute(target.normalized_x, dt)
                        blend = min(1.0, (abs_x - blend_start)
                                    / max(blend_full - blend_start, 0.1))
                        steer = steer * (1.0 - blend) + direct_steer * blend
                        clamp_s = float(getattr(self._cfg, "max_steer_offset_byte", 25.0))
                        steer = max(-clamp_s, min(clamp_s, steer))
                    return steer

        # Direct PID pursuit fallback
        if self._last_pursuit_mode != "direct":
            self._last_mode_switch_time = now
            self._steering.reset()
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
        steer = self._steering.compute(target.normalized_x, dt)
        # Cap direct-mode steering lower than the global max; close-range detections
        # are noisier and a full-gain correction causes overshoot.  Also scale back
        # proportionally when confidence is below 0.6 to prevent spikes on uncertain
        # detections (common when the person nearly fills the frame).
        direct_max = float(getattr(self._cfg, "direct_mode_max_steer_byte", 18.0))
        steer = max(-direct_max, min(direct_max, steer))
        if target.confidence < 0.6:
            steer *= target.confidence / 0.6
        return steer

    def _apply_slip_compensation(
        self,
        speed: float,
        steer: float,
    ) -> tuple[float, float]:
        """Detect wheel slip and compensate.

        When |left_rpm − right_rpm| exceeds the configured threshold while the
        robot is commanded straight, throttle is reduced and a small steer
        feed-forward is injected to counteract the drift direction.

        No-op when RPM telemetry is unavailable.
        """
        left_rpm = self._actual_left_rpm
        right_rpm = self._actual_right_rpm
        if left_rpm is None or right_rpm is None:
            self._last_slip_active = False
            return speed, steer

        threshold = float(getattr(self._cfg, "slip_threshold_rpm", 200.0))
        is_straight = abs(steer) < 5.0 and speed > 0.0
        rpm_diff = abs(left_rpm - right_rpm)

        if rpm_diff > threshold and is_straight:
            reduction = float(getattr(self._cfg, "slip_throttle_reduction", 0.15))
            speed = speed * (1.0 - reduction)
            # Positive diff (left > right) means left wheel is spinning faster →
            # robot drifting right → inject a small right-turn correction (positive steer)
            ff_gain = float(getattr(self._cfg, "slip_feedforward_gain", 0.02))
            steer_correction = (left_rpm - right_rpm) * ff_gain
            max_s = float(getattr(self._cfg, "max_steer_offset_byte", 25.0))
            steer = max(-max_s, min(max_s, steer + steer_correction))
            self._last_slip_active = True
        else:
            self._last_slip_active = False
        return speed, steer

    def _handle_lost_target(self, now: float) -> tuple[int, int]:
        """Handle absence of detections: trail pursuit → search → stop."""
        elapsed = now - self._last_valid_time

        if self._last_valid_time > 0.0:
            trail_max_s = float(getattr(
                self._cfg,
                "lost_target_trail_pursuit_max_s",
                self._cfg.lost_target_timeout_s,
            ))
            odom = self._pick_odometry()

            # ── Blind trail pursuit ──────────────────────────────────────────
            if (
                self._trail_enabled
                and self._pursuit is not None
                and odom is not None
                and self._trail is not None
                and elapsed < trail_max_s
            ):
                trail_pts = self._trail.get_smoothed_trail()
                self._trail_length = len(trail_pts)
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count
                min_pts = int(getattr(self._cfg, "min_trail_points_for_pursuit", 2))

                if len(trail_pts) >= min_pts:
                    pose = odom.pose
                    decay_frac = max(0.0, 1.0 - elapsed / trail_max_s)
                    fwd = max(
                        self._last_speed_offset * decay_frac,
                        self._MIN_LOST_TARGET_SPEED,
                    )
                    curvatures = TrailManager.compute_curvatures(trail_pts)
                    cmd = self._pursuit.compute(
                        pose, trail_pts, fwd,
                        curvatures=curvatures, now=now,
                    )
                    search_delay_s = float(getattr(self._cfg, "search_mode_delay_s", 1.5))
                    trail_exhausted_threshold = int(getattr(
                        self._cfg, "trail_exhausted_remaining", 3))

                    last_pt = trail_pts[-1]
                    near_trail_end = math.hypot(
                        last_pt.x - pose.x, last_pt.y - pose.y
                    ) < 3.0

                    if cmd is not None:
                        dx_lk = cmd.lookahead_x - pose.x
                        dy_lk = cmd.lookahead_y - pose.y
                        fwd_x = math.cos(pose.theta)
                        fwd_y = math.sin(pose.theta)
                        lookahead_behind = (dx_lk * fwd_x + dy_lk * fwd_y) < 0
                    else:
                        lookahead_behind = False

                    trail_exhausted = (
                        near_trail_end
                        or lookahead_behind
                        or (cmd is not None
                            and cmd.trail_remaining <= trail_exhausted_threshold)
                    )
                    trail_ok = cmd is not None and (
                        elapsed < search_delay_s or not trail_exhausted
                    )

                    if trail_ok:
                        self._pursuit_mode = "trail"
                        self._pursuit_lookahead_x = cmd.lookahead_x
                        self._pursuit_lookahead_y = cmd.lookahead_y
                        steer = cmd.steer_byte

                        # Ramp in a bias toward the last known person direction
                        # to anticipate the turn they were making.
                        last_x = self._last_target_x or 0.0
                        if abs(last_x) > 0.3:
                            bias_gain = float(getattr(self._cfg, "lost_steer_bias_gain", 3.0))
                            time_factor = min(elapsed / max(search_delay_s, 0.1), 1.5)
                            steer_bias = last_x * bias_gain * time_factor
                            max_s = float(getattr(self._cfg, "max_steer_offset_byte", 25.0))
                            steer = max(-max_s, min(max_s, steer + steer_bias))

                        self._last_steer_offset = steer
                        self._last_speed_offset = fwd
                        return _mix_commands(fwd, steer)

            # ── Gentle search rotation ────────────────────────────────────────
            if elapsed < trail_max_s:
                self._pursuit_mode = "search"
                search_steer = float(getattr(self._cfg, "search_steer_cap_byte", 30.0))
                last_x = self._last_target_x or 0.0
                steer = (
                    math.copysign(search_steer, last_x)
                    if abs(last_x) > 0.05
                    else 0.0
                )
                fwd = float(self._MIN_LOST_TARGET_SPEED)
                self._last_steer_offset = steer
                self._last_speed_offset = fwd
                return _mix_commands(fwd, steer)

        # ── Full timeout — stop and reset everything ──────────────────────────
        self._reset_tracking_state()
        return NEUTRAL, NEUTRAL

    def _reset_tracking_state(self) -> None:
        """Reset all tracking state after a full target-loss timeout."""
        self._tracking = False
        self._last_distance_error = None
        self._last_speed_offset = 0.0
        self._last_steer_offset = 0.0
        self._last_target_confidence = 0.0
        self._last_target_track_id = None
        self._cached_left = NEUTRAL
        self._cached_right = NEUTRAL
        self._prev_fresh_detection = False
        self._reacq_time = 0.0
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
        self._tracker.reset()
        self._steering.reset()
        self._safety.reset()
        if self._trail is not None:
            self._trail.clear()
        if self._pursuit is not None:
            self._pursuit.reset()
        self._trail_distance_m = 0.0
        self._trail_rejected_jump_count = 0
        self._trail_rejected_speed_count = 0
        self._last_target_world_x = None
        self._last_target_world_y = None
        if self._odometry is not None:
            self._odometry.reset()
        if self._gps_odom is not None:
            self._gps_odom.reset()

    # ── Status / telemetry ───────────────────────────────────────────────────

    def get_status(self) -> dict:
        status: dict = {
            "follow_me_tracking": self._tracking,
            "follow_me_target_z_m": self._last_target_z,
            "follow_me_target_x_m": self._last_target_x,
            "follow_me_distance_error_m": self._last_distance_error,
            "follow_me_speed_offset": self._last_speed_offset,
            "follow_me_steer_offset": self._last_steer_offset,
            "follow_me_num_detections": self._last_num_detections,
            "follow_me_target_confidence": self._last_target_confidence,
            "follow_me_target_track_id": self._last_target_track_id,
            "follow_me_pursuit_mode": self._pursuit_mode,
            "follow_me_slip_active": self._last_slip_active,
            "follow_me_actual_speed_mps": self._actual_speed_mps,
        }
        if self._trail_enabled:
            status["trail_length"] = self._trail_length
            status["trail_distance_m"] = round(self._trail_distance_m, 2)
            status["trail_rejected_jump_count"] = self._trail_rejected_jump_count
            status["trail_rejected_speed_count"] = self._trail_rejected_speed_count
            status["trail_lookahead_x"] = self._pursuit_lookahead_x
            status["trail_lookahead_y"] = self._pursuit_lookahead_y
            status["trail_curvature_at_lookahead"] = round(self._curvature_at_lookahead, 4)
            status["trail_speed_limited"] = self._speed_limited
            status["follow_me_target_world_x"] = self._last_target_world_x
            status["follow_me_target_world_y"] = self._last_target_world_y
            status["odom_source"] = self._odom_source
            odom = self._pick_odometry()
            if odom is not None:
                pose = odom.pose
                status["odom_x"] = round(pose.x, 3)
                status["odom_y"] = round(pose.y, 3)
                status["odom_theta_deg"] = round(math.degrees(pose.theta), 1)
            if self._gps_odom is not None:
                status["gps_speed_mps"] = round(self._gps_odom.speed_mps, 2)
        return status
