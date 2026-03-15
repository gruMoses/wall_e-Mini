"""
Autonomous person-following controller.

Pure logic — no hardware dependency. Given a list of spatial person detections,
selects the best target and computes differential-drive motor commands to
follow the person at a configurable distance.

Supports two steering modes:
  - Direct pursuit (PD on lateral offset) — original behavior
  - Trail-following Pure Pursuit (breadcrumb path) — follows the path
    the person walked, not a straight line to their current position
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


@dataclass(frozen=True)
class PersonDetection:
    """Single person detection from OAK-D Lite spatial network."""
    x_m: float       # lateral offset from camera center (+ = right)
    z_m: float       # forward distance in meters
    confidence: float
    bbox: tuple[float, float, float, float]  # xmin, ymin, xmax, ymax (normalized 0-1)
    track_id: int | None = None


class FollowMeController:
    CENTER_WEIGHT = 0.6
    DEPTH_WEIGHT = 0.4
    TRACK_ID_STICKY_BONUS = 0.3
    _MIN_LOST_TARGET_SPEED = 5  # minimum fwd speed during lost-target trail following

    def __init__(self, config: FollowMeConfig) -> None:
        self._cfg = config
        self._tracking = False
        self._last_target_z: float | None = None
        self._last_target_x: float | None = None
        self._last_distance_error: float | None = None
        self._last_speed_offset: float = 0.0
        self._last_steer_offset: float = 0.0
        self._last_num_detections: int = 0
        self._last_target_confidence: float = 0.0
        self._last_target_track_id: int | None = None
        self._last_valid_time: float = 0.0
        self._held_left: int = NEUTRAL
        self._held_right: int = NEUTRAL
        self._smoothed_x: float = 0.0
        self._prev_smoothed_x: float = 0.0
        self._prev_x_time: float = 0.0

        # Trail-following subsystems (feature-flagged)
        self._trail_enabled = bool(getattr(config, "trail_follow_enabled", False))
        self._pursuit_mode: str = "direct"  # "direct" or "trail"
        self._last_pursuit_mode: str = "direct"  # hysteresis tracker
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
        self._odom_source: str = "none"  # "gps" or "dead_reckon" or "none"
        self._last_compute_time: float = 0.0  # for speed ramp limiting
        self._last_mode_switch_time: float = 0.0  # hysteresis for direct<->trail

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
                max_steer_byte=getattr(config, "max_steer_offset_byte", 15.0),
                max_speed_byte=float(config.max_follow_speed_byte),
                curvature_scaling_enabled=getattr(config, "pursuit_curvature_scaling_enabled", True),
                curvature_alpha=getattr(config, "pursuit_curvature_alpha", 5.0),
                min_speed_byte=getattr(config, "pursuit_min_speed_byte", 15.0),
                lookahead_curvature_points=getattr(config, "pursuit_lookahead_curvature_points", 5),
                max_accel_byte_per_s=getattr(config, "pursuit_max_accel_byte_per_s", 50.0),
            ))

    def update_pose(self, heading_deg: float, motor_l: int, motor_r: int,
                    timestamp: float) -> None:
        """Feed IMU heading and actual motor commands for odometry.

        Called by controller.py each cycle before compute().
        """
        if self._odometry is not None:
            self._odometry.update(heading_deg, motor_l, motor_r, timestamp)
        if self._gps_odom is not None:
            self._gps_odom.update_gyro(heading_deg, timestamp)

    def update_gps(self, lat: float, lon: float, fix_quality: int,
                   timestamp: float) -> None:
        """Feed GPS position for GPS-based trail odometry.

        Called by controller.py each cycle when a GPS reading is available.
        """
        if self._gps_odom is not None:
            self._gps_odom.update_gps(lat, lon, fix_quality, timestamp)

    def compute(self, detections: list[PersonDetection]) -> tuple[int, int]:
        """Compute motor bytes (left, right) to follow the best-scored person.

        Returns (NEUTRAL, NEUTRAL) when no valid target is found.
        """
        self._last_num_detections = len(detections)
        target = self._select_target(detections)
        now = time.monotonic()

        if target is None:
            return self._handle_lost_target(now)

        self._tracking = True
        self._last_valid_time = now
        self._last_target_z = target.z_m
        self._last_target_x = target.x_m
        self._last_target_confidence = target.confidence
        self._last_target_track_id = target.track_id

        follow_dist = self._cfg.follow_distance_m

        if target.z_m <= self._cfg.min_distance_m:
            self._last_distance_error = target.z_m - follow_dist
            self._last_speed_offset = 0.0
            self._last_steer_offset = 0.0
            return NEUTRAL, NEUTRAL

        # Speed: proportional to distance error
        distance_error = target.z_m - follow_dist
        self._last_distance_error = distance_error
        if distance_error <= 0:
            speed_offset = 0.0
        else:
            max_speed_err = getattr(self._cfg, "max_speed_error_m", 2.5)
            speed_gain = self._cfg.max_follow_speed_byte / max(max_speed_err, 0.1)
            speed_offset = min(
                distance_error * speed_gain,
                float(self._cfg.max_follow_speed_byte),
            )
        # Ramp-limit speed increases ONLY when recovering from search/slow mode.
        # Normal tracking should accelerate freely; the ramp prevents lunging
        # after re-acquisition from near-zero speed (search mode).
        ramp_threshold = getattr(self._cfg, "speed_ramp_threshold_byte", 30.0)
        if self._last_speed_offset < ramp_threshold:
            max_accel = getattr(self._cfg, "max_speed_accel_byte_per_s", 150.0)
            dt = now - self._last_compute_time if self._last_compute_time > 0 else 0.0
            # Clamp dt to one control cycle when stale (e.g. after search mode gap)
            if dt > 0:
                dt = min(dt, 0.05)  # treat gaps as single 50Hz cycle
                max_increase = max_accel * dt
                speed_offset = min(speed_offset,
                                   self._last_speed_offset + max_increase)
        self._last_compute_time = now
        self._last_speed_offset = speed_offset

        # Add person to trail (world-frame breadcrumb)
        if self._trail_enabled and self._trail is not None:
            # Prefer GPS odometry when it has a fix, fall back to dead reckoning
            odom = self._pick_odometry()
            if odom is not None:
                wx, wy = odom.camera_to_world(target.x_m, target.z_m)
                self._last_target_world_x = wx
                self._last_target_world_y = wy
                self._trail.add_point(wx, wy, now, speed_hint=speed_offset)
                pose = odom.pose
                self._trail.prune(pose.x, pose.y, pose.theta, now)
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count

        # Decide steering mode: trail pursuit vs direct
        steer_offset = self._compute_steering(target, speed_offset, now)

        self._last_steer_offset = steer_offset

        left = NEUTRAL + speed_offset + steer_offset
        right = NEUTRAL + speed_offset - steer_offset

        left_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(left))))
        right_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(right))))
        self._held_left = left_out
        self._held_right = right_out
        return left_out, right_out

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

    def _compute_steering(self, target: PersonDetection, speed_offset: float,
                          now: float) -> float:
        """Choose between trail pursuit and direct pursuit for steering."""
        use_trail = False
        odom = self._pick_odometry()
        mode_dwell_s = getattr(self._cfg, "mode_switch_dwell_s", 0.5)

        if (self._trail_enabled and self._pursuit is not None
                and odom is not None and self._trail is not None):
            trail_pts = self._trail.get_smoothed_trail()
            self._trail_length = len(trail_pts)
            min_pts = getattr(self._cfg, "min_trail_points_for_pursuit", 2)
            direct_dist = getattr(self._cfg, "direct_pursuit_distance_m", 2.0)
            direct_lat = getattr(self._cfg, "direct_pursuit_lateral_m", 0.3)

            # Hysteresis: don't switch modes more often than mode_dwell_s
            time_in_mode = now - self._last_mode_switch_time
            if self._last_pursuit_mode == "trail":
                use_direct = (
                    target.z_m < direct_dist
                    and abs(target.x_m) < direct_lat
                    and time_in_mode >= mode_dwell_s
                )
            else:
                use_direct = not (
                    (target.z_m > direct_dist * 1.3
                     or abs(target.x_m) > direct_lat * 1.5)
                    and time_in_mode >= mode_dwell_s
                )

            if len(trail_pts) >= min_pts and not use_direct:
                pose = odom.pose
                # Use smoothed trail and curvature for better tracking
                smoothed = self._trail.get_smoothed_trail()
                curvatures = TrailManager.compute_curvatures(smoothed)
                cmd = self._pursuit.compute(
                    pose, smoothed, speed_offset,
                    curvatures=curvatures, now=now,
                )
                if cmd is not None:
                    use_trail = True
                    if self._last_pursuit_mode != "trail":
                        self._last_mode_switch_time = now
                    self._pursuit_mode = "trail"
                    self._last_pursuit_mode = "trail"
                    self._pursuit_lookahead_x = cmd.lookahead_x
                    self._pursuit_lookahead_y = cmd.lookahead_y
                    self._curvature_at_lookahead = cmd.curvature_at_lookahead
                    self._speed_limited = cmd.speed_limited
                    steer = cmd.steer_byte
                    # When person is significantly off-center, the trail
                    # geometry may lag behind their actual position (stale
                    # breadcrumbs from before a turn).  Blend in direct
                    # pursuit steering so the robot reacts to WHERE the
                    # person IS, not just where the trail points.
                    blend_start_m = getattr(
                        self._cfg, "trail_direct_blend_start_m", 1.0)
                    blend_full_m = getattr(
                        self._cfg, "trail_direct_blend_full_m", 3.0)
                    abs_x = abs(target.x_m)
                    if abs_x > blend_start_m:
                        direct_steer = self._direct_pursuit_steer(
                            target, now)
                        blend = min(1.0, (abs_x - blend_start_m)
                                    / max(blend_full_m - blend_start_m, 0.1))
                        steer = steer * (1.0 - blend) + direct_steer * blend
                        max_s = getattr(
                            self._cfg, "max_steer_offset_byte", 25.0)
                        steer = max(-max_s, min(max_s, steer))
                    return steer

        # Direct pursuit fallback: PD on lateral offset
        if self._last_pursuit_mode != "direct":
            self._last_mode_switch_time = now
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
        return self._direct_pursuit_steer(target, now)

    def _direct_pursuit_steer(self, target: PersonDetection, now: float) -> float:
        """Original PD steering on lateral offset."""
        alpha = getattr(self._cfg, "steering_ema_alpha", 0.4)
        if self._prev_x_time == 0.0:
            self._smoothed_x = target.x_m
        else:
            self._smoothed_x = alpha * target.x_m + (1.0 - alpha) * self._smoothed_x

        dt = now - self._prev_x_time if self._prev_x_time > 0.0 else 0.0
        dx_dt = (self._smoothed_x - self._prev_smoothed_x) / dt if dt > 0.01 else 0.0
        self._prev_smoothed_x = self._smoothed_x
        self._prev_x_time = now

        scale = self._cfg.max_follow_speed_byte / max(self._cfg.max_distance_m, 0.1)
        p_steer = target.x_m * self._cfg.steering_gain * scale
        d_gain = getattr(self._cfg, "steering_derivative_gain", 0.0)
        d_steer = dx_dt * d_gain * scale
        steer_offset = p_steer + d_steer

        max_abs = getattr(self._cfg, "max_steer_offset_byte", 1e9)
        steer_offset = max(-max_abs, min(max_abs, steer_offset))
        return steer_offset

    def _handle_lost_target(self, now: float) -> tuple[int, int]:
        """Handle when no person is detected."""
        elapsed = now - self._last_valid_time

        if self._last_valid_time > 0.0:
            # Trail-follow blind pursuit can continue longer than short detection timeout.
            # This lets the robot keep following where the person likely went around turns.
            trail_max_s = getattr(self._cfg, "lost_target_trail_pursuit_max_s", self._cfg.lost_target_timeout_s)
            odom = self._pick_odometry()
            if (self._trail_enabled and self._pursuit is not None
                    and odom is not None and self._trail is not None
                    and elapsed < trail_max_s):
                trail_pts = self._trail.get_smoothed_trail()
                self._trail_length = len(trail_pts)
                self._trail_distance_m = self._trail.trail_distance()
                self._trail_rejected_jump_count = self._trail.rejected_jump_count
                self._trail_rejected_speed_count = self._trail.rejected_speed_count
                min_pts = getattr(self._cfg, "min_trail_points_for_pursuit", 2)
                if len(trail_pts) >= min_pts:
                    pose = odom.pose
                    # Decay speed linearly: start at last speed, reach minimum
                    # at trail_max_s.  Robot slows as it loses confidence.
                    decay_frac = max(0.0, 1.0 - elapsed / trail_max_s)
                    fwd = max(self._last_speed_offset * decay_frac,
                              self._MIN_LOST_TARGET_SPEED)
                    curvatures = TrailManager.compute_curvatures(trail_pts)
                    cmd = self._pursuit.compute(
                        pose, trail_pts, fwd,
                        curvatures=curvatures, now=now,
                    )
                    # Only consider trail exhausted after person has been
                    # lost for a minimum time.  Brief detection dropouts
                    # (< 1s) should keep following the trail, not pivot.
                    search_delay_s = getattr(
                        self._cfg, "search_mode_delay_s", 1.5)
                    trail_exhausted_threshold = getattr(
                        self._cfg, "trail_exhausted_remaining", 3)
                    # Check multiple exhaustion signals:
                    # 1. Close to the last breadcrumb
                    last_pt = trail_pts[-1]
                    dist_to_end = math.hypot(
                        last_pt.x - pose.x, last_pt.y - pose.y)
                    near_trail_end = dist_to_end < 3.0
                    # 2. Lookahead is behind the robot (dot product < 0)
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
                        or cmd.trail_remaining <= trail_exhausted_threshold)
                    trail_ok = (cmd is not None and (
                        elapsed < search_delay_s
                        or not trail_exhausted))
                    if trail_ok:
                        self._pursuit_mode = "trail"
                        self._pursuit_lookahead_x = cmd.lookahead_x
                        self._pursuit_lookahead_y = cmd.lookahead_y
                        steer = cmd.steer_byte
                        # Bias blind trail steering toward last known person
                        # direction.  The trail geometry may be straight but
                        # the person was last seen off-center — anticipate
                        # the turn.  Bias grows with elapsed time since loss.
                        last_x = self._last_target_x or 0.0
                        if abs(last_x) > 0.3:
                            bias_gain = getattr(
                                self._cfg, "lost_steer_bias_gain", 3.0)
                            # Ramp bias: stronger as time passes without
                            # new detection (stale trail becomes less useful)
                            time_factor = min(elapsed / max(search_delay_s, 0.1), 1.5)
                            steer_bias = last_x * bias_gain * time_factor
                            max_s = getattr(
                                self._cfg, "max_steer_offset_byte", 25.0)
                            steer = max(-max_s, min(max_s, steer + steer_bias))
                        left = NEUTRAL + fwd + steer
                        right = NEUTRAL + fwd - steer
                        left_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(left))))
                        right_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(right))))
                        return left_out, right_out

            # Fallback: slow rotation toward last-seen direction.
            # Nearly zero forward speed — scan, don't drive.
            # Don't reset _last_speed_offset — preserve it so re-acquisition
            # resumes at the previous speed without ramping.  The final slew
            # limiter (SlewLimiterConfig) handles motor-level transitions.
            if elapsed < trail_max_s:
                self._pursuit_mode = "search"
                search_steer_cap = getattr(self._cfg, "search_steer_cap_byte", 30.0)
                if self._last_target_x is not None and abs(self._last_target_x) > 0.05:
                    direction = 1.0 if self._last_target_x > 0 else -1.0
                    steer = direction * search_steer_cap
                else:
                    steer = 0.0
                # Crawl forward slowly during search — just enough to not stall
                fwd = self._MIN_LOST_TARGET_SPEED
                left = max(MIN_OUTPUT, min(MAX_OUTPUT, int(NEUTRAL + fwd + steer)))
                right = max(MIN_OUTPUT, min(MAX_OUTPUT, int(NEUTRAL + fwd - steer)))
                return left, right

        # Full timeout — stop and reset
        self._tracking = False
        self._last_distance_error = None
        self._last_speed_offset = 0.0
        self._last_steer_offset = 0.0
        self._last_target_confidence = 0.0
        self._last_target_track_id = None
        self._held_left = NEUTRAL
        self._held_right = NEUTRAL
        self._pursuit_mode = "direct"
        self._last_pursuit_mode = "direct"
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
        return NEUTRAL, NEUTRAL

    def _select_target(
        self, detections: list[PersonDetection]
    ) -> PersonDetection | None:
        """Pick the person most likely to be the intended follow target."""
        best: PersonDetection | None = None
        best_score = -1.0

        for det in detections:
            if det.confidence < self._cfg.detection_confidence:
                continue
            if det.z_m < self._cfg.min_distance_m or det.z_m > self._cfg.max_distance_m:
                continue

            bbox_cx = (det.bbox[0] + det.bbox[2]) / 2.0
            center_closeness = 1.0 - abs(bbox_cx - 0.5) * 2.0
            center_closeness = max(0.0, min(1.0, center_closeness))

            depth_closeness = 1.0 - (det.z_m / self._cfg.max_distance_m)
            depth_closeness = max(0.0, min(1.0, depth_closeness))

            score = (
                self.CENTER_WEIGHT * center_closeness
                + self.DEPTH_WEIGHT * depth_closeness
            )
            if (
                det.track_id is not None
                and self._last_target_track_id is not None
                and det.track_id == self._last_target_track_id
            ):
                score += self.TRACK_ID_STICKY_BONUS
            if score > best_score:
                best_score = score
                best = det

        return best

    def get_status(self) -> dict:
        status = {
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
