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
        self._trail: TrailManager | None = None
        self._pursuit: PurePursuitController | None = None
        self._pursuit_lookahead_x: float = 0.0
        self._pursuit_lookahead_y: float = 0.0
        self._trail_length: int = 0

        if self._trail_enabled:
            self._odometry = DeadReckonOdometry(
                speed_scale=getattr(config, "trail_speed_scale_mps_per_byte", 0.01)
            )
            self._trail = TrailManager(TrailConfig(
                max_trail_points=getattr(config, "trail_max_points", 100),
                min_spacing_m=getattr(config, "trail_min_spacing_m", 0.3),
                max_age_s=getattr(config, "trail_max_age_s", 30.0),
                consume_radius_m=getattr(config, "trail_consume_radius_m", 0.4),
            ))
            self._pursuit = PurePursuitController(PursuitConfig(
                lookahead_base_m=getattr(config, "pursuit_lookahead_base_m", 1.0),
                lookahead_speed_scale=getattr(config, "pursuit_lookahead_speed_scale", 0.005),
                wheelbase_m=getattr(config, "pursuit_wheelbase_m", 0.28),
                max_steer_byte=getattr(config, "max_steer_offset_byte", 15.0),
                max_speed_byte=float(config.max_follow_speed_byte),
            ))

    def update_pose(self, heading_deg: float, motor_l: int, motor_r: int,
                    timestamp: float) -> None:
        """Feed IMU heading and actual motor commands for odometry.

        Called by controller.py each cycle before compute().
        """
        if self._odometry is not None:
            self._odometry.update(heading_deg, motor_l, motor_r, timestamp)

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
        self._last_speed_offset = speed_offset

        # Add person to trail (world-frame breadcrumb)
        if self._trail_enabled and self._odometry is not None and self._trail is not None:
            wx, wy = self._odometry.camera_to_world(target.x_m, target.z_m)
            self._trail.add_point(wx, wy, now, speed_hint=speed_offset)
            pose = self._odometry.pose
            self._trail.prune(pose.x, pose.y, pose.theta, now)

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

    def _compute_steering(self, target: PersonDetection, speed_offset: float,
                          now: float) -> float:
        """Choose between trail pursuit and direct pursuit for steering."""
        use_trail = False

        if (self._trail_enabled and self._pursuit is not None
                and self._odometry is not None and self._trail is not None):
            trail_pts = self._trail.get_trail()
            self._trail_length = len(trail_pts)
            min_pts = getattr(self._cfg, "min_trail_points_for_pursuit", 2)
            direct_dist = getattr(self._cfg, "direct_pursuit_distance_m", 2.0)
            direct_lat = getattr(self._cfg, "direct_pursuit_lateral_m", 0.3)

            # Hysteresis: use different thresholds for entering vs exiting
            # trail mode to avoid flip-flopping when person oscillates near
            # the threshold boundary.
            if self._last_pursuit_mode == "trail":
                # Currently in trail mode — switch to direct only when
                # person is clearly close AND centered (tighter thresholds).
                use_direct = (
                    target.z_m < direct_dist
                    and abs(target.x_m) < direct_lat
                )
            else:
                # Currently in direct mode — switch to trail only when
                # person is clearly far OR off-center (wider thresholds).
                use_direct = not (
                    target.z_m > direct_dist * 1.3
                    or abs(target.x_m) > direct_lat * 1.5
                )

            if len(trail_pts) >= min_pts and not use_direct:
                pose = self._odometry.pose
                cmd = self._pursuit.compute(pose, trail_pts, speed_offset)
                if cmd is not None:
                    use_trail = True
                    self._pursuit_mode = "trail"
                    self._last_pursuit_mode = "trail"
                    self._pursuit_lookahead_x = cmd.lookahead_x
                    self._pursuit_lookahead_y = cmd.lookahead_y
                    return cmd.steer_byte

        # Direct pursuit fallback: PD on lateral offset
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

        if self._last_valid_time > 0.0 and elapsed < self._cfg.lost_target_timeout_s:
            # Trail following: continue along the trail
            if (self._trail_enabled and self._pursuit is not None
                    and self._odometry is not None and self._trail is not None):
                trail_pts = self._trail.get_trail()
                self._trail_length = len(trail_pts)
                min_pts = getattr(self._cfg, "min_trail_points_for_pursuit", 2)
                if len(trail_pts) >= min_pts:
                    pose = self._odometry.pose
                    fwd = max(self._last_speed_offset * 0.5,
                              self._MIN_LOST_TARGET_SPEED)
                    cmd = self._pursuit.compute(pose, trail_pts, fwd)
                    if cmd is not None:
                        self._pursuit_mode = "trail"
                        self._pursuit_lookahead_x = cmd.lookahead_x
                        self._pursuit_lookahead_y = cmd.lookahead_y
                        left = NEUTRAL + fwd + cmd.steer_byte
                        right = NEUTRAL + fwd - cmd.steer_byte
                        left_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(left))))
                        right_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(right))))
                        return left_out, right_out

            # Fallback: original search turn
            search_pct = getattr(self._cfg, "lost_target_search_steer_pct", 0.25)
            max_steer = self._cfg.max_follow_speed_byte * search_pct
            steer_cap = getattr(self._cfg, "max_steer_offset_byte", 1e9)
            if steer_cap < 1e6:
                max_steer = min(max_steer, steer_cap)
            if self._last_target_x is not None and abs(self._last_target_x) > 0.05:
                direction = 1.0 if self._last_target_x > 0 else -1.0
                steer = direction * max_steer
            else:
                steer = 0.0
            fwd = self._last_speed_offset * 0.5
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
        if self._odometry is not None:
            self._odometry.reset()
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
            status["trail_lookahead_x"] = self._pursuit_lookahead_x
            status["trail_lookahead_y"] = self._pursuit_lookahead_y
            if self._odometry is not None:
                pose = self._odometry.pose
                status["odom_x"] = round(pose.x, 3)
                status["odom_y"] = round(pose.y, 3)
                status["odom_theta_deg"] = round(math.degrees(pose.theta), 1)
        return status
