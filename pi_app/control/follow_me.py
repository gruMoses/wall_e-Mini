"""
Autonomous person-following controller.

Pure logic — no hardware dependency. Given a list of spatial person detections,
selects the best target and computes differential-drive motor commands to
follow the person at a configurable distance.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig
from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT

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

    def __init__(self, config: FollowMeConfig) -> None:
        self._cfg = config
        self._tracking = False
        self._dynamic_follow_dist: float | None = None
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
        self._prev_steer_offset: float = 0.0
        self._prev_steer_time: float = 0.0

    def compute(self, detections: list[PersonDetection]) -> tuple[int, int]:
        """Compute motor bytes (left, right) to follow the best-scored person.

        Returns (NEUTRAL, NEUTRAL) when no valid target is found.
        """
        self._last_num_detections = len(detections)
        target = self._select_target(detections)
        if target is None:
            elapsed = time.monotonic() - self._last_valid_time
            if self._last_valid_time > 0.0 and elapsed < self._cfg.lost_target_timeout_s:
                search_pct = getattr(self._cfg, "lost_target_search_steer_pct", 0.25)
                max_steer = self._cfg.max_follow_speed_byte * search_pct
                if self._last_target_x is not None and abs(self._last_target_x) > 0.05:
                    direction = 1.0 if self._last_target_x > 0 else -1.0
                    steer = direction * max_steer
                else:
                    steer = 0.0
                left = max(MIN_OUTPUT, min(MAX_OUTPUT, int(NEUTRAL + steer)))
                right = max(MIN_OUTPUT, min(MAX_OUTPUT, int(NEUTRAL - steer)))
                return left, right
            self._tracking = False
            self._dynamic_follow_dist = None
            self._last_distance_error = None
            self._last_speed_offset = 0.0
            self._last_steer_offset = 0.0
            self._last_target_confidence = 0.0
            self._last_target_track_id = None
            self._held_left = NEUTRAL
            self._held_right = NEUTRAL
            return NEUTRAL, NEUTRAL

        if not self._tracking:
            self._dynamic_follow_dist = target.z_m
        self._tracking = True
        self._last_valid_time = time.monotonic()
        self._last_target_z = target.z_m
        self._last_target_x = target.x_m
        self._last_target_confidence = target.confidence
        self._last_target_track_id = target.track_id

        follow_dist = self._dynamic_follow_dist or self._cfg.follow_distance_m

        # Too close — stop to avoid crowding the person
        if target.z_m <= self._cfg.min_distance_m:
            self._last_distance_error = target.z_m - follow_dist
            self._last_speed_offset = 0.0
            self._last_steer_offset = 0.0
            return NEUTRAL, NEUTRAL

        # Speed: proportional to distance error from follow distance
        distance_error = target.z_m - follow_dist
        self._last_distance_error = distance_error
        if distance_error <= 0:
            speed_offset = 0.0
        else:
            speed_gain = self._cfg.max_follow_speed_byte / max(
                self._cfg.max_distance_m - follow_dist, 0.1
            )
            speed_offset = min(
                distance_error * speed_gain,
                float(self._cfg.max_follow_speed_byte),
            )
        self._last_speed_offset = speed_offset

        # Steering: PD control on lateral offset for damped tracking.
        # P term: proportional to x_m (turns toward person)
        # D term: proportional to dx/dt (brakes the swing before overshoot)
        now = time.monotonic()
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

        max_rate = getattr(self._cfg, "max_steer_delta_per_s", 1e9)
        max_abs = getattr(self._cfg, "max_steer_offset_byte", 1e9)
        dt_steer = now - self._prev_steer_time if self._prev_steer_time > 0.0 else 0.0
        if dt_steer > 0.001 and max_rate < 1e6:
            max_delta = max_rate * dt_steer
            delta = steer_offset - self._prev_steer_offset
            if abs(delta) > max_delta:
                steer_offset = self._prev_steer_offset + max_delta * (1.0 if delta > 0 else -1.0)
        steer_offset = max(-max_abs, min(max_abs, steer_offset))
        self._prev_steer_offset = steer_offset
        self._prev_steer_time = now

        self._last_steer_offset = steer_offset

        left = NEUTRAL + speed_offset + steer_offset
        right = NEUTRAL + speed_offset - steer_offset

        left_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(left))))
        right_out = max(MIN_OUTPUT, min(MAX_OUTPUT, int(round(right))))
        self._held_left = left_out
        self._held_right = right_out
        return left_out, right_out

    def _select_target(
        self, detections: list[PersonDetection]
    ) -> PersonDetection | None:
        """Pick the person most likely to be the intended follow target.

        Scoring: weighted combination of proximity to frame center (bbox) and
        closeness in depth.  Detections outside the configured distance range
        or below the confidence threshold are discarded.
        """
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
                # Prefer continuity when the tracked target remains valid.
                score += self.TRACK_ID_STICKY_BONUS
            if score > best_score:
                best_score = score
                best = det

        return best

    def get_status(self) -> dict:
        return {
            "follow_me_tracking": self._tracking,
            "follow_me_target_z_m": self._last_target_z,
            "follow_me_target_x_m": self._last_target_x,
            "follow_me_follow_dist_m": self._dynamic_follow_dist,
            "follow_me_distance_error_m": self._last_distance_error,
            "follow_me_speed_offset": self._last_speed_offset,
            "follow_me_steer_offset": self._last_steer_offset,
            "follow_me_num_detections": self._last_num_detections,
            "follow_me_target_confidence": self._last_target_confidence,
            "follow_me_target_track_id": self._last_target_track_id,
        }
