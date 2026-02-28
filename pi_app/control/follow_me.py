"""
Autonomous person-following controller.

Pure logic — no hardware dependency. Given a list of spatial person detections,
selects the best target and computes differential-drive motor commands to
follow the person at a configurable distance.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import FollowMeConfig

NEUTRAL = 126


@dataclass(frozen=True)
class PersonDetection:
    """Single person detection from OAK-D Lite spatial network."""
    x_m: float       # lateral offset from camera center (+ = right)
    z_m: float       # forward distance in meters
    confidence: float
    bbox: tuple[float, float, float, float]  # xmin, ymin, xmax, ymax (normalized 0-1)


class FollowMeController:
    CENTER_WEIGHT = 0.6
    DEPTH_WEIGHT = 0.4

    def __init__(self, config: FollowMeConfig) -> None:
        self._cfg = config
        self._tracking = False
        self._last_target_z: float | None = None
        self._last_target_x: float | None = None

    def compute(self, detections: list[PersonDetection]) -> tuple[int, int]:
        """Compute motor bytes (left, right) to follow the best-scored person.

        Returns (NEUTRAL, NEUTRAL) when no valid target is found.
        """
        target = self._select_target(detections)
        if target is None:
            self._tracking = False
            return NEUTRAL, NEUTRAL

        self._tracking = True
        self._last_target_z = target.z_m
        self._last_target_x = target.x_m

        # Too close — stop to avoid crowding the person
        if target.z_m <= self._cfg.min_distance_m:
            return NEUTRAL, NEUTRAL

        # Speed: proportional to distance error from follow distance
        distance_error = target.z_m - self._cfg.follow_distance_m
        if distance_error <= 0:
            speed_offset = 0.0
        else:
            speed_gain = self._cfg.max_follow_speed_byte / max(
                self._cfg.max_distance_m - self._cfg.follow_distance_m, 0.1
            )
            speed_offset = min(
                distance_error * speed_gain,
                float(self._cfg.max_follow_speed_byte),
            )

        # Steering: proportional to lateral offset; positive x_m = person is right
        # Positive steer_offset turns the robot right (add to right, subtract from left)
        steer_offset = (
            target.x_m
            * self._cfg.steering_gain
            * self._cfg.max_follow_speed_byte
            / max(self._cfg.max_distance_m, 0.1)
        )

        left = NEUTRAL + speed_offset + steer_offset
        right = NEUTRAL + speed_offset - steer_offset

        return (
            max(0, min(255, int(round(left)))),
            max(0, min(255, int(round(right)))),
        )

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
            if score > best_score:
                best_score = score
                best = det

        return best

    def get_status(self) -> dict:
        return {
            "follow_me_tracking": self._tracking,
            "follow_me_target_z_m": self._last_target_z,
            "follow_me_target_x_m": self._last_target_x,
        }
