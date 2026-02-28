"""
Obstacle avoidance throttle scaling based on OAK-D Lite depth readings.

Pure logic — no hardware dependency. Computes a 0.0–1.0 throttle scale factor
from a forward distance measurement and its age.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig


class ObstacleAvoidanceController:
    def __init__(self, config: ObstacleAvoidanceConfig) -> None:
        self._cfg = config
        self._last_distance_m: float | None = None
        self._last_scale: float = 1.0

    def compute_throttle_scale(self, distance_m: float, age_s: float) -> float:
        """Return a throttle multiplier between 0.0 (full stop) and 1.0 (no limit).

        When depth data is stale (age > stale_timeout_s), behaviour depends on
        ``stale_policy``: "stop" returns 0.0, "clear" returns 1.0.
        """
        if age_s > self._cfg.stale_timeout_s:
            self._last_scale = 0.0 if self._cfg.stale_policy == "stop" else 1.0
            return self._last_scale

        self._last_distance_m = distance_m

        if distance_m >= self._cfg.slow_distance_m:
            scale = 1.0
        elif distance_m <= self._cfg.stop_distance_m:
            scale = 0.0
        else:
            rng = self._cfg.slow_distance_m - self._cfg.stop_distance_m
            scale = (distance_m - self._cfg.stop_distance_m) / rng

        self._last_scale = max(0.0, min(1.0, scale))
        return self._last_scale

    def get_status(self) -> dict:
        return {
            "obstacle_distance_m": self._last_distance_m,
            "obstacle_throttle_scale": self._last_scale,
        }
