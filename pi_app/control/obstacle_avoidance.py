"""
Obstacle avoidance throttle scaling based on OAK-D Lite depth readings
and YOLO safety-tier detections.

Pure logic — no hardware dependency. Computes a 0.0–1.0 throttle scale factor
from a forward distance measurement, its age, and optional safety-tier overrides.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig


class ObstacleAvoidanceController:
    def __init__(self, config: ObstacleAvoidanceConfig) -> None:
        self._cfg = config
        self._last_distance_m: float | None = None
        self._last_scale: float = 1.0
        self._safety_force_stop: bool = False
        self._safety_stop_ts: float = 0.0

    def set_safety_detections(self, detections: list) -> None:
        """Accept YOLO safety-tier detections and latch force-stop if needed.

        Each detection should have .safety_tier ('stop'/'slow'/'log') and .z_m.
        Called from the main loop after get_all_detections().
        """
        radius = float(getattr(self._cfg, "safety_stop_radius_m", 0.8))
        for det in detections:
            if getattr(det, "safety_tier", "") == "stop" and getattr(det, "z_m", 0) > 0:
                if det.z_m < radius:
                    self._safety_force_stop = True
                    self._safety_stop_ts = time.monotonic()
                    return
        self._safety_force_stop = False

    def compute_throttle_scale(self, distance_m: float, age_s: float, is_manual: bool = False) -> float:
        """Return a throttle multiplier between 0.0 (full stop) and 1.0 (no limit).

        When depth data is stale (age > stale_timeout_s), behaviour depends on
        ``stale_policy``: "stop" returns 0.0, "clear" returns 1.0.

        Safety-tier force-stop (from set_safety_detections) overrides all other
        logic and returns 0.0.
        """
        # Safety-tier hard stop takes absolute priority.
        if self._safety_force_stop:
            self._last_distance_m = distance_m
            self._last_scale = 0.0
            return 0.0

        if age_s > self._cfg.stale_timeout_s:
            if self._cfg.stale_policy == "stop":
                stale_floor = self._cfg.manual_stale_throttle_scale if is_manual else 0.0
            else:
                stale_floor = 1.0
            self._last_scale = stale_floor
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
            "safety_force_stop": self._safety_force_stop,
        }
