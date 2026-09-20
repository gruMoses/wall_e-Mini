"""
Obstacle avoidance throttle scaling based on OAK-D Lite depth readings.

Pure logic — no hardware dependency. Computes a 0.0–1.0 throttle scale factor
from a forward distance measurement and its age.

Person/animal safety stop: the YOLO "stop" tier is applied upstream in the
depth poll (``oak_depth.OakDepthReader._apply_safety_tier_override``), which
forces the reported ``min_distance`` to 0 when a stop-tier detection is within
``safety_stop_radius_m``. That 0 flows into compute_throttle_scale() here and
yields scale 0.0 (full stop). This module deliberately holds no separate
detection state — the depth poll is the single, canonical stop tier.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))

from config import ObstacleAvoidanceConfig


class ObstacleAvoidanceController:
    def __init__(self, config: ObstacleAvoidanceConfig) -> None:
        self._cfg = config
        self._last_distance_m: float | None = None
        self._last_scale: float = 1.0

    def compute_throttle_scale(self, distance_m: float, age_s: float, is_manual: bool = False) -> float:
        """Return a throttle multiplier between 0.0 (full stop) and 1.0 (no limit).

        When depth data is stale (age > stale_timeout_s), behaviour depends on
        ``stale_policy``: "stop" returns 0.0, "clear" returns 1.0.

        A person/animal within ``safety_stop_radius_m`` arrives here as
        ``distance_m`` <= ``stop_distance_m`` (forced upstream in the depth
        poll), so it naturally yields scale 0.0.
        """
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

        scale = max(0.0, min(1.0, scale))
        # MANUAL: the corridor stop is a floor, not a wall. The operator can
        # always creep forward at manual_obstacle_min_scale, so a phantom
        # corridor obstacle cannot strand the robot (2026-09-19 19:53: max-
        # disparity noise read 0.37 m for two minutes after sunset; the robot
        # had to be backed into the garage). The YOLO person/animal stop tier
        # arrives as distance 0.0 exactly (forced upstream) and stays absolute;
        # a genuine corridor obstacle always reads >= min_depth_mm.
        if is_manual and distance_m > 0.0:
            floor = float(getattr(self._cfg, "manual_obstacle_min_scale", 0.0))
            if floor > 0.0:
                scale = max(scale, min(1.0, floor))
        self._last_scale = scale
        return self._last_scale

    def get_status(self) -> dict:
        # An empty-but-fresh corridor reports distance inf ("clear"); emit None
        # so JSON consumers (SSE, MCAP, CSV log) never see a non-finite float.
        dist = self._last_distance_m
        if dist is not None and not math.isfinite(dist):
            dist = None
        return {
            "obstacle_distance_m": dist,
            "obstacle_throttle_scale": self._last_scale,
        }
