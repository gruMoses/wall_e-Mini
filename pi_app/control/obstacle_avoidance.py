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

    def compute_throttle_scale(
        self,
        distance_m: float,
        age_s: float,
        is_manual: bool = False,
        corridor_muted: bool = False,
    ) -> float:
        """Return a throttle multiplier between 0.0 (full stop) and 1.0 (no limit).

        When depth data is stale (age > stale_timeout_s), behaviour depends on
        ``stale_policy``: "stop" returns 0.0, "clear" returns 1.0.

        A person/animal within ``safety_stop_radius_m`` arrives here as
        ``distance_m`` <= ``stop_distance_m`` (forced upstream in the depth
        poll), so it naturally yields scale 0.0.

        ``corridor_muted`` (the operator's netting mute, 2026-10-08) ignores
        the depth corridor in MANUAL only: scale 1.0 for any corridor
        reading. It never applies to a stale depth (blind is blind), to the
        autonomous modes, or to the YOLO stop channel (distance 0.0 exactly).
        """
        if age_s > self._cfg.stale_timeout_s:
            if self._cfg.stale_policy == "stop":
                stale_floor = self._cfg.manual_stale_throttle_scale if is_manual else 0.0
            else:
                stale_floor = 1.0
            self._last_scale = stale_floor
            return self._last_scale

        self._last_distance_m = distance_m

        if corridor_muted and is_manual and distance_m > 0.0:
            self._last_scale = 1.0
            return self._last_scale

        manual_curve = self._manual_curve_points() if is_manual and distance_m > 0.0 else None
        if manual_curve is not None:
            scale = self._manual_curve_scale(distance_m, *manual_curve)
        elif distance_m >= self._cfg.slow_distance_m:
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

    def _manual_curve_points(self) -> tuple[float, float, float, float] | None:
        """(slow, half, floor distance, floor scale) for MANUAL, or None.

        None (use the old linear law) when manual_half_throttle_distance_m is
        0.0, when the points are not strictly ordered floor < half < slow, or
        when manual_obstacle_min_scale is 0.0 ("restore the hard stop" keeps
        its meaning: the MANUAL stop stays at stop_distance_m).
        """
        slow = float(self._cfg.slow_distance_m)
        half = float(getattr(self._cfg, "manual_half_throttle_distance_m", 0.0))
        low = float(getattr(self._cfg, "manual_floor_distance_m", 0.0))
        floor_scale = float(getattr(self._cfg, "manual_obstacle_min_scale", 0.0))
        if not (0.0 < low < half < slow) or not (0.0 < floor_scale < 0.5):
            return None
        return slow, half, low, floor_scale

    @staticmethod
    def _manual_curve_scale(
        distance_m: float, slow: float, half: float, low: float, floor_scale: float
    ) -> float:
        """Piecewise linear: 1.0 at slow, 0.5 at half, floor_scale at low."""
        if distance_m >= slow:
            return 1.0
        if distance_m >= half:
            return 0.5 + 0.5 * (distance_m - half) / (slow - half)
        if distance_m > low:
            return floor_scale + (0.5 - floor_scale) * (distance_m - low) / (half - low)
        # At or inside the floor distance, and NaN: the creep floor.
        return floor_scale

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
