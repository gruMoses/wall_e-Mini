"""Breadcrumb trail manager for robot path tracking and recovery."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrailPoint:
    x: float
    y: float
    timestamp: float
    speed_hint: float


@dataclass(frozen=True)
class TrailConfig:
    max_trail_points: int = 100
    min_spacing_m: float = 0.3
    max_age_s: float = 30.0
    consume_radius_m: float = 0.4
    smoothing_enabled: bool = True
    smoothing_window: int = 5     # must be odd, >= 3
    smoothing_poly_order: int = 2  # must be < window


class TrailManager:
    def __init__(self, config: TrailConfig) -> None:
        self._config = config
        self._trail: deque[TrailPoint] = deque(maxlen=config.max_trail_points)
        # Cache Savitzky-Golay convolution coefficients (computed once)
        self._sg_coeffs: np.ndarray | None = None
        if config.smoothing_enabled and config.smoothing_window >= 3:
            self._sg_coeffs = self._compute_sg_coefficients(
                config.smoothing_window, config.smoothing_poly_order
            )

    @staticmethod
    def _compute_sg_coefficients(window: int, poly_order: int) -> np.ndarray:
        """Compute Savitzky-Golay convolution coefficients using numpy."""
        half = window // 2
        # Vandermonde matrix: rows = sample positions, cols = polynomial powers
        A = np.vander(np.arange(-half, half + 1, dtype=float), poly_order + 1, increasing=True)
        # Least-squares solution for the center point (smoothing coefficients)
        coeffs = np.linalg.lstsq(A, np.eye(window)[half], rcond=None)[0]
        return A @ coeffs

    def add_point(
        self,
        world_x: float,
        world_y: float,
        timestamp: float,
        speed_hint: float = 0.0,
    ) -> None:
        point = TrailPoint(world_x, world_y, timestamp, speed_hint)
        if not self._trail:
            self._trail.append(point)
            return
        last = self._trail[-1]
        dist = math.hypot(world_x - last.x, world_y - last.y)
        if dist >= self._config.min_spacing_m:
            self._trail.append(point)

    def prune(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        now: float,
    ) -> None:
        heading_x = math.cos(robot_theta)
        heading_y = math.sin(robot_theta)
        max_age = now - self._config.max_age_s
        consume_r = self._config.consume_radius_m

        def keep(p: TrailPoint) -> bool:
            if p.timestamp < max_age:
                return False
            dx = p.x - robot_x
            dy = p.y - robot_y
            dot = dx * heading_x + dy * heading_y
            behind = dot < 0
            dist = math.hypot(dx, dy)
            within = dist <= consume_r
            if behind and within:
                return False
            return True

        kept = [p for p in self._trail if keep(p)]
        self._trail.clear()
        for p in kept:
            self._trail.append(p)

    def get_trail(self) -> list[TrailPoint]:
        return list(self._trail)

    def get_smoothed_trail(self) -> list[TrailPoint]:
        """Return trail with Savitzky-Golay smoothing applied to x/y coordinates.

        Falls back to raw trail if smoothing is disabled or too few points.
        """
        raw = list(self._trail)
        if self._sg_coeffs is None or len(raw) < self._config.smoothing_window:
            return raw

        xs = np.array([p.x for p in raw])
        ys = np.array([p.y for p in raw])

        # Convolve with SG coefficients (mode='valid' trims edges)
        xs_smooth = np.convolve(xs, self._sg_coeffs, mode='valid')
        ys_smooth = np.convolve(ys, self._sg_coeffs, mode='valid')

        # Pad edges with original points
        half = self._config.smoothing_window // 2
        result: list[TrailPoint] = []
        for i in range(len(raw)):
            if i < half or i >= len(raw) - half:
                result.append(raw[i])
            else:
                result.append(TrailPoint(
                    x=float(xs_smooth[i - half]),
                    y=float(ys_smooth[i - half]),
                    timestamp=raw[i].timestamp,
                    speed_hint=raw[i].speed_hint,
                ))
        return result

    @staticmethod
    def compute_curvatures(trail: list[TrailPoint]) -> list[float]:
        """Compute curvature at each trail point using the three-point circle method.

        Returns a list of curvature values (same length as trail).
        First and last points get curvature 0.0.
        """
        n = len(trail)
        if n < 3:
            return [0.0] * n

        curvatures = [0.0] * n
        for i in range(1, n - 1):
            p1, p2, p3 = trail[i - 1], trail[i], trail[i + 1]
            ax, ay = p2.x - p1.x, p2.y - p1.y
            bx, by = p3.x - p2.x, p3.y - p2.y
            cross = ax * by - ay * bx
            a = math.hypot(ax, ay)
            b = math.hypot(bx, by)
            c = math.hypot(p3.x - p1.x, p3.y - p1.y)
            denom = a * b * c
            if denom > 1e-10:
                curvatures[i] = 2.0 * cross / denom
        return curvatures

    def clear(self) -> None:
        self._trail.clear()

    @property
    def length(self) -> int:
        return len(self._trail)

    def trail_distance(self) -> float:
        if len(self._trail) < 2:
            return 0.0
        total = 0.0
        prev = self._trail[0]
        for p in list(self._trail)[1:]:
            total += math.hypot(p.x - prev.x, p.y - prev.y)
            prev = p
        return total
