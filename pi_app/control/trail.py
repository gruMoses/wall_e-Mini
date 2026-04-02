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
    # Reject implausible breadcrumb jumps that usually come from transient
    # detection/transform glitches rather than true person motion.
    max_step_m: float = 1.2
    max_speed_mps: float = 2.5
    smoothing_enabled: bool = True
    smoothing_window: int = 5     # must be odd, >= 3
    smoothing_poly_order: int = 2  # must be < window


class TrailManager:
    def __init__(self, config: TrailConfig) -> None:
        self._config = config
        self._trail: deque[TrailPoint] = deque(maxlen=config.max_trail_points)
        self._rejected_jump_count: int = 0
        self._rejected_speed_count: int = 0
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

        # Reject impossible jumps (telemetry/noise spikes) before spacing logic.
        if dist > self._config.max_step_m:
            self._rejected_jump_count += 1
            return
        dt = timestamp - last.timestamp
        if dt > 1e-3 and (dist / dt) > self._config.max_speed_mps:
            self._rejected_speed_count += 1
            return

        if dist >= self._config.min_spacing_m:
            self._trail.append(point)

    def prune(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        now: float,
    ) -> int:
        """Remove stale/consumed trail points from the front. Returns count removed.

        O(k) popleft loop instead of O(n) list rebuild: old and consumed points
        are always at the front of the chronologically-ordered deque.
        """
        heading_x = math.cos(robot_theta)
        heading_y = math.sin(robot_theta)
        max_age = now - self._config.max_age_s
        consume_r = self._config.consume_radius_m

        def should_remove(p: TrailPoint) -> bool:
            if p.timestamp < max_age:
                return True
            dx = p.x - robot_x
            dy = p.y - robot_y
            dot = dx * heading_x + dy * heading_y
            if dot < 0 and math.hypot(dx, dy) <= consume_r:
                return True
            return False

        old_len = len(self._trail)
        while self._trail and should_remove(self._trail[0]):
            self._trail.popleft()
        return old_len - len(self._trail)

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

    def get_tangent_direction(self) -> tuple[float, float]:
        """Return unit tangent (dx, dy) from the last 2-3 trail points.

        Returns (0.0, 0.0) when the trail has fewer than 2 points or the last
        points are coincident.
        """
        trail = list(self._trail)
        if len(trail) < 2:
            return 0.0, 0.0
        ref_idx = max(0, len(trail) - 3)
        last = trail[-1]
        ref = trail[ref_idx]
        dx = last.x - ref.x
        dy = last.y - ref.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0
        return dx / dist, dy / dist

    def extrapolate(self, num_points: int, max_dist: float) -> int:
        """Append extra points along the last trail tangent.

        Thin wrapper around extrapolate_trail() with caller-specified
        num_points and max_dist; uses current monotonic time.
        Returns the number of points added.
        """
        import time as _time
        return self.extrapolate_trail(
            now=_time.monotonic(),
            max_dist_m=max_dist,
            num_points=num_points,
        )

    def extrapolate_trail(
        self,
        now: float,
        max_dist_m: float = 1.5,
        num_points: int = 3,
    ) -> int:
        """Append extra points along the last trail tangent.

        Called once when the target is first lost so the robot can follow
        Kevin's last known heading a bit further before entering search mode.
        Returns the number of points added.
        """
        trail = list(self._trail)
        if len(trail) < 2:
            return 0

        # Stable tangent from last 2–3 points
        ref_idx = max(0, len(trail) - 3)
        last = trail[-1]
        ref = trail[ref_idx]
        dx = last.x - ref.x
        dy = last.y - ref.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0
        tx, ty = dx / dist, dy / dist

        # Average speed_hint from recent points
        recent = trail[-3:]
        avg_speed = sum(p.speed_hint for p in recent) / len(recent)

        spacing = max(self._config.min_spacing_m, 0.3)
        added = 0
        for i in range(1, num_points + 1):
            step = spacing * i
            if step > max_dist_m:
                break
            ex = last.x + tx * step
            ey = last.y + ty * step
            # Small timestamp offset prevents immediate age-pruning
            self._trail.append(TrailPoint(ex, ey, now + i * 0.1, avg_speed))
            added += 1
        return added

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

    @property
    def rejected_jump_count(self) -> int:
        return self._rejected_jump_count

    @property
    def rejected_speed_count(self) -> int:
        return self._rejected_speed_count
