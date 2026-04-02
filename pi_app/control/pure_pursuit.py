"""Pure Pursuit path-following controller for differential-drive robots."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .odometry import RobotPose
from .trail import TrailPoint


@dataclass(frozen=True)
class PursuitConfig:
    # Adaptive lookahead: lookahead = clamp(speed_mps * time_s, min_m, max_m)
    lookahead_time_s: float = 0.8
    lookahead_min_m: float = 0.5
    lookahead_max_m: float = 2.5
    speed_scale_mps_per_byte: float = 0.01  # convert speed_byte to m/s for lookahead
    wheelbase_m: float = 0.28
    max_steer_byte: float = 15.0
    max_speed_byte: float = 115.0
    # Forward-only closest point search
    closest_point_search_window: int = 20
    # Curvature-based velocity scaling
    curvature_scaling_enabled: bool = True
    curvature_alpha: float = 5.0           # higher = more decel in turns
    min_speed_byte: float = 15.0           # floor speed in tight turns
    lookahead_curvature_points: int = 5    # look ahead for pre-deceleration
    max_accel_byte_per_s: float = 50.0     # smooth speed transitions


@dataclass(frozen=True)
class PursuitCommand:
    speed_byte: float
    steer_byte: float
    lookahead_x: float
    lookahead_y: float
    trail_remaining: int
    speed_limited: bool = False
    curvature_at_lookahead: float = 0.0


class PurePursuitController:
    def __init__(self, config: PursuitConfig) -> None:
        self._config = config
        self._last_closest_idx: int = 0
        self._last_speed_byte: float = 0.0
        self._last_compute_time: float = 0.0

    def reset(self) -> None:
        """Reset state when trail is cleared."""
        self._last_closest_idx = 0
        self._last_speed_byte = 0.0
        self._last_compute_time = 0.0

    @property
    def last_closest_idx(self) -> int:
        """Last closest-point index for observability."""
        return self._last_closest_idx

    def adjust_for_prune(self, num_removed: int) -> None:
        """Shift _last_closest_idx down after num_removed points pruned from trail front."""
        if num_removed <= 0:
            return
        self._last_closest_idx = max(0, self._last_closest_idx - num_removed)

    def compute(
        self,
        pose: RobotPose,
        trail: list[TrailPoint],
        speed_byte: float,
        curvatures: list[float] | None = None,
        now: float = 0.0,
    ) -> PursuitCommand | None:
        if len(trail) < 2:
            return None

        # Adaptive lookahead
        cfg = self._config
        speed_mps = speed_byte * cfg.speed_scale_mps_per_byte
        lookahead_dist = max(cfg.lookahead_min_m,
                             min(cfg.lookahead_max_m, speed_mps * cfg.lookahead_time_s))

        # Forward-only closest-point search
        best_idx = self._find_closest_forward(pose, trail)

        # Walk along trail from closest point to find lookahead target
        accumulated = 0.0
        lookahead_idx = best_idx
        lookahead_x = trail[best_idx].x
        lookahead_y = trail[best_idx].y

        for i in range(best_idx, len(trail) - 1):
            dx = trail[i + 1].x - trail[i].x
            dy = trail[i + 1].y - trail[i].y
            seg_len = math.hypot(dx, dy)
            if accumulated + seg_len >= lookahead_dist:
                t = (lookahead_dist - accumulated) / seg_len if seg_len > 0 else 1.0
                lookahead_x = trail[i].x + t * dx
                lookahead_y = trail[i].y + t * dy
                lookahead_idx = i + 1
                break
            accumulated += seg_len
            lookahead_idx = i + 1
            lookahead_x = trail[i + 1].x
            lookahead_y = trail[i + 1].y

        trail_remaining = len(trail) - lookahead_idx

        # Transform lookahead point to robot-local frame
        dx = lookahead_x - pose.x
        dy = lookahead_y - pose.y
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c

        dist_sq = local_x * local_x + local_y * local_y
        if dist_sq < 0.01 * 0.01:
            return PursuitCommand(
                speed_byte=speed_byte,
                steer_byte=0.0,
                lookahead_x=lookahead_x,
                lookahead_y=lookahead_y,
                trail_remaining=trail_remaining,
            )

        # Curvature-based velocity scaling
        actual_speed = speed_byte
        speed_limited = False
        curvature_at_lookahead = 0.0
        if (curvatures is not None and cfg.curvature_scaling_enabled
                and len(curvatures) == len(trail)):
            actual_speed, speed_limited, curvature_at_lookahead = (
                self._compute_speed(curvatures, lookahead_idx, speed_byte, now)
            )

        # Pure pursuit steering
        kappa = 2.0 * local_y / dist_sq
        steer_byte = kappa * cfg.wheelbase_m * actual_speed / 2.0
        steer_byte = max(-cfg.max_steer_byte, min(cfg.max_steer_byte, steer_byte))

        return PursuitCommand(
            speed_byte=actual_speed,
            steer_byte=steer_byte,
            lookahead_x=lookahead_x,
            lookahead_y=lookahead_y,
            trail_remaining=trail_remaining,
            speed_limited=speed_limited,
            curvature_at_lookahead=curvature_at_lookahead,
        )

    def _find_closest_forward(self, pose: RobotPose, trail: list[TrailPoint]) -> int:
        """Find closest trail point, searching forward from last known index."""
        n = len(trail)
        if self._last_closest_idx >= n:
            self._last_closest_idx = 0

        start = self._last_closest_idx
        end = min(start + self._config.closest_point_search_window, n)

        # If search window doesn't cover enough, extend to end
        if end - start < 2 and n > 2:
            start = max(0, n - self._config.closest_point_search_window)
            end = n

        best_idx = start
        best_dist_sq = float('inf')
        for i in range(start, end):
            dx = trail[i].x - pose.x
            dy = trail[i].y - pose.y
            d2 = dx * dx + dy * dy
            if d2 < best_dist_sq:
                best_dist_sq = d2
                best_idx = i

        self._last_closest_idx = best_idx
        return best_idx

    def _compute_speed(
        self, curvatures: list[float], lookahead_idx: int,
        base_speed: float, now: float,
    ) -> tuple[float, bool, float]:
        """Compute speed with curvature scaling and acceleration limiting."""
        cfg = self._config
        n = len(curvatures)

        # Look ahead from lookahead_idx for pre-deceleration
        look_start = max(0, lookahead_idx)
        look_end = min(n, lookahead_idx + cfg.lookahead_curvature_points)
        if look_start >= look_end:
            return base_speed, False, 0.0

        max_curvature = max(abs(curvatures[i]) for i in range(look_start, look_end))
        curvature_at_lookahead = curvatures[min(lookahead_idx, n - 1)]

        # Scale speed inversely with curvature
        target_speed = base_speed / (1.0 + cfg.curvature_alpha * max_curvature)
        target_speed = max(cfg.min_speed_byte, min(base_speed, target_speed))

        # Acceleration-limited transition
        dt = now - self._last_compute_time if self._last_compute_time > 0 else 0.0
        self._last_compute_time = now
        if dt > 0 and dt < 1.0:
            max_change = cfg.max_accel_byte_per_s * dt
            delta = target_speed - self._last_speed_byte
            delta = max(-max_change, min(max_change, delta))
            actual_speed = self._last_speed_byte + delta
        else:
            actual_speed = target_speed

        actual_speed = max(cfg.min_speed_byte, min(base_speed, actual_speed))
        self._last_speed_byte = actual_speed
        speed_limited = actual_speed < base_speed * 0.95

        return actual_speed, speed_limited, curvature_at_lookahead
