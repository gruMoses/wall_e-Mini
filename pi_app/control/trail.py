"""Breadcrumb trail manager for robot path tracking and recovery."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


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


class TrailManager:
    def __init__(self, config: TrailConfig) -> None:
        self._config = config
        self._trail: deque[TrailPoint] = deque(maxlen=config.max_trail_points)

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
