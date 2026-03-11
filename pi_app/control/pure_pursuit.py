"""Pure Pursuit path-following controller for differential-drive robots."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .odometry import RobotPose
from .trail import TrailPoint


@dataclass(frozen=True)
class PursuitConfig:
    lookahead_base_m: float = 1.0
    lookahead_speed_scale: float = 0.005
    wheelbase_m: float = 0.28
    max_steer_byte: float = 15.0
    max_speed_byte: float = 115.0


@dataclass(frozen=True)
class PursuitCommand:
    speed_byte: float
    steer_byte: float
    lookahead_x: float
    lookahead_y: float
    trail_remaining: int


class PurePursuitController:
    def __init__(self, config: PursuitConfig) -> None:
        self._config = config

    def compute(
        self, pose: RobotPose, trail: list[TrailPoint], speed_byte: float
    ) -> PursuitCommand | None:
        if len(trail) < 2:
            return None

        lookahead_dist = (
            self._config.lookahead_base_m
            + speed_byte * self._config.lookahead_speed_scale
        )

        best_idx = min(
            range(len(trail)),
            key=lambda i: math.hypot(trail[i].x - pose.x, trail[i].y - pose.y),
        )
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

        kappa = 2.0 * local_y / dist_sq
        steer_byte = kappa * self._config.wheelbase_m * speed_byte / 2.0
        steer_byte = max(
            -self._config.max_steer_byte,
            min(self._config.max_steer_byte, steer_byte),
        )

        return PursuitCommand(
            speed_byte=speed_byte,
            steer_byte=steer_byte,
            lookahead_x=lookahead_x,
            lookahead_y=lookahead_y,
            trail_remaining=trail_remaining,
        )
