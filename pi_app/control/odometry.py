"""Dead-reckoning odometry for robot pose estimation from IMU heading and motor values."""

from __future__ import annotations

import math
from dataclasses import dataclass

NEUTRAL = 126


@dataclass
class RobotPose:
    x: float
    y: float
    theta: float
    timestamp: float


class DeadReckonOdometry:
    def __init__(self, speed_scale: float = 0.01) -> None:
        self.speed_scale = speed_scale
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)
        self._prev_timestamp: float | None = None

    def update(
        self, heading_deg: float, motor_l: int, motor_r: int, timestamp: float
    ) -> RobotPose:
        theta = math.radians(heading_deg)
        dt = (timestamp - self._prev_timestamp) if self._prev_timestamp is not None else 0.0
        self._prev_timestamp = timestamp

        offset_l = max(0, motor_l - NEUTRAL)
        offset_r = max(0, motor_r - NEUTRAL)
        avg_offset = (offset_l + offset_r) / 2.0
        v = avg_offset * self.speed_scale

        if dt > 0:
            self._pose = RobotPose(
                x=self._pose.x + v * math.cos(theta) * dt,
                y=self._pose.y + v * math.sin(theta) * dt,
                theta=theta,
                timestamp=timestamp,
            )
        else:
            self._pose = RobotPose(
                x=self._pose.x,
                y=self._pose.y,
                theta=theta,
                timestamp=timestamp,
            )

        return self._pose

    def reset(self) -> None:
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)
        self._prev_timestamp = None

    @property
    def pose(self) -> RobotPose:
        return self._pose

    def camera_to_world(self, x_cam: float, z_cam: float) -> tuple[float, float]:
        rx, ry, theta = self._pose.x, self._pose.y, self._pose.theta
        world_x = rx + z_cam * math.cos(theta) - x_cam * math.sin(theta)
        world_y = ry + z_cam * math.sin(theta) + x_cam * math.cos(theta)
        return (world_x, world_y)
