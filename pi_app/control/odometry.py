"""Dead-reckoning odometry for robot pose estimation from IMU heading and motor values."""

from __future__ import annotations

import math
from dataclasses import dataclass

NEUTRAL = 126
MAX_DT = 0.2


@dataclass(frozen=True)
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
        if not math.isfinite(heading_deg):
            return self._pose

        theta = math.radians(heading_deg)
        dt = (timestamp - self._prev_timestamp) if self._prev_timestamp is not None else 0.0
        self._prev_timestamp = timestamp

        # Detect counter-rotating motors (spin in place) and zero out speed.
        if (motor_l > NEUTRAL and motor_r < NEUTRAL) or (motor_l < NEUTRAL and motor_r > NEUTRAL):
            v = 0.0
        else:
            offset_l = max(0, motor_l - NEUTRAL)
            offset_r = max(0, motor_r - NEUTRAL)
            avg_offset = (offset_l + offset_r) / 2.0
            v = avg_offset * self.speed_scale

        if 0 < dt <= MAX_DT:
            self._pose = RobotPose(
                x=self._pose.x + v * math.cos(theta) * dt,
                y=self._pose.y + v * math.sin(theta) * dt,
                theta=theta,
                timestamp=timestamp,
            )
        else:
            # dt <= 0 (first call) or dt > MAX_DT (pipeline stall) --
            # update heading only, do not integrate position.
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
