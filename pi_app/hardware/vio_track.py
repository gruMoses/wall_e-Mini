"""Log-only bookkeeping for the BasaltVIO pose stream (no control authority).

BasaltVIO reports a pose in its own world frame: Z up, yaw counter-clockwise
positive, origin at the pose where the session started. WALL-E's heading is
compass-style (clockwise positive, see OakImuReader.read). This module turns
the pose stream into numbers that can be logged beside the gyro heading:

  * ``heading_deg``: the VIO heading change since the session started,
    clockwise positive and unwrapped, so it compares directly with a change
    of ``heading_deg`` from the IMU;
  * position (x, y, z) in metres and the path length;
  * roll and pitch, as a frame check: a VIO fed the wrong IMU axes puts its
    world upside down (roll near 180 deg, measured 2026-11-06), so a large
    roll or pitch at the start marks the session's VIO as not trustworthy.

Nothing here reaches the motors or the controller.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

# |roll| or |pitch| above this means the VIO world frame is wrong (the robot
# drives on the ground; the camera tilt is a few degrees).
FRAME_TILT_LIMIT_DEG = 30.0
_RATE_WINDOW_S = 2.0


def quaternion_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """(roll, pitch, yaw) in degrees, yaw about world Z, counter-clockwise positive."""
    roll = math.degrees(math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.degrees(math.asin(s))
    yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    return roll, pitch, yaw


def _wrap_step_deg(prev: float, new: float) -> float:
    """Shortest signed step from ``prev`` to ``new`` in degrees."""
    return (new - prev + 180.0) % 360.0 - 180.0


class VioTracker:
    """Latest VIO pose and the derived log fields for one camera session."""

    def __init__(self) -> None:
        self.session = 0
        self._reset_state()

    def _reset_state(self) -> None:
        self.poses = 0
        self.last_ts: Optional[float] = None
        self.x = self.y = self.z = None
        self.roll_deg = self.pitch_deg = None
        self._yaw0: Optional[float] = None
        self._yaw_prev: Optional[float] = None
        self._yaw_unwrapped = 0.0
        self.path_m = 0.0
        self._last_xyz: Optional[tuple] = None
        self._times: deque = deque()
        self.frame_ok: Optional[bool] = None

    def new_session(self) -> None:
        """Start over: a new camera session restarts the VIO at the origin."""
        self.session += 1
        self._reset_state()

    def update(self, x: float, y: float, z: float,
               qx: float, qy: float, qz: float, qw: float, now_s: float) -> None:
        vals = (x, y, z, qx, qy, qz, qw)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            return
        roll, pitch, yaw = quaternion_to_rpy_deg(qx, qy, qz, qw)
        if self._yaw0 is None:
            self._yaw0 = yaw
            self._yaw_prev = yaw
            self._yaw_unwrapped = yaw
        else:
            self._yaw_unwrapped += _wrap_step_deg(self._yaw_prev, yaw)
            self._yaw_prev = yaw
        xyz = (float(x), float(y), float(z))
        if self._last_xyz is not None:
            self.path_m += math.dist(xyz, self._last_xyz)
        self._last_xyz = xyz
        self.x, self.y, self.z = xyz
        self.roll_deg, self.pitch_deg = roll, pitch
        tilted = abs(roll) > FRAME_TILT_LIMIT_DEG or abs(pitch) > FRAME_TILT_LIMIT_DEG
        if self.frame_ok is None:
            self.frame_ok = not tilted
        elif tilted:
            # Once wrong, the session stays marked: a VIO that lost its frame
            # does not earn trust back by passing through level.
            self.frame_ok = False
        self.poses += 1
        self.last_ts = now_s
        self._times.append(now_s)
        while self._times and now_s - self._times[0] > _RATE_WINDOW_S:
            self._times.popleft()

    @property
    def heading_deg(self) -> Optional[float]:
        """Heading change since the session start, clockwise positive."""
        if self._yaw0 is None:
            return None
        return -(self._yaw_unwrapped - self._yaw0)

    def rate_hz(self, now_s: float) -> float:
        while self._times and now_s - self._times[0] > _RATE_WINDOW_S:
            self._times.popleft()
        if len(self._times) < 2:
            return 0.0
        return len(self._times) / _RATE_WINDOW_S

    def snapshot(self, now_s: float) -> dict:
        def r(v, n):
            return round(v, n) if isinstance(v, float) else v
        age = (now_s - self.last_ts) if self.last_ts is not None else None
        return {
            "vio_session": self.session,
            "vio_poses": self.poses,
            "vio_age_s": r(age, 3),
            "vio_rate_hz": round(self.rate_hz(now_s), 1),
            "vio_heading_deg": r(self.heading_deg, 2),
            "vio_x_m": r(self.x, 3),
            "vio_y_m": r(self.y, 3),
            "vio_z_m": r(self.z, 3),
            "vio_path_m": round(self.path_m, 3),
            "vio_roll_deg": r(self.roll_deg, 1),
            "vio_pitch_deg": r(self.pitch_deg, 1),
            "vio_frame_ok": self.frame_ok,
        }
