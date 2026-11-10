"""Robot pose for the follow-me trail: the frame contract and dead reckoning.

FRAME CONTRACT (the single definition; every odometry class and every
consumer of a ``RobotPose`` obeys it -- trail placement, pure pursuit, trail
pruning, the blind-search tangent sign and the status/UI fields):

* World frame: ``x`` and ``y`` in metres. ``+x`` is the direction of heading
  0. ``+y`` is 90 degrees CLOCKWISE of ``+x`` seen from above (the direction
  of heading 90). ``GpsOdometry``: ``+x`` = true north, ``+y`` = east.
  ``DeadReckonOdometry``: ``+x`` = the gyro's heading-0 direction (boot
  orientation), ``+y`` = 90 degrees to the right of it.
* ``theta``: radians, CLOCKWISE-positive seen from above, i.e.
  ``theta = radians(heading_deg)`` with ``heading_deg`` as defined by
  ``OakImuReader.read`` (it increases on a RIGHT turn).
* Robot forward = ``forward_unit(theta)`` = ``(cos theta, sin theta)``.
  Robot right = ``right_unit(theta)`` = ``(-sin theta, cos theta)``.
  Camera coordinates: ``x_cam`` metres to the right of the optical axis
  (+ = right), ``z_cam`` metres forward. Steer bytes: + = turn right.

Seen from above with z down this is the north-east-down convention, so the
compass heading, the gyro heading and the world frame share one sign. If a
consumer steers the wrong way, fix the frame at its source (the odometry
class), never with a second inversion downstream.

Only these helpers convert between body and world coordinates. Do not
re-derive ``(cos, sin)`` / ``(-sin, cos)`` at a call site.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

NEUTRAL = 126
MAX_DT = 0.2

# Heading-input guard (shared by both odometry classes).
# A heading step larger than this between two updates is a discontinuity of
# the gyro frame (reader restart / cumulative reset), not a turn: the largest
# step on the 2026-11-06/08 drives was 12 deg in one tick (about 100 deg/s).
HEADING_JUMP_DEG = 30.0


def forward_unit(theta: float) -> tuple[float, float]:
    """World-frame unit vector of the robot's forward direction."""
    return math.cos(theta), math.sin(theta)


def right_unit(theta: float) -> tuple[float, float]:
    """World-frame unit vector to the robot's physical RIGHT.

    It is the forward vector after a further +90 deg of heading; heading
    increases on a right turn, so this is right by construction.
    """
    return -math.sin(theta), math.cos(theta)


def body_to_world(pose: "RobotPose", forward_m: float, right_m: float) -> tuple[float, float]:
    """World position of a point ``forward_m`` ahead and ``right_m`` to the right."""
    fx, fy = forward_unit(pose.theta)
    rx, ry = right_unit(pose.theta)
    return (pose.x + forward_m * fx + right_m * rx,
            pose.y + forward_m * fy + right_m * ry)


def world_to_body(pose: "RobotPose", wx: float, wy: float) -> tuple[float, float]:
    """``(forward_m, right_m)`` of a world point relative to the robot."""
    dx, dy = wx - pose.x, wy - pose.y
    fx, fy = forward_unit(pose.theta)
    rx, ry = right_unit(pose.theta)
    return dx * fx + dy * fy, dx * rx + dy * ry


def wrap_pi(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def is_pivot(motor_l: int, motor_r: int) -> bool:
    """Tracks counter-rotating: a spin in place."""
    return (motor_l > NEUTRAL and motor_r < NEUTRAL) or (motor_l < NEUTRAL and motor_r > NEUTRAL)


def wheel_speed_mps(motor_l: int, motor_r: int, speed_scale: float) -> float:
    """Forward ground speed estimate from the commanded motor bytes.

    Pivot -> 0. Reverse offsets count as 0 (follow-me does not reverse).
    ``speed_scale`` is m/s per byte above neutral. On the 2026-11-06/08 drives
    the scale that best matched the GPS track over 30 s windows was
    0.0100-0.0105 (leave-one-out median of the per-drive fits 0.0104).
    """
    if is_pivot(motor_l, motor_r):
        return 0.0
    offset_l = max(0, motor_l - NEUTRAL)
    offset_r = max(0, motor_r - NEUTRAL)
    return (offset_l + offset_r) / 2.0 * speed_scale


@dataclass(frozen=True)
class RobotPose:
    """Pose in the frame contract above (``theta`` clockwise-positive)."""
    x: float
    y: float
    theta: float
    timestamp: float


class HeadingGuard:
    """Decides whether a gyro heading sample can be trusted for odometry.

    ``update(heading_deg, heading_valid)`` returns one of:
      ``"start"``   first usable sample after construction, ``reset`` or an
                    unusable stretch (a new gyro frame starts: the heading may
                    have moved while it could not be trusted),
      ``"ok"``      usable and continuous with the previous usable sample,
      ``"jump"``    a step > ``HEADING_JUMP_DEG`` (a new gyro frame starts),
      ``"invalid"`` not finite, or the caller says the IMU heading is not
                    live (``heading_valid`` False). On 2026-11-07 the OAK IMU
                    was stale for two whole drives and the heading read a
                    constant 0.000: a finite number that is not a heading.
    """

    def __init__(self, jump_deg: float = HEADING_JUMP_DEG) -> None:
        self._jump_rad = math.radians(float(jump_deg))
        self.reset()

    def reset(self) -> None:
        self._last: float | None = None

    def update(self, heading_deg: float, heading_valid: bool = True) -> str:
        if not heading_valid or not math.isfinite(heading_deg):
            self._last = None
            return "invalid"
        if self._last is None:
            self._last = heading_deg
            return "start"
        step = abs(wrap_pi(math.radians(heading_deg - self._last)))
        self._last = heading_deg
        return "jump" if step > self._jump_rad else "ok"


class DeadReckonOdometry:
    """Gyro heading + wheel-speed dead reckoning in the frame contract above.

    ``+x`` is the gyro's heading-0 direction; the frame is internally
    consistent (camera points and robot motion are rotated by the same
    heading), which is all a 30 s trail needs.

    ``pose_valid`` is False until the first usable heading arrives and while
    the heading is not usable (see ``HeadingGuard``). ``frame_epoch``
    increases each time a new frame starts (first usable heading, a heading
    discontinuity, recovery after an unusable stretch, ``reset``): positions
    from different epochs must not be mixed, so a trail built in an older
    epoch must be discarded.
    """

    def __init__(self, speed_scale: float = 0.01) -> None:
        self.speed_scale = speed_scale
        self._guard = HeadingGuard()
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)
        self._prev_timestamp: float | None = None
        self._valid = False
        self._frame_epoch = 0

    def update(
        self, heading_deg: float, motor_l: int, motor_r: int, timestamp: float,
        heading_valid: bool = True,
    ) -> RobotPose:
        status = self._guard.update(heading_deg, heading_valid)
        if status == "invalid":
            self._valid = False
            self._prev_timestamp = None
            return self._pose
        dt = (timestamp - self._prev_timestamp) if self._prev_timestamp is not None else 0.0
        self._prev_timestamp = timestamp
        theta = math.radians(heading_deg)

        if status in ("start", "jump"):
            # New gyro frame: start a fresh world frame at the origin.
            self._frame_epoch += 1
            self._valid = True
            self._pose = RobotPose(0.0, 0.0, theta, timestamp)
            return self._pose

        v = wheel_speed_mps(motor_l, motor_r, self.speed_scale)
        if 0 < dt <= MAX_DT:
            fx, fy = forward_unit(theta)
            self._pose = RobotPose(
                x=self._pose.x + v * fx * dt,
                y=self._pose.y + v * fy * dt,
                theta=theta,
                timestamp=timestamp,
            )
        else:
            # dt <= 0 or dt > MAX_DT (pipeline stall) --
            # update heading only, do not integrate position.
            self._pose = RobotPose(self._pose.x, self._pose.y, theta, timestamp)
        return self._pose

    def reset(self) -> None:
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)
        self._prev_timestamp = None
        self._guard.reset()
        self._valid = False
        self._frame_epoch += 1

    @property
    def pose(self) -> RobotPose:
        return self._pose

    @property
    def pose_valid(self) -> bool:
        return self._valid

    @property
    def frame_epoch(self) -> int:
        return self._frame_epoch

    def camera_to_world(self, x_cam: float, z_cam: float) -> tuple[float, float]:
        """World position of a camera point (``x_cam`` + = right, ``z_cam`` forward)."""
        return body_to_world(self._pose, forward_m=z_cam, right_m=x_cam)
