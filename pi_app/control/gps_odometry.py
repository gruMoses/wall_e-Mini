"""GPS-anchored odometry for the follow-me trail, in the shared frame contract.

Frame (see ``pi_app/control/odometry.py`` -- the one definition):
``+x`` = true north, ``+y`` = east, metres from the first fix; ``theta`` is
the TRUE compass heading in radians, clockwise-positive. Forward is
``(cos theta, sin theta)`` = (north, east) components, right is
``(-sin theta, cos theta)``. The same helpers as ``DeadReckonOdometry``
convert camera and trail points, so pure pursuit and the search-tangent sign
need no per-source code.

Heading: ``theta = gyro heading + offset``. The gyro heading is relative to
its boot orientation (59-78 deg off true north on the 2026-11-06/08 drives),
so the offset is learned from the GPS course over ground. A course sample is
taken only between two consecutive fixes of the same quality, when every
motion update in between drove both tracks forward (never reversing, never
pivoting), the gyro turned less than ``cog_max_turn_deg``, and the robot
covered at least ``cog_min_delta_m`` at ``cog_min_speed_mps`` or more. The
first ``cog_init_samples`` samples must agree within
``cog_init_spread_deg``; after that each sample moves the offset by
``1 - heading_alpha`` of its error. The gyro supplies every heading CHANGE;
the course only corrects the offset (the old code overwrote the fused heading
with the raw gyro on every tick, so the course blend lasted one tick).

Position: the last fix plus wheel-speed dead reckoning since that fix (GPS
arrives at 1 Hz). ``pose_valid`` is False until there is a fix AND the
offset is known AND the gyro heading is usable. ``frame_epoch`` increases
when the offset is first learned, when the fix quality changes (a change
moved the parked position 1.3-2.1 m on 2026-11-08), when the gyro frame
restarts, and on ``reset``: a trail from an older epoch must be discarded.

Not selected by default: ``FollowMeConfig.trail_odometry_source`` is
``"dead_reckon"`` (see the comment there for the numbers).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pi_app.control.odometry import (
    MAX_DT,
    NEUTRAL,
    HeadingGuard,
    RobotPose,
    body_to_world,
    forward_unit,
    is_pivot,
    wheel_speed_mps,
    wrap_pi,
)

_DEG_TO_RAD = math.pi / 180.0
_EARTH_R_M = 6_371_000.0


@dataclass(frozen=True)
class GpsOdometryConfig:
    """Configuration for GPS-anchored odometry."""
    # A course sample needs at least this GPS ground speed (m/s) ...
    cog_min_speed_mps: float = 0.5
    # ... and at least this distance between the two fixes (m).
    cog_min_delta_m: float = 0.05
    # After the offset is known: weight kept on the old offset per sample.
    heading_alpha: float = 0.85
    # Largest gyro heading change across the fix interval for a course sample.
    cog_max_turn_deg: float = 4.0
    # Samples needed before the offset (and so the pose) is offered ...
    cog_init_samples: int = 3
    # ... and how closely they must agree (max deviation from their mean).
    cog_init_spread_deg: float = 15.0
    # Wheel speed between fixes (same model as DeadReckonOdometry).
    speed_scale_mps_per_byte: float = 0.01


class GpsOdometry:
    """GPS position + gyro heading with a course-over-ground offset."""

    def __init__(self, config: GpsOdometryConfig | None = None) -> None:
        self._cfg = config or GpsOdometryConfig()
        self._guard = HeadingGuard()
        self._frame_epoch = 0
        self._init_state()

    def _init_state(self) -> None:
        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._cos_origin_lat = 1.0
        # Last fix (north, east), its quality and timestamp.
        self._fix_n = 0.0
        self._fix_e = 0.0
        self._fix_quality: int | None = None
        self._fix_ts: float | None = None
        self._has_fix = False
        self._speed_mps = 0.0
        # Wheel dead reckoning since the last fix, true frame (north, east).
        self._dr_n = 0.0
        self._dr_e = 0.0
        # Gyro state.
        self._gyro_rad: float | None = None
        self._gyro_usable = False
        self._prev_motion_ts: float | None = None
        # Gyro-to-true offset.
        self._offset_rad = 0.0
        self._offset_known = False
        self._init_samples: list[float] = []
        self._reset_interval()
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)

    def _reset_interval(self) -> None:
        """Start collecting the motion between this fix and the next."""
        self._iv_ok = self._gyro_usable
        self._iv_n = 0
        self._iv_sin = 0.0
        self._iv_cos = 0.0
        self._iv_turn = 0.0
        self._iv_max_turn = 0.0

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def _latlon_to_ne(self, lat: float, lon: float) -> tuple[float, float]:
        """Lat/lon to (north, east) metres from the origin."""
        north = (lat - self._origin_lat) * _DEG_TO_RAD * _EARTH_R_M
        east = (lon - self._origin_lon) * _DEG_TO_RAD * _EARTH_R_M * self._cos_origin_lat
        return north, east

    # ------------------------------------------------------------------
    # Update methods
    # ------------------------------------------------------------------

    def update_motion(
        self,
        heading_deg: float,
        motor_l: int,
        motor_r: int,
        timestamp: float,
        heading_valid: bool = True,
    ) -> None:
        """Feed the gyro heading (``OakImuReader`` convention, CW-positive,
        boot-relative) and the emitted motor bytes every control tick."""
        status = self._guard.update(heading_deg, heading_valid)
        if status == "invalid":
            self._gyro_usable = False
            self._iv_ok = False
            self._prev_motion_ts = None
            self._refresh_pose(timestamp)
            return
        g = math.radians(heading_deg)
        if status in ("start", "jump"):
            # A new gyro frame: the learned offset no longer applies.
            if self._offset_known:
                self._frame_epoch += 1
            self._offset_known = False
            self._init_samples = []
            self._iv_ok = False
            self._dr_n = self._dr_e = 0.0
            self._gyro_rad = g
            self._gyro_usable = True
            self._prev_motion_ts = timestamp
            self._refresh_pose(timestamp)
            return

        step = wrap_pi(g - self._gyro_rad) if self._gyro_rad is not None else 0.0
        self._gyro_rad = g
        self._gyro_usable = True
        dt = (timestamp - self._prev_motion_ts) if self._prev_motion_ts is not None else 0.0
        self._prev_motion_ts = timestamp

        # Course-sample qualification for the current fix interval.
        forward = motor_l > NEUTRAL and motor_r > NEUTRAL and not is_pivot(motor_l, motor_r)
        if not forward:
            self._iv_ok = False
        self._iv_turn += step
        self._iv_max_turn = max(self._iv_max_turn, abs(self._iv_turn))
        self._iv_sin += math.sin(g)
        self._iv_cos += math.cos(g)
        self._iv_n += 1

        if self._offset_known and 0.0 < dt <= MAX_DT:
            v = wheel_speed_mps(motor_l, motor_r, self._cfg.speed_scale_mps_per_byte)
            fn, fe = forward_unit(g + self._offset_rad)
            self._dr_n += v * fn * dt
            self._dr_e += v * fe * dt
        self._refresh_pose(timestamp)

    def update_gyro(self, heading_deg: float, timestamp: float) -> None:
        """Heading-only feed (no motion): kept for callers without motor bytes."""
        self.update_motion(heading_deg, NEUTRAL, NEUTRAL, timestamp)

    def update_gps(self, lat: float, lon: float, fix_quality: int,
                   timestamp: float) -> None:
        """Feed a GPS reading. The same reading may be fed every tick; only a
        new ``timestamp`` is processed."""
        if fix_quality < 1:
            return
        if self._fix_ts is not None and timestamp == self._fix_ts:
            return
        if self._origin_lat is None:
            self._origin_lat = lat
            self._origin_lon = lon
            self._cos_origin_lat = math.cos(lat * _DEG_TO_RAD)
        n, e = self._latlon_to_ne(lat, lon)

        if self._fix_ts is not None:
            dn, de = n - self._fix_n, e - self._fix_e
            dt = timestamp - self._fix_ts
            dist = math.hypot(dn, de)
            if dt > 0.01:
                self._speed_mps = dist / dt
            if fix_quality != self._fix_quality:
                # Datum jump: positions before and after do not agree.
                self._frame_epoch += 1
            elif (self._iv_ok and self._gyro_usable and self._iv_n >= 2
                  and self._iv_max_turn <= math.radians(self._cfg.cog_max_turn_deg)
                  and dist >= self._cfg.cog_min_delta_m
                  and self._speed_mps >= self._cfg.cog_min_speed_mps):
                course = math.atan2(de, dn)
                gyro_mean = math.atan2(self._iv_sin, self._iv_cos)
                self._add_offset_sample(wrap_pi(course - gyro_mean))

        self._fix_n, self._fix_e = n, e
        self._fix_quality = fix_quality
        self._fix_ts = timestamp
        self._has_fix = True
        self._dr_n = self._dr_e = 0.0
        self._reset_interval()
        self._refresh_pose(timestamp)

    def _add_offset_sample(self, meas: float) -> None:
        if self._offset_known:
            self._offset_rad = wrap_pi(
                self._offset_rad + (1.0 - self._cfg.heading_alpha) * wrap_pi(meas - self._offset_rad)
            )
            return
        self._init_samples.append(meas)
        need = max(1, int(self._cfg.cog_init_samples))
        if len(self._init_samples) > need:
            self._init_samples = self._init_samples[-need:]
        if len(self._init_samples) < need:
            return
        mean = math.atan2(sum(math.sin(a) for a in self._init_samples),
                          sum(math.cos(a) for a in self._init_samples))
        spread = max(abs(wrap_pi(a - mean)) for a in self._init_samples)
        if spread <= math.radians(self._cfg.cog_init_spread_deg):
            self._offset_rad = mean
            self._offset_known = True
            self._init_samples = []
            self._frame_epoch += 1

    def _refresh_pose(self, timestamp: float) -> None:
        theta = wrap_pi((self._gyro_rad or 0.0) + self._offset_rad)
        self._pose = RobotPose(
            x=self._fix_n + self._dr_n,
            y=self._fix_e + self._dr_e,
            theta=theta,
            timestamp=timestamp,
        )

    # ------------------------------------------------------------------
    # Interface shared with DeadReckonOdometry
    # ------------------------------------------------------------------

    @property
    def pose(self) -> RobotPose:
        return self._pose

    @property
    def pose_valid(self) -> bool:
        return self._has_fix and self._offset_known and self._gyro_usable

    @property
    def frame_epoch(self) -> int:
        return self._frame_epoch

    @property
    def heading_reference_known(self) -> bool:
        return self._offset_known

    @property
    def heading_offset_deg(self) -> float:
        """True heading minus gyro heading (deg); meaningful once known."""
        return math.degrees(self._offset_rad)

    @property
    def has_fix(self) -> bool:
        return self._has_fix

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    def camera_to_world(self, x_cam: float, z_cam: float) -> tuple[float, float]:
        """World (north, east) of a camera point (``x_cam`` + = right, ``z_cam`` forward)."""
        return body_to_world(self._pose, forward_m=z_cam, right_m=x_cam)

    def reset(self) -> None:
        """Forget everything (origin, fix, offset). Starts a new frame epoch."""
        self._guard.reset()
        self._init_state()
        self._frame_epoch += 1
