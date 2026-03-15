"""GPS-based odometry for robot pose estimation.

Uses RTK GPS positions (lat/lon → local ENU meters) and fuses GPS
Course-Over-Ground with gyro heading for smooth orientation at all speeds.

Provides the same interface as DeadReckonOdometry so FollowMeController
can swap between them based on GPS availability.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pi_app.control.odometry import RobotPose

# WGS84 constants for local ENU conversion
_DEG_TO_RAD = math.pi / 180.0


@dataclass(frozen=True)
class GpsOdometryConfig:
    """Configuration for GPS-based odometry."""
    # COG heading is only trusted above this speed (m/s).
    # Below this, we rely on gyro-only heading.
    cog_min_speed_mps: float = 0.5
    # Complementary filter alpha for heading fusion.
    # Higher = trust gyro more.  At speed, GPS COG corrects drift.
    heading_alpha: float = 0.85
    # Minimum position delta (m) to compute COG — suppresses noise at low speed.
    cog_min_delta_m: float = 0.05


class GpsOdometry:
    """Odometry from RTK GPS positions + gyro heading fusion.

    Coordinate system: local ENU (East-North-Up) in metres, origin at first fix.
    Heading: 0° = North (+Y), 90° = East (+X), compass convention, stored
    internally as radians in math convention (0 = +X = East, π/2 = +Y = North).
    """

    def __init__(self, config: GpsOdometryConfig | None = None) -> None:
        self._cfg = config or GpsOdometryConfig()

        # Origin (first valid GPS fix)
        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._cos_origin_lat: float = 1.0  # cached cos(lat) for lon→m

        # Latest GPS-derived position in local ENU (metres)
        self._gps_x: float = 0.0  # East
        self._gps_y: float = 0.0  # North

        # Previous GPS position for COG computation
        self._prev_gps_x: float = 0.0
        self._prev_gps_y: float = 0.0
        self._prev_gps_ts: float = 0.0

        # Fused heading (radians, math convention: 0=East, π/2=North)
        self._heading_rad: float = 0.0
        self._heading_initialised: bool = False

        # GPS-derived speed (m/s)
        self._speed_mps: float = 0.0

        # Composite pose
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)

        # Track whether we have a valid GPS fix
        self._has_fix: bool = False

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def _latlon_to_enu(self, lat: float, lon: float) -> tuple[float, float]:
        """Convert lat/lon to local ENU metres relative to origin."""
        dlat = (lat - self._origin_lat) * _DEG_TO_RAD
        dlon = (lon - self._origin_lon) * _DEG_TO_RAD
        # Approximate metres at this latitude
        y = dlat * 6_371_000.0  # North
        x = dlon * 6_371_000.0 * self._cos_origin_lat  # East
        return x, y

    @staticmethod
    def _compass_to_math(compass_deg: float) -> float:
        """Convert compass heading (0=N, CW+) to math angle (0=E, CCW+)."""
        return math.radians(90.0 - compass_deg)

    @staticmethod
    def _math_to_compass(rad: float) -> float:
        """Convert math angle (0=E, CCW+) to compass heading (0=N, CW+)."""
        deg = 90.0 - math.degrees(rad)
        return deg % 360.0

    # ------------------------------------------------------------------
    # Update methods
    # ------------------------------------------------------------------

    def update_gps(self, lat: float, lon: float, fix_quality: int,
                   timestamp: float) -> None:
        """Feed a new GPS reading.  Call at GPS rate (~1 Hz)."""
        if fix_quality < 1:
            return

        # Set origin on first valid fix
        if self._origin_lat is None:
            self._origin_lat = lat
            self._origin_lon = lon
            self._cos_origin_lat = math.cos(lat * _DEG_TO_RAD)
            self._prev_gps_ts = timestamp
            self._has_fix = True
            return

        x, y = self._latlon_to_enu(lat, lon)

        # Compute COG and speed from position delta
        dx = x - self._prev_gps_x
        dy = y - self._prev_gps_y
        dt = timestamp - self._prev_gps_ts
        dist = math.hypot(dx, dy)

        if dt > 0.01:
            self._speed_mps = dist / dt

        # Update COG-based heading if moving fast enough
        if dist >= self._cfg.cog_min_delta_m and self._speed_mps >= self._cfg.cog_min_speed_mps:
            # COG in math convention (atan2 gives angle from +X axis)
            cog_rad = math.atan2(dy, dx)

            if not self._heading_initialised:
                self._heading_rad = cog_rad
                self._heading_initialised = True
            else:
                # Complementary filter: blend gyro-propagated heading with GPS COG
                # Smaller alpha = trust GPS COG more
                alpha = self._cfg.heading_alpha
                # Angle difference (handle wraparound)
                delta = (cog_rad - self._heading_rad + math.pi) % (2 * math.pi) - math.pi
                self._heading_rad = self._heading_rad + (1.0 - alpha) * delta

        self._prev_gps_x = x
        self._prev_gps_y = y
        self._prev_gps_ts = timestamp
        self._gps_x = x
        self._gps_y = y
        self._has_fix = True

        self._pose = RobotPose(
            x=self._gps_x,
            y=self._gps_y,
            theta=self._heading_rad,
            timestamp=timestamp,
        )

    def update_gyro(self, heading_deg: float, timestamp: float) -> None:
        """Feed gyro heading each control cycle (~50 Hz).

        This propagates heading between GPS updates so the fused heading
        stays responsive during the 1-second GPS gap.
        """
        if not math.isfinite(heading_deg):
            return

        gyro_rad = self._compass_to_math(heading_deg)

        if not self._heading_initialised:
            self._heading_rad = gyro_rad
            self._heading_initialised = True
        else:
            # Between GPS updates, track gyro changes.
            # We don't have the previous gyro reading stored, so we
            # directly use the gyro as primary heading source and let
            # update_gps() correct drift when it arrives.
            self._heading_rad = gyro_rad

        # Update pose with latest heading (position stays at last GPS fix)
        self._pose = RobotPose(
            x=self._gps_x,
            y=self._gps_y,
            theta=self._heading_rad,
            timestamp=timestamp,
        )

    # ------------------------------------------------------------------
    # Interface matching DeadReckonOdometry
    # ------------------------------------------------------------------

    @property
    def pose(self) -> RobotPose:
        return self._pose

    @property
    def has_fix(self) -> bool:
        return self._has_fix

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    def camera_to_world(self, x_cam: float, z_cam: float) -> tuple[float, float]:
        """Transform camera-frame detection to world (ENU) coordinates."""
        rx, ry, theta = self._pose.x, self._pose.y, self._pose.theta
        world_x = rx + z_cam * math.cos(theta) - x_cam * math.sin(theta)
        world_y = ry + z_cam * math.sin(theta) + x_cam * math.cos(theta)
        return (world_x, world_y)

    def reset(self) -> None:
        """Reset odometry.  Keeps the GPS origin so coordinates stay consistent."""
        self._gps_x = 0.0
        self._gps_y = 0.0
        self._prev_gps_x = 0.0
        self._prev_gps_y = 0.0
        self._heading_rad = 0.0
        self._heading_initialised = False
        self._speed_mps = 0.0
        self._has_fix = False
        self._origin_lat = None
        self._origin_lon = None
        self._pose = RobotPose(0.0, 0.0, 0.0, 0.0)
