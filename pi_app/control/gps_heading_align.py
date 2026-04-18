"""
GPS course-over-ground heading aligner.

The OAK-D Lite BMI270 has no magnetometer, so integrated-gyro heading is
relative to startup orientation and drifts over time. Whenever the robot is
actually moving (RTK fix, good speed, enough displacement), the GPS track
direction is a near-truth reference for where the robot is *actually*
pointed. This module maintains a single scalar offset so that:

    corrected_heading = (raw_imu_heading + offset) mod 360

is referenced to true north. The aligner is intentionally mode-agnostic —
owned by the controller and updated every tick regardless of whether the
robot is in MANUAL, FOLLOW_ME, or WAYPOINT_NAV.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GpsHeadingAlignConfig:
    enabled: bool = True
    min_distance_m: float = 2.0       # lock offset after this much displacement
    min_speed_mps: float = 0.3        # only trust GPS COG above this speed
    min_fix_quality: int = 4          # require RTK fix
    alpha: float = 0.1                # EMA factor for ongoing drift correction
    history_seconds: float = 4.0      # rolling GPS window


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360.0


def _signed_error_deg(target: float, current: float) -> float:
    """Shortest-arc signed error in (-180, 180]."""
    return ((target - current + 180.0) % 360.0) - 180.0


class GpsHeadingAligner:
    """Estimates and maintains IMU-vs-true-north heading offset from GPS track."""

    def __init__(self, cfg: GpsHeadingAlignConfig) -> None:
        self._cfg = cfg
        self._offset_deg: float = 0.0
        self._locked: bool = False
        # Rolling GPS history: list of (monotonic_t, lat, lon).
        self._history: list[tuple[float, float, float]] = []

    @property
    def offset_deg(self) -> float:
        return self._offset_deg

    @property
    def locked(self) -> bool:
        return self._locked

    def reset(self) -> None:
        """Drop history and unlock. Use on disarm or coordinate-frame change."""
        self._offset_deg = 0.0
        self._locked = False
        self._history.clear()

    def correct(self, raw_imu_heading_deg: float) -> float:
        """Return true-frame heading for a raw-IMU reading."""
        return (raw_imu_heading_deg + self._offset_deg) % 360.0

    def imu_target_heading(self, true_bearing_deg: float) -> float:
        """Return the raw-IMU heading that corresponds to a true-frame bearing.

        Downstream PIDs that consume raw IMU heading should use this as their
        setpoint so they don't need to know about the offset.
        """
        return (true_bearing_deg - self._offset_deg) % 360.0

    def update(
        self,
        lat: float,
        lon: float,
        raw_imu_heading_deg: float,
        fix_quality: int,
        monotonic_t: float,
    ) -> None:
        """Sample one GPS+IMU pair; maybe lock or refine the offset.

        Safe to call every tick — self-gates on fix quality, displacement, and
        speed. Callers don't need to know about the internal window.
        """
        cfg = self._cfg
        if not cfg.enabled:
            return
        if fix_quality < cfg.min_fix_quality:
            # Without a good fix, stale samples would poison the offset.
            # Don't reset what we already learned — just stop updating.
            self._history.clear()
            return

        self._history.append((monotonic_t, lat, lon))
        cutoff = monotonic_t - cfg.history_seconds
        while len(self._history) > 1 and self._history[0][0] < cutoff:
            self._history.pop(0)

        t0, lat0, lon0 = self._history[0]
        dt = monotonic_t - t0
        if dt <= 0.0:
            return
        displacement = _haversine_m(lat0, lon0, lat, lon)
        if displacement < cfg.min_distance_m:
            return
        speed = displacement / dt
        if speed < cfg.min_speed_mps:
            return

        gps_cog = _bearing_deg(lat0, lon0, lat, lon)
        new_offset = _signed_error_deg(gps_cog, raw_imu_heading_deg)
        if not self._locked:
            self._offset_deg = new_offset
            self._locked = True
            return
        # EMA on the shortest-arc difference between offset hypotheses so
        # wraparound is safe even if raw heading is near 0/360.
        delta = _signed_error_deg(
            (raw_imu_heading_deg + new_offset) % 360.0,
            (raw_imu_heading_deg + self._offset_deg) % 360.0,
        )
        updated = self._offset_deg + cfg.alpha * delta
        self._offset_deg = ((updated + 180.0) % 360.0) - 180.0
