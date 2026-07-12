"""
GPS course-over-ground heading aligner.

The OAK-D Lite BMI270 has no magnetometer, so integrated-gyro heading is
relative to startup orientation and drifts over time. During an explicitly
gated forward, straight manual-RC run, RTK GPS course-over-ground provides a
one-shot reference for where the robot is pointed. This module freezes a
single scalar offset so that:

    corrected_heading = (raw_imu_heading + offset) mod 360

is referenced to true north for the remainder of the armed session. Other
modes may consume the corrected heading but cannot collect lock history,
establish a lock, or refine the frozen offset.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

from config import GpsHeadingAlignConfig

_logger = logging.getLogger(__name__)

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GpsHeadingAlignStatus:
    """Read-only snapshot for telemetry and field diagnostics."""

    enabled: bool
    locked: bool
    frozen: bool
    refining: bool
    offset_deg: float
    last_cog_deg: Optional[float]
    last_speed_mps: Optional[float]
    last_displacement_m: Optional[float]
    history_samples: int


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
    """One-shot IMU-to-true-north alignment from gated forward GPS motion."""

    def __init__(self, cfg: GpsHeadingAlignConfig) -> None:
        self._cfg = cfg
        self._offset_deg: float = 0.0
        self._locked: bool = False
        # Rolling GPS history: list of (sample_ts, lat, lon).
        # sample_ts is the GPS reading's own monotonic timestamp — not the
        # controller-loop clock — so duplicate polls of the same fix do not
        # inflate apparent speed.
        self._history: list[tuple[float, float, float]] = []
        self._last_sample_ts: Optional[float] = None
        self._last_cog_deg: Optional[float] = None
        self._last_speed_mps: Optional[float] = None
        self._last_displacement_m: Optional[float] = None

    @property
    def offset_deg(self) -> float:
        return self._offset_deg

    @property
    def locked(self) -> bool:
        return self._locked

    @property
    def enabled(self) -> bool:
        return self._cfg.enabled

    @property
    def last_cog_deg(self) -> Optional[float]:
        return self._last_cog_deg

    def status(self) -> GpsHeadingAlignStatus:
        return GpsHeadingAlignStatus(
            enabled=self._cfg.enabled,
            locked=self._locked,
            frozen=self._locked,
            refining=False,
            offset_deg=self._offset_deg,
            last_cog_deg=self._last_cog_deg,
            last_speed_mps=self._last_speed_mps,
            last_displacement_m=self._last_displacement_m,
            history_samples=len(self._history),
        )

    def reset(self) -> None:
        """Drop history and unlock. Use when an armed session ends."""
        self._offset_deg = 0.0
        self._locked = False
        self._history.clear()
        self._last_sample_ts = None
        self._last_cog_deg = None
        self._last_speed_mps = None
        self._last_displacement_m = None

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
        sample_ts: float,
        *,
        lock_allowed: bool = False,
        yaw_rate_dps: Optional[float] = None,
    ) -> None:
        """Sample one GPS+IMU pair; maybe establish a one-shot offset lock.

        ``sample_ts`` must be the GPS reading's own ``GpsReading.timestamp``
        (time.monotonic() when the fix was captured). Calling with the same
        timestamp repeatedly is a no-op so controller-loop duplicates do not
        distort movement speed.

        ``lock_allowed`` must represent an explicit trustworthy straight-run
        condition from the controller. While it is false, or body yaw exceeds
        the configured limit, movement history is discarded so a later lock
        cannot include curved motion. Once locked, the offset is frozen until
        ``reset()``.
        """
        cfg = self._cfg
        if not cfg.enabled:
            return
        # Require exact RTK fixed (quality 4). A >=4 check would also admit
        # RTK float (5), which must not establish heading lock.
        if fix_quality != cfg.min_fix_quality:
            # Without RTK fixed, stale samples would poison the offset.
            # Don't reset what we already learned — just stop updating.
            self._history.clear()
            self._last_sample_ts = None
            return

        if self._locked:
            return

        max_yaw_rate = float(getattr(cfg, "max_lock_yaw_rate_dps", 3.0))
        if (
            not lock_allowed
            or yaw_rate_dps is None
            or abs(yaw_rate_dps) > max_yaw_rate
        ):
            self._history.clear()
            self._last_sample_ts = None
            return

        if self._last_sample_ts is not None:
            if sample_ts == self._last_sample_ts:
                return
            if sample_ts < self._last_sample_ts:
                return

        self._last_sample_ts = sample_ts
        self._history.append((sample_ts, lat, lon))
        cutoff = sample_ts - cfg.history_seconds
        while len(self._history) > 1 and self._history[0][0] < cutoff:
            self._history.pop(0)

        t0, lat0, lon0 = self._history[0]
        dt = sample_ts - t0
        if dt <= 0.0:
            return
        displacement = _haversine_m(lat0, lon0, lat, lon)
        self._last_displacement_m = displacement
        if displacement < cfg.min_distance_m:
            return
        speed = displacement / dt
        self._last_speed_mps = speed
        if speed < cfg.min_speed_mps:
            return

        gps_cog = _bearing_deg(lat0, lon0, lat, lon)
        self._last_cog_deg = gps_cog
        new_offset = _signed_error_deg(gps_cog, raw_imu_heading_deg)
        self._offset_deg = new_offset
        self._locked = True
        _logger.warning(
            "GPS heading aligner LOCKED (frozen): offset=%+.1f° "
            "(gps_cog=%.1f° raw_imu=%.1f° displacement=%.2fm "
            "speed=%.2fm/s yaw_rate=%.1f°/s fix=%d)",
            self._offset_deg,
            gps_cog,
            raw_imu_heading_deg,
            displacement,
            speed,
            yaw_rate_dps,
            fix_quality,
        )
