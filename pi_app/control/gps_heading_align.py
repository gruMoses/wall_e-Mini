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
    # Per-epoch course-over-ground path (2026-09-19).
    cog_samples: int = 0
    cog_spread_deg: Optional[float] = None
    lock_source: Optional[str] = None   # "displacement" | "cog" | None


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


def _circular_mean_deg(values_deg: list[float]) -> float:
    """Circular mean in (-180, 180]."""
    sx = sum(math.cos(math.radians(v)) for v in values_deg)
    sy = sum(math.sin(math.radians(v)) for v in values_deg)
    return _signed_error_deg(math.degrees(math.atan2(sy, sx)), 0.0)


def _circular_spread_deg(values_deg: list[float]) -> float:
    """Circular standard deviation (degrees) from the mean resultant length."""
    n = len(values_deg)
    if n == 0:
        return float("inf")
    sx = sum(math.cos(math.radians(v)) for v in values_deg) / n
    sy = sum(math.sin(math.radians(v)) for v in values_deg) / n
    r = math.hypot(sx, sy)
    if r <= 1e-12:
        return float("inf")
    if r >= 1.0:
        return 0.0
    return math.degrees(math.sqrt(-2.0 * math.log(r)))


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
        # Per-epoch course-over-ground lock path: (sample_ts, offset_deg).
        self._cog_samples: list[tuple[float, float]] = []
        self._last_cog_sample_ts: Optional[float] = None
        self._cog_spread_deg: Optional[float] = None
        self._lock_source: Optional[str] = None

    @property
    def lock_source(self) -> Optional[str]:
        return self._lock_source

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
            cog_samples=len(self._cog_samples),
            cog_spread_deg=self._cog_spread_deg,
            lock_source=self._lock_source,
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
        self._cog_samples.clear()
        self._last_cog_sample_ts = None
        self._cog_spread_deg = None
        self._lock_source = None

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
        (time.monotonic() when the fix was captured). Duplicate timestamps are
        ignored before lock/yaw gates so controller-loop polls of the same fix
        do not clear history or distort movement speed. Out-of-order timestamps
        clear history (fail-closed).

        ``lock_allowed`` must represent an explicit trustworthy straight-run
        condition from the controller. While it is false on a **new** GPS
        sample, or body yaw exceeds the configured limit, movement history is
        discarded so a later lock cannot include curved motion. Once locked, the
        offset is frozen until ``reset()``.
        """
        cfg = self._cfg
        if not cfg.enabled:
            return

        if self._locked:
            return

        # Deduplicate before lock/yaw gates so controller-loop polls of the
        # same GPS fix (~30 Hz vs ~1 Hz GPS) do not re-evaluate straightness
        # or clear history on transient IMU yaw spikes.
        if self._last_sample_ts is not None:
            if sample_ts == self._last_sample_ts:
                return
            if sample_ts < self._last_sample_ts:
                self._history.clear()
                self._last_sample_ts = None
                return

        # Require exact RTK fixed (quality 4). A >=4 check would also admit
        # RTK float (5), which must not establish heading lock.
        if fix_quality != cfg.min_fix_quality:
            # Without RTK fixed, stale samples would poison the offset.
            # Clear the unlocked candidate; the early return above protects
            # a frozen offset once lock has been established.
            self._history.clear()
            self._last_sample_ts = sample_ts
            return

        max_yaw_rate = float(getattr(cfg, "max_lock_yaw_rate_dps", 3.0))
        if (
            not lock_allowed
            or yaw_rate_dps is None
            or abs(yaw_rate_dps) > max_yaw_rate
        ):
            self._history.clear()
            self._last_sample_ts = sample_ts
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
        self._lock_source = "displacement"
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

    def update_cog(
        self,
        raw_imu_heading_deg: float,
        cog_deg: Optional[float],
        sog_mps: Optional[float],
        fix_quality: int,
        sample_ts: float,
        *,
        forward_intent: bool,
        yaw_rate_dps: Optional[float] = None,
    ) -> None:
        """Per-epoch course-over-ground lock (2026-09-19).

        Each GPS epoch that qualifies contributes one offset sample,
        ``course_over_ground - raw_imu_heading``, taken at the same instant.
        Because every sample is paired, the path does not need to be straight
        and the command does not need to be perfectly equal on both tracks:
        a wobble moves the course and the heading together. The lock happens
        when ``cog_min_samples`` samples inside ``cog_window_s`` agree to
        within ``cog_max_spread_deg`` (circular standard deviation); their
        circular mean becomes the frozen offset.

        Gates, all fail-closed: the offset is frozen once locked; RTK fixed
        only (a fix loss clears the candidate samples); ``forward_intent``
        (both tracks commanded forward — a reverse run would give a course
        180 deg from the heading); ``sog_mps >= cog_min_speed_mps`` (Doppler
        course is noise when slow); ``|yaw_rate| <= cog_max_yaw_rate_dps``
        (GPS latency during a pivot skews the pair). Non-qualifying epochs are
        skipped, not fatal: each accepted sample was taken under qualifying
        conditions on its own.
        """
        cfg = self._cfg
        if not cfg.enabled or not bool(getattr(cfg, "cog_lock_enabled", True)):
            return
        if self._locked:
            return
        if self._last_cog_sample_ts is not None:
            if sample_ts == self._last_cog_sample_ts:
                return
            if sample_ts < self._last_cog_sample_ts:
                self._cog_samples.clear()
                self._last_cog_sample_ts = None
                return
        self._last_cog_sample_ts = sample_ts

        if fix_quality != cfg.min_fix_quality:
            self._cog_samples.clear()
            self._cog_spread_deg = None
            return

        window_s = float(getattr(cfg, "cog_window_s", 20.0))
        cutoff = sample_ts - window_s
        self._cog_samples = [(t, v) for (t, v) in self._cog_samples if t >= cutoff]

        min_speed = float(getattr(cfg, "cog_min_speed_mps", 0.3))
        max_yaw = float(getattr(cfg, "cog_max_yaw_rate_dps", 6.0))
        if (
            not forward_intent
            or cog_deg is None
            or sog_mps is None
            or float(sog_mps) < min_speed
            or yaw_rate_dps is None
            or abs(float(yaw_rate_dps)) > max_yaw
        ):
            self._cog_spread_deg = (
                _circular_spread_deg([v for (_, v) in self._cog_samples])
                if len(self._cog_samples) >= 2 else None
            )
            return

        self._last_cog_deg = float(cog_deg)
        self._last_speed_mps = float(sog_mps)
        self._cog_samples.append(
            (sample_ts, _signed_error_deg(float(cog_deg), raw_imu_heading_deg))
        )
        values = [v for (_, v) in self._cog_samples]
        spread = _circular_spread_deg(values) if len(values) >= 2 else None
        self._cog_spread_deg = spread

        min_samples = int(getattr(cfg, "cog_min_samples", 6))
        max_spread = float(getattr(cfg, "cog_max_spread_deg", 8.0))
        if len(values) < min_samples or spread is None or spread > max_spread:
            return

        self._offset_deg = _circular_mean_deg(values)
        self._locked = True
        self._lock_source = "cog"
        _logger.warning(
            "GPS heading aligner LOCKED (frozen) from %d course-over-ground "
            "epochs: offset=%+.1f° spread=%.1f° (last cog=%.1f° raw_imu=%.1f° "
            "sog=%.2fm/s yaw_rate=%.1f°/s fix=%d)",
            len(values), self._offset_deg, spread, float(cog_deg),
            raw_imu_heading_deg, float(sog_mps), float(yaw_rate_dps), fix_quality,
        )
