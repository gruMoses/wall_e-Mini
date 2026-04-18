"""
Pure-logic waypoint navigation controller.

State machine:
- IDLE: no active waypoint sequence -> (v=0, yaw=0)
- ALIGN: heading error too large to drive forward; pivot in place at a fixed
         normalized yaw command until the error drops below align_threshold_deg
- DRIVE: drive forward with PID-based steering; fall back to ALIGN if the
         heading error exceeds recovery_threshold_deg
- ARRIVE: inside arrival_radius_m of the current target -> (v=0, yaw=0) before
          advancing to the next waypoint

compute() returns (v_cmd, yaw_cmd, state) as normalized floats in [-1, 1];
a mixer converts these into left/right motor bytes.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

EARTH_RADIUS_M = 6_371_000.0


class NavState(str, Enum):
    IDLE = "IDLE"
    ALIGN = "ALIGN"
    DRIVE = "DRIVE"
    ARRIVE = "ARRIVE"


@dataclass
class Waypoint:
    lat: float
    lon: float
    name: str = ""


@dataclass
class NavStatus:
    active: bool = False
    waypoint_index: int = 0
    waypoint_total: int = 0
    waypoint_name: str = ""
    bearing_deg: float = 0.0
    distance_m: float = 0.0
    heading_error_deg: float = 0.0
    state: str = NavState.IDLE.value
    v_cmd: float = 0.0
    yaw_cmd: float = 0.0
    fix_quality: int = 0
    gps_stale: bool = False
    completed: bool = False
    heading_offset_deg: float = 0.0
    heading_offset_locked: bool = False
    imu_target_heading_deg: float = 0.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Geodesic distance in metres between two (lat, lon) in degrees."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing (0-360 CW from north) from point 1 to point 2."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360.0


def load_waypoints(path: str | Path) -> list[Waypoint]:
    """Load waypoints from a JSON file (list of {lat, lon, name?})."""
    data = json.loads(Path(path).read_text())
    return [
        Waypoint(lat=wp["lat"], lon=wp["lon"], name=wp.get("name", ""))
        for wp in data
    ]


def _heading_error_deg(target_bearing_deg: float, current_heading_deg: float) -> float:
    """Signed shortest-arc heading error in [-180, 180]."""
    return ((target_bearing_deg - current_heading_deg + 180.0) % 360.0) - 180.0


@dataclass
class WaypointNavConfig:
    arrival_radius_m: float = 1.0
    cruise_speed_byte: int = 40
    approach_speed_byte: int = 20
    slow_radius_m: float = 2.0
    min_rtk_quality: int = 4
    stale_timeout_s: float = 3.0
    align_threshold_deg: float = 12.0
    recovery_threshold_deg: float = 25.0
    pivot_yaw_cmd: float = 0.5
    motor_deadband_byte: int = 12
    gps_align_enabled: bool = True
    gps_align_min_distance_m: float = 2.0
    gps_align_min_speed_mps: float = 0.3
    gps_align_alpha: float = 0.1
    gps_align_history_seconds: float = 4.0


def mix_to_bytes(
    v_cmd: float,
    yaw_cmd: float,
    deadband_byte: int = 12,
    neutral: int = 126,
    half_range: int = 127,
) -> tuple[int, int]:
    """Convert normalized (v_cmd, yaw_cmd) commands into motor bytes.

    v_cmd: -1 (full reverse) to +1 (full forward), 0 = stop.
    yaw_cmd: -1 (full left / CCW) to +1 (full right / CW).

    Skid-steer mix:
      left  = v + yaw   (yaw > 0 turns right: spin left wheel faster)
      right = v - yaw
    Output raw is clamped to [-1, 1] per side, then scaled into the byte range.
    Deadband compensation ensures any non-zero command produces at least
    ``deadband_byte`` of offset from neutral so motors actually move.
    """
    left_raw = max(-1.0, min(1.0, v_cmd + yaw_cmd))
    right_raw = max(-1.0, min(1.0, v_cmd - yaw_cmd))

    left_byte = int(round(neutral + left_raw * half_range))
    right_byte = int(round(neutral + right_raw * half_range))

    if left_raw != 0.0 and abs(left_byte - neutral) < deadband_byte:
        left_byte = neutral + deadband_byte * (1 if left_raw > 0 else -1)
    if right_raw != 0.0 and abs(right_byte - neutral) < deadband_byte:
        right_byte = neutral + deadband_byte * (1 if right_raw > 0 else -1)

    return (
        max(0, min(255, left_byte)),
        max(0, min(255, right_byte)),
    )


class WaypointNavController:
    """State-machine waypoint navigator producing normalized drive commands."""

    def __init__(self, cfg: WaypointNavConfig, waypoints: list[Waypoint] | None = None) -> None:
        self._cfg = cfg
        self._waypoints = waypoints or []
        self._index = 0
        self._completed = False
        self._state: NavState = NavState.IDLE
        self._last_bearing_deg = 0.0
        self._last_distance_m = 0.0
        self._last_heading_error_deg = 0.0
        self._last_v_cmd = 0.0
        self._last_yaw_cmd = 0.0
        # GPS-derived heading offset: corrected_heading = (raw_imu + offset) mod 360.
        # Populated only while in DRIVE, after enough displacement to trust GPS COG.
        self._heading_offset_deg: float = 0.0
        self._heading_offset_locked: bool = False
        self._gps_history: list[tuple[float, float, float]] = []  # (mono_t, lat, lon)

    @property
    def waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def completed(self) -> bool:
        return self._completed

    @property
    def state(self) -> NavState:
        return self._state

    def set_waypoints(self, wps: list[Waypoint]) -> None:
        self._waypoints = list(wps)
        self._index = 0
        self._completed = False
        self._state = NavState.IDLE

    def compute(
        self,
        lat: float,
        lon: float,
        fix_quality: int,
        gps_age_s: float,
        current_heading_deg: Optional[float] = None,
    ) -> tuple[float, float, NavState]:
        """Compute (v_cmd, yaw_cmd, state).

        v_cmd and yaw_cmd are normalized floats in [-1, 1]. Returns zeros in
        IDLE/ARRIVE or whenever GPS is unusable.
        """
        cfg = self._cfg

        if self._completed or not self._waypoints:
            return self._emit(NavState.IDLE, 0.0, 0.0)

        if fix_quality < cfg.min_rtk_quality or gps_age_s > cfg.stale_timeout_s:
            return self._emit(self._state, 0.0, 0.0)

        wp = self._waypoints[self._index]
        dist = haversine_m(lat, lon, wp.lat, wp.lon)
        brg = bearing_deg(lat, lon, wp.lat, wp.lon)
        self._last_bearing_deg = brg
        self._last_distance_m = dist

        if dist <= cfg.arrival_radius_m:
            self._index += 1
            if self._index >= len(self._waypoints):
                self._completed = True
                return self._emit(NavState.ARRIVE, 0.0, 0.0)
            wp = self._waypoints[self._index]
            dist = haversine_m(lat, lon, wp.lat, wp.lon)
            brg = bearing_deg(lat, lon, wp.lat, wp.lon)
            self._last_bearing_deg = brg
            self._last_distance_m = dist
            self._state = NavState.ALIGN

        if current_heading_deg is None:
            self._last_heading_error_deg = 0.0
            return self._emit(self._state if self._state != NavState.IDLE else NavState.ALIGN, 0.0, 0.0)

        corrected_heading = (current_heading_deg + self._heading_offset_deg) % 360.0
        err_signed = _heading_error_deg(brg, corrected_heading)
        err_abs = abs(err_signed)
        self._last_heading_error_deg = err_signed

        if self._state in (NavState.IDLE, NavState.ARRIVE):
            self._state = NavState.ALIGN

        if self._state == NavState.ALIGN:
            # Spinning in place -> GPS track is noise; reset history so drift
            # sampling starts fresh when DRIVE resumes.
            self._gps_history.clear()
            if err_abs <= cfg.align_threshold_deg:
                self._state = NavState.DRIVE
            else:
                yaw = -cfg.pivot_yaw_cmd if err_signed > 0 else cfg.pivot_yaw_cmd
                return self._emit(NavState.ALIGN, 0.0, yaw)

        # DRIVE state
        if err_abs > cfg.recovery_threshold_deg:
            self._state = NavState.ALIGN
            self._gps_history.clear()
            yaw = -cfg.pivot_yaw_cmd if err_signed > 0 else cfg.pivot_yaw_cmd
            return self._emit(NavState.ALIGN, 0.0, yaw)

        if cfg.gps_align_enabled:
            self._update_heading_offset(lat, lon, current_heading_deg)

        v_cmd = self._forward_v_for_distance(dist)
        # yaw_cmd for DRIVE is left to the imu compensator (returned as 0 here).
        return self._emit(NavState.DRIVE, v_cmd, 0.0)

    def _update_heading_offset(
        self, lat: float, lon: float, raw_imu_heading_deg: float
    ) -> None:
        """Sample GPS track vs raw IMU heading; lock/refine heading offset.

        Pushes current (t, lat, lon) onto a rolling buffer, discards samples
        older than ``gps_align_history_seconds``, and whenever the oldest
        surviving sample is far enough away and we've been moving fast enough,
        derives a GPS course-over-ground and updates ``self._heading_offset_deg``.
        First valid sample locks the offset; subsequent samples apply an EMA.
        """
        cfg = self._cfg
        now = time.monotonic()
        self._gps_history.append((now, lat, lon))
        cutoff = now - cfg.gps_align_history_seconds
        # Drop entries that fell out of the window, keeping at least one.
        while len(self._gps_history) > 1 and self._gps_history[0][0] < cutoff:
            self._gps_history.pop(0)

        t0, lat0, lon0 = self._gps_history[0]
        dt = now - t0
        if dt <= 0.0:
            return
        displacement = haversine_m(lat0, lon0, lat, lon)
        if displacement < cfg.gps_align_min_distance_m:
            return
        speed = displacement / dt
        if speed < cfg.gps_align_min_speed_mps:
            return

        gps_cog = bearing_deg(lat0, lon0, lat, lon)
        new_offset = _heading_error_deg(gps_cog, raw_imu_heading_deg)
        if not self._heading_offset_locked:
            self._heading_offset_deg = new_offset
            self._heading_offset_locked = True
        else:
            # EMA on the shortest-arc difference so wraparound is safe.
            delta = _heading_error_deg(
                (raw_imu_heading_deg + new_offset) % 360.0,
                (raw_imu_heading_deg + self._heading_offset_deg) % 360.0,
            )
            self._heading_offset_deg = (self._heading_offset_deg + cfg.gps_align_alpha * delta) % 360.0
            if self._heading_offset_deg > 180.0:
                self._heading_offset_deg -= 360.0

    def _emit(self, state: NavState, v_cmd: float, yaw_cmd: float) -> tuple[float, float, NavState]:
        self._state = state
        self._last_v_cmd = v_cmd
        self._last_yaw_cmd = yaw_cmd
        return v_cmd, yaw_cmd, state

    def _forward_v_for_distance(self, dist_m: float) -> float:
        """Distance-based forward speed profile, returned as a normalized float.

        Byte speeds from config are scaled by 127 to get normalized magnitudes.
        """
        cfg = self._cfg
        if dist_m <= cfg.arrival_radius_m:
            return 0.0
        hi = cfg.cruise_speed_byte / 127.0
        lo = cfg.approach_speed_byte / 127.0
        if cfg.slow_radius_m <= cfg.arrival_radius_m:
            return hi
        if dist_m <= cfg.slow_radius_m:
            frac = (dist_m - cfg.arrival_radius_m) / (
                cfg.slow_radius_m - cfg.arrival_radius_m
            )
            return max(lo, lo + (hi - lo) * frac)
        return hi

    @property
    def heading_offset_deg(self) -> float:
        return self._heading_offset_deg

    @property
    def heading_offset_locked(self) -> bool:
        return self._heading_offset_locked

    def imu_target_heading_deg(self, bearing_deg_val: float) -> float:
        """Target heading in raw-IMU frame for the downstream PID.

        PID consumes raw IMU heading; we want the robot's real heading to match
        ``bearing_deg_val``, so subtract the learned offset.
        """
        return (bearing_deg_val - self._heading_offset_deg) % 360.0

    def get_status(self) -> NavStatus:
        wp = self._waypoints[self._index] if self._index < len(self._waypoints) else None
        return NavStatus(
            active=not self._completed and len(self._waypoints) > 0,
            waypoint_index=self._index,
            waypoint_total=len(self._waypoints),
            waypoint_name=wp.name if wp else "",
            bearing_deg=self._last_bearing_deg,
            distance_m=self._last_distance_m,
            heading_error_deg=self._last_heading_error_deg,
            state=self._state.value,
            v_cmd=self._last_v_cmd,
            yaw_cmd=self._last_yaw_cmd,
            completed=self._completed,
            heading_offset_deg=self._heading_offset_deg,
            heading_offset_locked=self._heading_offset_locked,
            imu_target_heading_deg=self.imu_target_heading_deg(self._last_bearing_deg),
        )
