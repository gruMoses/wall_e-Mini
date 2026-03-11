"""
Pure-logic waypoint navigation controller.

Computes bearing and distance to the next waypoint (Haversine / forward
azimuth), produces a speed byte using a cruise/approach/stop profile, and
calls ``ImuSteeringCompensator.set_target_heading()`` to steer.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

EARTH_RADIUS_M = 6_371_000.0


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
    speed_byte: int = 0
    fix_quality: int = 0
    gps_stale: bool = False
    completed: bool = False


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


@dataclass
class WaypointNavConfig:
    arrival_radius_m: float = 0.5
    cruise_speed_byte: int = 40
    approach_speed_byte: int = 20
    slow_radius_m: float = 2.0
    min_rtk_quality: int = 4
    stale_timeout_s: float = 3.0


class WaypointNavController:
    """Compute speed and heading commands toward a waypoint sequence."""

    def __init__(self, cfg: WaypointNavConfig, waypoints: list[Waypoint] | None = None) -> None:
        self._cfg = cfg
        self._waypoints = waypoints or []
        self._index = 0
        self._completed = False

    @property
    def waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def completed(self) -> bool:
        return self._completed

    def set_waypoints(self, wps: list[Waypoint]) -> None:
        self._waypoints = list(wps)
        self._index = 0
        self._completed = False

    def compute(
        self,
        lat: float,
        lon: float,
        fix_quality: int,
        gps_age_s: float,
    ) -> tuple[float, int]:
        """Return (target_bearing_deg, speed_byte).

        speed_byte is 0 when GPS quality is insufficient, data is stale,
        or all waypoints have been reached.
        """
        if self._completed or not self._waypoints:
            return 0.0, 0

        if fix_quality < self._cfg.min_rtk_quality:
            return 0.0, 0

        if gps_age_s > self._cfg.stale_timeout_s:
            return 0.0, 0

        wp = self._waypoints[self._index]
        dist = haversine_m(lat, lon, wp.lat, wp.lon)
        brg = bearing_deg(lat, lon, wp.lat, wp.lon)

        if dist <= self._cfg.arrival_radius_m:
            self._index += 1
            if self._index >= len(self._waypoints):
                self._completed = True
                return brg, 0
            wp = self._waypoints[self._index]
            dist = haversine_m(lat, lon, wp.lat, wp.lon)
            brg = bearing_deg(lat, lon, wp.lat, wp.lon)

        speed = self._speed_for_distance(dist)
        return brg, speed

    def _speed_for_distance(self, dist_m: float) -> int:
        if dist_m <= self._cfg.arrival_radius_m:
            return 0
        if self._cfg.slow_radius_m <= self._cfg.arrival_radius_m:
            return self._cfg.cruise_speed_byte
        if dist_m <= self._cfg.slow_radius_m:
            frac = (dist_m - self._cfg.arrival_radius_m) / (
                self._cfg.slow_radius_m - self._cfg.arrival_radius_m
            )
            lo = self._cfg.approach_speed_byte
            hi = self._cfg.cruise_speed_byte
            return max(lo, int(round(lo + (hi - lo) * frac)))
        return self._cfg.cruise_speed_byte

    def get_status(self) -> NavStatus:
        wp = self._waypoints[self._index] if self._index < len(self._waypoints) else None
        return NavStatus(
            active=not self._completed and len(self._waypoints) > 0,
            waypoint_index=self._index,
            waypoint_total=len(self._waypoints),
            waypoint_name=wp.name if wp else "",
            completed=self._completed,
        )
