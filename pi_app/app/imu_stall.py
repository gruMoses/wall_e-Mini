"""IMU stall events for the JSON run log (pure logic, no hardware).

Incident 2026-11-06 19:34:37: the OAK IMU stream stopped with depth and
detections live. The journal printed one warning and the JSON log showed
nothing but ``integrate_status`` "stale" on every tick for 38.8 h, so the
two drives on 2026-11-07 ran with a frozen heading and no event to find.

``ImuStallMonitor`` turns the reader's health fields (``imu_stale``,
``imu_age_s``, ``imu_stall_restarts``, ``imu_stall_fault``; see
``OakDepthReader.get_health``) into JSON log events, the same shape as the
``oak_vision_hung`` events (``pi_app/app/vision_hang.py``):

* ``oak_imu_stall`` -- the IMU has been stale for ``event_s``. Shorter drops
  recover by themselves and are not events (2026-10-22..24: one every
  ~4.4 s for 40 h).
* ``oak_imu_stall_restart`` -- the reader's IMU watchdog ended the camera
  session (its restart counter went up).
* ``oak_imu_stall_fault`` -- the shared restart budget was full and the
  watchdog latched its fault.
* ``oak_imu_recovered`` -- fresh IMU samples after an ``oak_imu_stall``.

The caller adds ``"type": "event"`` and ``"ts"``.
"""

from __future__ import annotations

import math


def _num(value) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def imu_status_text(imu_status: dict | None, oak_health: dict | None) -> str:
    """The IMU part of the 5 s journal status line.

    Fresh: ``IMU <heading>°→<target>°``. Stale: ``IMU STALE <age>s (frozen
    <heading>°)`` plus `` FAULT`` when the IMU watchdog has latched, so a
    dead stream no longer prints a normal-looking ``IMU 0°→0°``. Stale is
    the compensator's ``heading_stale`` (what the hold acts on), or the OAK
    health ``imu_stale`` when the OAK IMU is the heading source
    (``imu_watchdog``).
    """
    if not imu_status:
        return "IMU DISABLED"
    if not imu_status.get("is_available"):
        return "IMU OFFLINE"
    health = oak_health if isinstance(oak_health, dict) else {}
    heading = imu_status.get("heading_deg") or 0.0
    stale = bool(imu_status.get("heading_stale")) or (
        bool(health.get("imu_watchdog")) and bool(health.get("imu_stale"))
    )
    if not stale:
        target = imu_status.get("target_heading_deg") or 0.0
        return f"IMU {heading:.0f}°→{target:.0f}°"
    age = _num(health.get("imu_age_s"))
    age_str = f"{age:.0f}s" if age is not None else "?s"
    fault = " FAULT" if health.get("imu_stall_fault") else ""
    return f"IMU STALE {age_str} (frozen {heading:.0f}°){fault}"


class ImuStallMonitor:
    def __init__(self, event_s: float = 5.0) -> None:
        self._event_s = max(0.0, float(event_s or 0.0))
        self._stale_since: float | None = None
        self._stall_logged = False
        self._restarts: int | None = None
        self._fault = False

    def update(self, now: float, health: dict | None) -> list[dict]:
        """Events for this tick (usually none). ``now`` is monotonic."""
        if not isinstance(health, dict) or "imu_stale" not in health:
            return []
        events: list[dict] = []
        stale = bool(health.get("imu_stale"))
        age = _num(health.get("imu_age_s"))
        rounded_age = round(age, 1) if age is not None else None

        restarts = health.get("imu_stall_restarts")
        if isinstance(restarts, int) and not isinstance(restarts, bool):
            if self._restarts is not None and restarts > self._restarts:
                events.append({
                    "event": "oak_imu_stall_restart",
                    "imu_age_s": rounded_age,
                    "imu_stall_restarts": restarts,
                })
            self._restarts = restarts

        fault = bool(health.get("imu_stall_fault", False))
        if fault and not self._fault:
            events.append({
                "event": "oak_imu_stall_fault",
                "imu_age_s": rounded_age,
                "imu_stall_restarts": self._restarts,
            })
        self._fault = fault

        if stale:
            if self._stale_since is None:
                self._stale_since = now
            stale_for = now - self._stale_since
            if not self._stall_logged and stale_for >= self._event_s:
                self._stall_logged = True
                events.append({
                    "event": "oak_imu_stall",
                    "imu_age_s": rounded_age,
                    "stale_for_s": round(stale_for, 1),
                })
        else:
            if self._stall_logged:
                events.append({
                    "event": "oak_imu_recovered",
                    "stalled_for_s": round(now - (self._stale_since or now), 1),
                    "imu_stall_restarts": self._restarts,
                })
            self._stale_since = None
            self._stall_logged = False
        return events
