"""Operator-engaged corridor mute for mesh netting loaded on the robot.

2026-10-08: electric-fence mesh netting draped over the lens reads as a
solid wall at 0.40-0.50 m to the stereo camera (60 raw frames: 97-98
percent of the valid corridor pixels at 0.35-0.65 m; the drive log: 81 m
with the corridor at 0.40-0.50 m), so the MANUAL obstacle curve held Kevin
at 0.3-0.4 throttle. Depth alone cannot tell the mesh from a wall, and an
automatic "the wall moves with us" latch was rejected in review: a close
person YOLO misses, wheels slipping on grass in front of a real wall, or
anything the robot is pushing satisfies the same test, and a safety mute
must be one the hazard cannot satisfy.

So the mute is EXPLICIT and self-limiting (pure logic, no hardware):

* The operator taps the phone with the netting already on the lens:
  armed, MANUAL, and a dense near field in view (``near_field_present``
  with ``min_near_share``), or the tap is refused. There is no waiting
  state: a mute cannot arm itself on the next close surface.
* While engaged, MANUAL ignores the depth corridor only as long as the
  field holds (share hysteresis: ``hold_near_share``). A finite corridor
  reading beyond ``max_distance_m`` ends the mute ON THAT TICK, so a fence
  that appears when the netting lifts is never ignored. Only a dropout (no
  reading, stale depth, or a share dip inside the band) gets a hold of
  ``absent_release_s`` before the drop.
* It also drops on any person or stop/slow-tier animal detection, on
  disarm, on leaving MANUAL, on ``timeout_s``, or on the operator's tap.
  After any drop a new tap is required: there is no resume.
* Thread safety: the web thread only queues a request; ``update()`` on the
  control thread is the single writer of the state (review 2026-10-08: a
  same-tick operator drop could otherwise leave ``active`` True with
  ``engaged`` False).
* The YOLO 0.8 m person/animal stop arrives on its own channel (distance
  0.0, forced upstream) and is never muted; a stale depth is never muted;
  FOLLOW_ME and WAYPOINT_NAV never use the mute (Kevin's safety principle:
  human input may carry a little extra risk, autonomy never).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass


def near_field_present(
    distance_m: float | None,
    near_px: int | float | None,
    support_px: int | float | None,
    max_distance_m: float,
    min_near_share: float,
) -> bool:
    """True when the corridor reports a dense, close surface.

    ``distance_m`` is the published corridor distance (0.0 = the YOLO stop
    channel, never a near field; inf/None = clear or no reading).
    ``near_px`` / ``support_px`` are the corridor's valid pixels inside
    slow_distance_m and its valid pixels in total.
    """
    share = near_share(near_px, support_px)
    return _field_present(distance_m, share, max_distance_m, min_near_share)


def near_share(near_px, support_px) -> float | None:
    try:
        support = float(support_px or 0.0)
        near = float(near_px or 0.0)
    except (TypeError, ValueError):
        return None
    if support <= 0.0:
        return None
    return near / support


def _field_present(distance_m, share, max_distance_m: float, share_thr: float) -> bool:
    if distance_m is None or share is None:
        return False
    try:
        d = float(distance_m)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(d) or d <= 0.0 or d > float(max_distance_m):
        return False
    return float(share) >= float(share_thr)


def detection_drops_mute(all_detections, persons) -> bool:
    """Any person, or any detection in the stop or slow safety tier.

    Birds are the ``log`` tier on purpose: the flock is always in the yard.
    """
    if persons:
        return True
    for d in all_detections or ():
        if getattr(d, "safety_tier", "log") in ("stop", "slow"):
            return True
    return False


@dataclass(frozen=True)
class NettingMuteState:
    engaged: bool
    active: bool
    since_s: float | None
    drop_reason: str | None      # why it last dropped (sticky until re-engaged)
    refusal: str | None          # why the last tap was refused (sticky until a tap succeeds)
    event: str | None = None     # this tick: "active" | "dropped" | "refused"

    def as_dict(self) -> dict:
        return {
            "engaged": self.engaged,
            "active": self.active,
            "since_s": None if self.since_s is None else round(self.since_s, 1),
            "drop_reason": self.drop_reason,
            "refusal": self.refusal,
            "event": self.event,
        }


class NettingMute:
    def __init__(
        self,
        enabled: bool = True,
        timeout_s: float = 900.0,
        absent_release_s: float = 0.5,
        max_distance_m: float = 0.55,
        min_near_share: float = 0.85,
        hold_near_share: float = 0.70,
    ) -> None:
        self._enabled = bool(enabled)
        self._timeout_s = max(0.0, float(timeout_s or 0.0))
        self._absent_release_s = max(0.0, float(absent_release_s or 0.0))
        self._max_distance_m = float(max_distance_m)
        self._min_near_share = float(min_near_share)
        self._hold_near_share = min(float(hold_near_share), self._min_near_share)
        self._lock = threading.Lock()
        self._pending: str | None = None     # "on" | "off", written by the web thread
        # State below is written ONLY by update() (control thread).
        self._engaged_since: float | None = None
        self._absent_since: float | None = None
        self._drop_reason: str | None = None
        self._refusal: str | None = None

    @property
    def engaged(self) -> bool:
        return self._engaged_since is not None

    @property
    def active(self) -> bool:
        return self.engaged

    @property
    def max_distance_m(self) -> float:
        return self._max_distance_m

    def check_on(self, is_armed: bool, is_manual: bool, distance_m, share) -> tuple[bool, str]:
        """The tap's preconditions, read-only (also re-checked in update())."""
        if not self._enabled:
            return False, "netting mute disabled in config"
        if not is_armed:
            return False, "robot must be armed"
        if not is_manual:
            return False, "MANUAL only"
        if not _field_present(distance_m, share, self._max_distance_m, self._min_near_share):
            return False, "no netting in view: load it over the lens first"
        return True, "engaged"

    def request_on(self, is_armed: bool, is_manual: bool, distance_m, share) -> tuple[bool, str]:
        """Web thread: queue an engage. Returns the provisional answer."""
        if self.engaged:
            return True, "already on"
        ok, reason = self.check_on(is_armed, is_manual, distance_m, share)
        if ok:
            with self._lock:
                self._pending = "on"
        return ok, reason

    def request_off(self) -> tuple[bool, str]:
        """Web thread: queue a drop, or cancel a tap that has not been applied yet."""
        with self._lock:
            if self._pending == "on":
                self._pending = None
                return True, "cancelled"
            if not self.engaged:
                return True, "already off"
            self._pending = "off"
        return True, "dropped"

    def _drop(self, reason: str) -> None:
        self._engaged_since = None
        self._absent_since = None
        self._drop_reason = reason

    def update(
        self,
        now: float,
        is_armed: bool,
        is_manual: bool,
        distance_m,
        share,
        detection_present: bool,
    ) -> NettingMuteState:
        """Control thread, every tick. ``distance_m`` None = no fresh reading."""
        with self._lock:
            pending, self._pending = self._pending, None
        event = None
        if pending == "off" and self.engaged:
            self._drop("operator")
            event = "dropped"
        elif pending == "on" and not self.engaged:
            ok, reason = self.check_on(is_armed, is_manual, distance_m, share)
            if ok:
                self._engaged_since = now
                self._absent_since = None
                self._drop_reason = None
                self._refusal = None
                event = "active"
            else:
                self._refusal = reason
                event = "refused"

        if self.engaged and event != "active":
            since = now - float(self._engaged_since)
            if not is_armed:
                self._drop("disarmed")
            elif not is_manual:
                self._drop("left MANUAL")
            elif detection_present:
                self._drop("person or animal detected")
            elif self._timeout_s > 0.0 and since >= self._timeout_s:
                self._drop("timeout")
            elif _field_present(distance_m, share, self._max_distance_m, self._hold_near_share):
                self._absent_since = None
            else:
                far = False
                try:
                    far = (distance_m is not None and math.isfinite(float(distance_m))
                           and float(distance_m) > self._max_distance_m)
                except (TypeError, ValueError):
                    far = False
                if far:
                    # A real reading beyond the band: the netting is off the
                    # lens. Never hold against it.
                    self._drop("near field gone")
                elif self._absent_since is None:
                    self._absent_since = now
                elif (now - self._absent_since) >= self._absent_release_s:
                    self._drop("near field gone")
            if not self.engaged:
                event = "dropped"
        since_s = None if self._engaged_since is None else now - self._engaged_since
        return NettingMuteState(
            engaged=self.engaged,
            active=self.engaged,
            since_s=since_s,
            drop_reason=self._drop_reason,
            refusal=self._refusal,
            event=event,
        )
