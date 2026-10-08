"""Vision-worker hang monitor for the main loop (pure logic, no hardware).

Incident 2026-10-04 13:41 -> 2026-10-08 17:36: a USB hub glitch made the
colour-stall watchdog restart the OAK session; ``pipeline.stop()`` in
depthai's teardown never returned. The worker thread sat inside C++ for four
days with ``pipeline_running`` still True, so the supervisor never ran,
depth was four days stale, FOLLOW_ME was refused, and MANUAL drove at the
10 percent stale-depth floor. The only journal evidence was the status
line's ``scl=0.1``. Kevin's report: "something we did recently makes the
robot drive extremely slowly."

A thread stuck inside depthai cannot be interrupted from Python, so the
recovery is a process exit and a systemd restart (``Restart=on-failure``).
This module turns the reader's ``vision_hung`` health flag into actions:

* ``restart_permitted`` -- the gate. Not ``cmd.is_armed`` alone: that goes
  False on a 1 s RC dropout with the arm switch still up (Grok review
  2026-10-08). With a fresh RC link the RAW ch3 level decides (no debounce,
  so a switch flipped high a moment ago already blocks). A short dropout
  blocks (the switch may still be up). A long silence, or no RC ever,
  allows: the transmitter is off, RC-stale forces disarm and phone teleop
  needs RC armed, so nobody can be driving.
* ``VisionHangMonitor`` -- ``onset`` prints and writes a JSON event;
  ``log_now`` every ``log_interval_s``; ``restart`` fires once, after
  ``hang_restart_s`` hung AND the gate has held continuously for
  ``settle_s``; ``recovered`` re-arms for a later hang.
  ``hang_restart_s <= 0`` means log only.
* ``RestartBudget`` -- at most ``max_restarts`` exits per ``window_s``,
  remembered in a small JSON file across restarts, so a hang that recurs on
  every boot (hub flapping) ends in "log only" instead of a 95 s loop.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


def restart_permitted(
    is_armed: bool,
    rc_age_s: float | None,
    ch3_us: int | None,
    arm_low_threshold_us: int,
    rc_fresh_s: float = 1.0,
    rc_off_s: float = 30.0,
) -> tuple[bool, str]:
    """Return (may_restart, reason) for this tick.

    ``rc_age_s`` None means no RC frame has ever been seen.
    """
    if is_armed:
        return False, "armed"
    if rc_age_s is None:
        return True, "no RC ever"
    if rc_age_s <= rc_fresh_s:
        if ch3_us is not None and ch3_us <= arm_low_threshold_us:
            return True, "disarmed, switch low"
        return False, "switch up"
    if rc_off_s > 0.0 and rc_age_s >= rc_off_s:
        return True, f"disarmed, no RC for {rc_age_s:.0f} s"
    return False, "RC dropout"


@dataclass(frozen=True)
class VisionHangAction:
    hung: bool
    hung_for_s: float
    onset: bool = False
    log_now: bool = False
    restart: bool = False
    recovered: bool = False
    restart_in_s: float | None = None  # seconds until the restart fires; None = gated off


class VisionHangMonitor:
    def __init__(
        self,
        hang_restart_s: float,
        log_interval_s: float = 60.0,
        settle_s: float = 10.0,
    ) -> None:
        self._restart_s = max(0.0, float(hang_restart_s or 0.0))
        self._log_interval_s = max(1.0, float(log_interval_s or 60.0))
        self._settle_s = max(0.0, float(settle_s or 0.0))
        self._hung_since: float | None = None
        self._clear_since: float | None = None
        self._last_log_ts: float = 0.0
        self._restart_requested: bool = False

    @property
    def hung_since(self) -> float | None:
        return self._hung_since

    @property
    def restart_requested(self) -> bool:
        return self._restart_requested

    def update(self, now: float, hung: bool, may_restart: bool) -> VisionHangAction:
        # The settle clock runs on every tick, hung or not: it measures how
        # long the gate (disarmed, switch low / transmitter off) has held.
        if may_restart:
            if self._clear_since is None:
                self._clear_since = now
        else:
            self._clear_since = None

        if not hung:
            if self._hung_since is None:
                return VisionHangAction(hung=False, hung_for_s=0.0)
            hung_for = now - self._hung_since
            self._hung_since = None
            self._last_log_ts = 0.0
            self._restart_requested = False
            return VisionHangAction(hung=False, hung_for_s=hung_for, recovered=True)

        if self._hung_since is None:
            self._hung_since = now
            self._last_log_ts = now
            return VisionHangAction(
                hung=True,
                hung_for_s=0.0,
                onset=True,
                log_now=True,
                restart_in_s=self._restart_in(now, 0.0),
            )

        hung_for = now - self._hung_since
        log_now = (now - self._last_log_ts) >= self._log_interval_s
        if log_now:
            self._last_log_ts = now
        restart = (
            self._restart_s > 0.0
            and self._clear_since is not None
            and (now - self._clear_since) >= self._settle_s
            and not self._restart_requested
            and hung_for >= self._restart_s
        )
        if restart:
            self._restart_requested = True
        return VisionHangAction(
            hung=True,
            hung_for_s=hung_for,
            log_now=log_now,
            restart=restart,
            restart_in_s=self._restart_in(now, hung_for),
        )

    def _restart_in(self, now: float, hung_for: float) -> float | None:
        """Seconds until the restart fires; None when gated off or disabled."""
        if (
            self._restart_s <= 0.0
            or self._clear_since is None
            or self._restart_requested
        ):
            return None
        settle_left = max(0.0, self._settle_s - (now - self._clear_since))
        hang_left = max(0.0, self._restart_s - hung_for)
        return max(settle_left, hang_left)


def read_boot_id() -> str:
    """The kernel boot id, or "unknown" when it cannot be read."""
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip() or "unknown"
    except Exception:
        return "unknown"


class RestartBudget:
    """At most ``max_restarts`` self-restarts per ``window_s``, within one boot.

    Stamps are ``time.monotonic()`` seconds (since boot) and the file also
    carries the kernel boot id. The restart loop this bounds happens within
    one boot (systemd restarts the service, not the Pi), and the monotonic
    clock is immune to the epoch steps this Pi takes at its first NTP sync
    (Grok review 2026-10-08: epoch stamps from a pre-NTP boot were pruned
    by the jump, which reopened the loop). A file from another boot, or an
    unreadable one, counts as an empty history. A stamp that cannot be
    written DENIES the restart: a budget that cannot be kept must not turn
    into an unbounded loop (``logs/`` has filled before).
    ``max_restarts <= 0`` = no cap.
    """

    def __init__(
        self,
        path: Path | str,
        max_restarts: int = 3,
        window_s: float = 3600.0,
        boot_id: str | None = None,
    ) -> None:
        self.path = Path(path)
        self.max_restarts = int(max_restarts)
        self.window_s = max(0.0, float(window_s or 0.0))
        self.boot_id = boot_id if boot_id is not None else read_boot_id()

    def _load(self) -> list[float]:
        try:
            data = json.loads(self.path.read_text())
        except Exception:
            return []
        if not isinstance(data, dict) or data.get("boot_id") != self.boot_id:
            return []
        stamps = data.get("stamps")
        if not isinstance(stamps, list):
            return []
        out: list[float] = []
        for t in stamps:
            if isinstance(t, (int, float)):
                out.append(float(t))
        return out

    def recent(self, now_mono: float) -> list[float]:
        return [t for t in self._load() if 0.0 <= (now_mono - t) < self.window_s]

    def allow(self, now_mono: float) -> tuple[bool, int]:
        """Record and allow a restart, or deny it. Returns (allowed, count):
        the restarts in the window including this one when allowed, or the
        count that blocked it. ``now_mono`` is ``time.monotonic()``."""
        stamps = self.recent(now_mono)
        if self.max_restarts > 0 and len(stamps) >= self.max_restarts:
            return False, len(stamps)
        stamps.append(now_mono)
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(self.path.suffix + ".tmp")
            tmp.write_text(json.dumps({"boot_id": self.boot_id, "stamps": stamps}))
            tmp.replace(self.path)
        except Exception:
            return False, len(stamps) - 1
        return True, len(stamps)
