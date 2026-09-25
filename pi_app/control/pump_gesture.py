"""Log-only low two-hand "pump" detector.

Pure geometry on a person box plus the stereo range from the same
detection. The camera and the tracker live elsewhere; this module is
hardware-free and import-safe in unit tests.

Frames are counted on NEW observations (a ``PumpObservation.ts`` this
detector has not processed). A repeated observation or None only runs
the time checks. A 30 Hz control tick must not count as a frame.

Metric edges use one range, ``z_m``, and the NN-frame intrinsics. A
person walking closer grows the normalised box and does not look like
a pump.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass
from typing import Any, Optional


# Timer comparisons tolerate float rounding. A gap that lands on the
# threshold counts as reached (1.4 - 1.0 is 0.39999999999999997).
_TIME_EPS_S = 1e-6


def _cfg_float(cfg: Any, name: str, default: float) -> float:
    if cfg is None:
        return default
    try:
        return float(getattr(cfg, name, default))
    except (TypeError, ValueError):
        return default


def _cfg_int(cfg: Any, name: str, default: int) -> int:
    if cfg is None:
        return default
    try:
        return int(getattr(cfg, name, default))
    except (TypeError, ValueError):
        return default


def _reached(elapsed: float, limit: float) -> bool:
    """True once ``elapsed`` has reached ``limit`` (float-tolerant)."""
    return elapsed >= limit - _TIME_EPS_S


@dataclass(frozen=True)
class PumpObservation:
    """One published person detection. None means no detection this call."""
    ts: float  # monotonic time the detection list was published
    bbox: tuple[float, float, float, float]  # normalised NN-frame (xmin, ymin, xmax, ymax)
    z_m: float
    depth_ok: bool  # depth_status == "ok"
    track_id: Optional[int]


class PumpDetector:
    """Rest-width pump state machine. States: idle, active, cooldown.

    IDLE builds a rolling rest width, freezes it at the first rise, and
    starts after ``start_peaks`` completed peaks inside ``start_window_s``.
    ACTIVE holds the frozen rest and stops on hands down, a detection
    gap, or the time limit. COOLDOWN ignores pumps until the cooldown
    has passed and hands have been down for ``rearm_rest_s``.
    """

    def __init__(self, cfg: Any = None) -> None:
        self._cfg = cfg
        self._min_range_m = _cfg_float(cfg, "min_range_m", 1.0)
        self._max_range_m = _cfg_float(cfg, "max_range_m", 5.0)
        self._edge_margin = _cfg_float(cfg, "edge_margin", 0.01)
        self._min_body_w_m = _cfg_float(cfg, "min_body_w_m", 0.3)
        self._max_body_w_m = _cfg_float(cfg, "max_body_w_m", 2.2)
        self._rest_window_s = _cfg_float(cfg, "rest_window_s", 3.0)
        self._rest_min_samples = _cfg_int(cfg, "rest_min_samples", 8)
        self._rest_min_span_s = _cfg_float(cfg, "rest_min_span_s", 1.5)
        self._freeze_ratio = _cfg_float(cfg, "freeze_ratio", 1.4)
        self._out_ratio = _cfg_float(cfg, "out_ratio", 1.5)
        self._edge_out_frac = _cfg_float(cfg, "edge_out_frac", 0.25)
        self._max_centre_shift_m = _cfg_float(cfg, "max_centre_shift_m", 0.15)
        self._min_out_frames = _cfg_int(cfg, "min_out_frames", 3)
        self._peak_end_ratio = _cfg_float(cfg, "peak_end_ratio", 1.4)
        self._start_peaks = _cfg_int(cfg, "start_peaks", 2)
        self._start_window_s = _cfg_float(cfg, "start_window_s", 3.0)
        self._continue_window_s = _cfg_float(cfg, "continue_window_s", 0.7)
        self._no_detection_stop_s = _cfg_float(cfg, "no_detection_stop_s", 0.3)
        self._max_active_s = _cfg_float(cfg, "max_active_s", 8.0)
        self._cooldown_s = _cfg_float(cfg, "cooldown_s", 2.0)
        self._rearm_rest_ratio = _cfg_float(cfg, "rearm_rest_ratio", 1.3)
        self._rearm_rest_s = _cfg_float(cfg, "rearm_rest_s", 1.0)
        self.reset()

    def reset(self) -> None:
        """Clear gesture state, the rest window, and the start counter."""
        self._state: str = "idle"
        self._last_obs_ts: Optional[float] = None
        self._have_track: bool = False
        self._track_id: Optional[int] = None
        # (t, w_m, c_m) accepted samples inside the rest window.
        self._rest: list[tuple[float, float, float]] = []
        self._rest_w: Optional[float] = None
        self._rest_c: Optional[float] = None
        self._rest_l: Optional[float] = None
        self._rest_r: Optional[float] = None
        self._frozen: bool = False
        self._freeze_ts: Optional[float] = None
        self._rebaseline_until: Optional[float] = None
        self._w: Optional[float] = None
        self._c: Optional[float] = None
        self._l: Optional[float] = None
        self._r: Optional[float] = None
        self._out_run: int = 0
        self._out_flag: bool = False
        self._peak_times: list[float] = []
        self._last_accepted_ts: Optional[float] = None
        self._last_out_ts: Optional[float] = None
        self._active_since: Optional[float] = None
        self._cooldown_since: Optional[float] = None
        self._rearm_since: Optional[float] = None
        self._rearm_last: Optional[float] = None
        self._start_event: bool = False
        self._stop_event: bool = False
        self._stop_reason: Optional[str] = None
        self._stop_active_s: float = 0.0
        self._would_start_count: int = 0

    def update(
        self,
        obs: Optional[PumpObservation],
        intrinsics: Optional[tuple[float, float, float, float]],
        frame_w: int,
        now: float,
    ) -> dict:
        """Advance the state machine. ``now`` is monotonic seconds.

        ``intrinsics`` is ``(fx, fy, cx, cy)`` in pixels for the NN frame.
        ``frame_w`` is that frame's width in pixels. A repeated ``obs.ts``
        or ``obs is None`` does not count as a frame.
        """
        self._start_event = False
        self._stop_event = False
        self._out_flag = False
        accepted = False
        reject: Optional[str] = None
        new = False
        meas: Optional[tuple[float, float]] = None

        if self._state == "idle":
            self._maybe_unfreeze(now)

        if obs is not None and obs.ts != self._last_obs_ts:
            new = True
            self._last_obs_ts = obs.ts
            geom, reject = self._classify(obs, intrinsics, frame_w)
            if geom is not None:
                meas = (geom[0], geom[1])
            if reject is None and geom is not None:
                if self._have_track and obs.track_id != self._track_id:
                    self._reset_for_new_target()
                self._have_track = True
                self._track_id = obs.track_id
                self._w, self._c, self._l, self._r = geom
                accepted = True
                self._last_accepted_ts = now
                self._apply_accepted(now)

        if self._state == "active":
            self._maybe_stop(now)
            # The frame that ends ACTIVE can be the first hands-down sample.
            if accepted and self._state == "cooldown":
                self._note_rearm(now)
        if self._state == "cooldown":
            self._maybe_leave_cooldown(now, accepted)

        self._prune_peaks(now)
        return self._result(now, accepted, reject if new else None, meas, new, obs)

    # ── Acceptance ────────────────────────────────────────────────────────

    def _classify(
        self,
        obs: PumpObservation,
        intrinsics: Optional[tuple[float, float, float, float]],
        frame_w: int,
    ) -> tuple[Optional[tuple[float, float, float, float]], Optional[str]]:
        """Return ``((w_m, c_m, l_m, r_m), None)`` or ``(geom_or_None, reason)``.

        Reject order: intrinsics, depth, range, edge, width.
        """
        if intrinsics is None or frame_w is None or int(frame_w) <= 0:
            return None, "no_intrinsics"
        try:
            fx = float(intrinsics[0])
            # fy, cy are part of the contract; cx is the one the width uses.
            float(intrinsics[1])
            cx = float(intrinsics[2])
            float(intrinsics[3])
        except (TypeError, ValueError, IndexError):
            return None, "no_intrinsics"
        if fx <= 0.0:
            return None, "no_intrinsics"
        if not obs.depth_ok:
            return None, "depth"
        try:
            z = float(obs.z_m)
        except (TypeError, ValueError):
            return None, "range"
        if z < self._min_range_m or z > self._max_range_m:
            return None, "range"
        try:
            xmin = float(obs.bbox[0])
            float(obs.bbox[1])
            xmax = float(obs.bbox[2])
            float(obs.bbox[3])
        except (TypeError, ValueError, IndexError):
            return None, "edge"
        margin = self._edge_margin
        if xmin <= margin or xmax >= 1.0 - margin:
            return None, "edge"
        fw = float(frame_w)
        l_m = (xmin * fw - cx) * z / fx
        r_m = (xmax * fw - cx) * z / fx
        w_m = r_m - l_m
        c_m = (l_m + r_m) / 2.0
        geom = (w_m, c_m, l_m, r_m)
        if w_m < self._min_body_w_m or w_m > self._max_body_w_m:
            return geom, "width"
        return geom, None

    def _reset_for_new_target(self) -> None:
        """A new accepted track_id drops the gesture and keeps the counter."""
        count = self._would_start_count
        last_ts = self._last_obs_ts
        self.reset()
        self._would_start_count = count
        self._last_obs_ts = last_ts

    # ── Rest, out, peak ───────────────────────────────────────────────────

    def _maybe_unfreeze(self, now: float) -> None:
        if not self._frozen or self._freeze_ts is None:
            return
        if _reached(now - self._freeze_ts, self._start_window_s):
            self._frozen = False
            self._freeze_ts = None
            self._trim_rest(now)
            # A rise that did not become a start can be a lasting width
            # change (a coat, a carried board). Let the rest re-learn for
            # one window; otherwise the next wide frame re-freezes it and
            # the baseline stays stale for good. The abandoned rise's run
            # and peaks go with it.
            self._rebaseline_until = now + self._rest_window_s
            self._out_run = 0
            self._peak_times.clear()

    def _window_valid(self) -> bool:
        if len(self._rest) < self._rest_min_samples:
            return False
        span = self._rest[-1][0] - self._rest[0][0]
        return _reached(span, self._rest_min_span_s)

    def _trim_rest(self, now: float) -> None:
        limit = self._rest_window_s
        self._rest = [s for s in self._rest if now - s[0] <= limit + _TIME_EPS_S]

    def _recompute_rest(self) -> None:
        """Update the frozen-or-live baseline only when the window qualifies.

        A short window keeps the previous baseline. That is what lets a
        rearmed gesture still compare against the rest from before the
        cooldown, instead of going blind for another ``rest_min_span_s``.
        """
        if not self._window_valid():
            return
        self._rest_w = float(statistics.median(s[1] for s in self._rest))
        self._rest_c = float(statistics.median(s[2] for s in self._rest))
        self._rest_l = self._rest_c - self._rest_w / 2.0
        self._rest_r = self._rest_c + self._rest_w / 2.0

    def _add_rest_sample(self, now: float) -> None:
        if self._w is None or self._c is None:
            return
        self._rest.append((now, self._w, self._c))
        self._trim_rest(now)
        self._recompute_rest()

    def _is_out(self) -> bool:
        rw = self._rest_w
        if (
            rw is None or rw <= 0.0 or self._rest_c is None
            or self._rest_l is None or self._rest_r is None
            or self._w is None or self._l is None or self._r is None
            or self._c is None
        ):
            return False
        if self._w < self._out_ratio * rw:
            return False
        if self._l > self._rest_l - self._edge_out_frac * rw:
            return False
        if self._r < self._rest_r + self._edge_out_frac * rw:
            return False
        # Strict: a shift of exactly max_centre_shift_m is not "out".
        if abs(self._c - self._rest_c) >= self._max_centre_shift_m:
            return False
        return True

    def _note_out_or_peak(self, now: float, *, count_peak: bool) -> None:
        out = self._is_out()
        self._out_flag = out
        if out:
            self._out_run += 1
            self._last_out_ts = now
            return
        below_end = (
            self._rest_w is not None
            and self._w is not None
            and self._w < self._peak_end_ratio * self._rest_w
        )
        armed = self._out_run >= self._min_out_frames
        if armed and not below_end:
            # A peak that already has its out frames holds through the
            # band between peak_end_ratio and out_ratio (and through a
            # wide frame that fails symmetry). Resetting here dropped
            # real pumps whose descent landed one frame in that band.
            return
        if count_peak and armed and below_end:
            self._peak_times.append(now)
        self._out_run = 0

    def _prune_peaks(self, now: float) -> None:
        win = self._start_window_s
        self._peak_times = [t for t in self._peak_times if now - t <= win + _TIME_EPS_S]

    def _apply_accepted(self, now: float) -> None:
        if self._state == "idle":
            self._apply_idle(now)
        elif self._state == "active":
            self._note_out_or_peak(now, count_peak=False)
        else:
            self._note_out_or_peak(now, count_peak=False)
            self._note_rearm(now)

    def _apply_idle(self, now: float) -> None:
        if self._rebaseline_until is not None and now < self._rebaseline_until:
            # Re-learning the rest: every sample goes in, and no run is
            # tracked. A run armed against the stale rest would otherwise
            # complete as a false peak the moment the rest catches up.
            self._add_rest_sample(now)
            self._out_flag = self._is_out()
            self._out_run = 0
            return
        # Freeze on the first rise and do not let that sample into the median.
        if not self._frozen and self._rest_w is not None and self._w is not None:
            if self._w >= self._freeze_ratio * self._rest_w:
                self._frozen = True
                self._freeze_ts = now
            else:
                self._add_rest_sample(now)
        elif not self._frozen:
            self._add_rest_sample(now)
        self._note_out_or_peak(now, count_peak=True)
        self._prune_peaks(now)
        if len(self._peak_times) >= self._start_peaks:
            self._enter_active(now)

    def _enter_active(self, now: float) -> None:
        self._state = "active"
        self._active_since = now
        self._rebaseline_until = None
        self._start_event = True
        self._would_start_count += 1
        self._stop_reason = None
        if not self._frozen:
            self._frozen = True
            self._freeze_ts = now

    # ── Active / cooldown ─────────────────────────────────────────────────

    def _maybe_stop(self, now: float) -> None:
        if self._state != "active":
            return
        if (
            self._last_accepted_ts is not None
            and _reached(now - self._last_accepted_ts, self._no_detection_stop_s)
        ):
            self._enter_cooldown(now, "no_detection")
            return
        if (
            self._last_out_ts is not None
            and _reached(now - self._last_out_ts, self._continue_window_s)
        ):
            self._enter_cooldown(now, "hands_down")
            return
        if (
            self._active_since is not None
            and _reached(now - self._active_since, self._max_active_s)
        ):
            self._enter_cooldown(now, "time_limit")

    def _enter_cooldown(self, now: float, reason: str) -> None:
        elapsed = 0.0
        if self._active_since is not None:
            elapsed = max(0.0, now - self._active_since)
        self._stop_active_s = elapsed
        self._stop_event = True
        self._stop_reason = reason
        self._state = "cooldown"
        self._cooldown_since = now
        self._active_since = None
        self._out_run = 0
        self._rearm_since = None
        self._rearm_last = None

    def _note_rearm(self, now: float) -> None:
        rw = self._rest_w
        if rw is None or self._w is None:
            return
        if self._w < self._rearm_rest_ratio * rw:
            if self._rearm_since is None:
                self._rearm_since = now
            self._rearm_last = now
        else:
            self._rearm_since = None
            self._rearm_last = None

    def _maybe_leave_cooldown(self, now: float, accepted: bool) -> None:
        if self._state != "cooldown":
            return
        # Hands-down evidence that has gone stale does not count as continued.
        if (
            not accepted
            and self._rearm_last is not None
            and _reached(now - self._rearm_last, self._no_detection_stop_s)
        ):
            self._rearm_since = None
            self._rearm_last = None
        if self._cooldown_since is None:
            return
        if not _reached(now - self._cooldown_since, self._cooldown_s):
            return
        if self._rearm_since is None:
            return
        if not _reached(now - self._rearm_since, self._rearm_rest_s):
            return
        self._state = "idle"
        self._peak_times.clear()
        self._frozen = False
        self._freeze_ts = None
        self._out_run = 0
        self._rearm_since = None
        self._rearm_last = None
        self._stop_reason = None
        self._cooldown_since = None
        if accepted:
            self._add_rest_sample(now)

    # ── Result ────────────────────────────────────────────────────────────

    def _result(
        self,
        now: float,
        accepted: bool,
        reject: Optional[str],
        meas: Optional[tuple[float, float]],
        new: bool,
        obs: Optional[PumpObservation],
    ) -> dict:
        if accepted:
            w_m, c_m = self._w, self._c
        elif new and reject == "width" and meas is not None:
            w_m, c_m = meas
        elif new:
            w_m, c_m = None, None
        else:
            w_m, c_m = self._w, self._c
        ratio = None
        if (
            w_m is not None and self._rest_w is not None and self._rest_w > 0.0
        ):
            ratio = w_m / self._rest_w
        if self._stop_event:
            active_s = self._stop_active_s
        elif self._state == "active" and self._active_since is not None:
            active_s = max(0.0, now - self._active_since)
        else:
            active_s = 0.0
        if new and obs is not None:
            track_id = obs.track_id
        else:
            track_id = self._track_id
        return {
            "state": self._state,
            "accepted": accepted,
            "reject_reason": reject,
            "w_m": w_m,
            "c_m": c_m,
            "rest_w": self._rest_w,
            "ratio": ratio,
            "out": bool(self._out_flag) if accepted else False,
            "out_run": self._out_run,
            "peaks": len(self._peak_times),
            "start_event": self._start_event,
            "stop_event": self._stop_event,
            "stop_reason": self._stop_reason,
            "active_s": active_s,
            "track_id": track_id,
            "would_start_count": self._would_start_count,
        }
