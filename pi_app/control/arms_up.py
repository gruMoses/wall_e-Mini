"""Both-wrists-above-shoulders pose detector.

Pure geometry on a PoseSample. MediaPipe / the camera live in the pose
worker; this module is hardware-free and import-safe in unit tests.
Image y grows DOWNWARD, so a raised wrist has a smaller y than its shoulder.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional


# Shoulder width below this is treated as degenerate (no divide-by-zero,
# and no trigger). Normalised image units.
_MIN_SHOULDER_WIDTH = 1e-4


@dataclass(frozen=True)
class PoseJoint:
    x: float
    y: float
    visibility: float


@dataclass(frozen=True)
class PoseSample:
    """One pose observation. A None sample means "no pose this frame"."""
    ts: float  # monotonic
    l_shoulder: PoseJoint
    r_shoulder: PoseJoint
    l_elbow: PoseJoint
    r_elbow: PoseJoint
    l_wrist: PoseJoint
    r_wrist: PoseJoint
    nose_y: float
    # Full-frame normalised crop window the landmarks were estimated in.
    # None keeps constructors that predate the crop-edge check working.
    crop_y0: Optional[float] = None
    crop_y1: Optional[float] = None


def _cfg(cfg: Any, name: str, default: float) -> float:
    if cfg is None:
        return default
    return float(getattr(cfg, name, default))


def _cfg_int(cfg: Any, name: str, default: int) -> int:
    if cfg is None:
        return default
    try:
        return int(getattr(cfg, name, default))
    except (TypeError, ValueError):
        return default


def arms_up(sample: Optional[PoseSample], cfg: Any = None) -> bool:
    """True only when BOTH wrists are raised above BOTH shoulders.

    Requires wrist, shoulder, and elbow visibility >= min_visibility on
    both sides, each wrist_y < shoulder_y - margin, and each elbow_y
    <= shoulder_y + margin. One arm up is not a trigger. Degenerate
    shoulder width returns False.

    When the sample carries a crop window (crop_y0 and crop_y1), a wrist
    pinned at the top of that crop is False: the landmark is on the crop
    boundary, not a measured point above the shoulders.
    """
    if sample is None:
        return False
    min_vis = _cfg(cfg, "min_visibility", 0.6)
    frac = _cfg(cfg, "wrist_above_shoulder_frac", 0.25)

    if sample.l_wrist.visibility < min_vis or sample.r_wrist.visibility < min_vis:
        return False
    if sample.l_shoulder.visibility < min_vis or sample.r_shoulder.visibility < min_vis:
        return False
    if sample.l_elbow.visibility < min_vis or sample.r_elbow.visibility < min_vis:
        return False

    shoulder_width = abs(sample.l_shoulder.x - sample.r_shoulder.x)
    if shoulder_width < _MIN_SHOULDER_WIDTH:
        return False
    margin = frac * shoulder_width

    if sample.l_wrist.y >= sample.l_shoulder.y - margin:
        return False
    if sample.r_wrist.y >= sample.r_shoulder.y - margin:
        return False

    # Both elbows are required. A low-visibility elbow used to be ignored;
    # that let a glitch put the wrists "up" while the arm was unmeasured.
    if sample.l_elbow.y > sample.l_shoulder.y + margin:
        return False
    if sample.r_elbow.y > sample.r_shoulder.y + margin:
        return False

    crop_y0 = sample.crop_y0
    crop_y1 = sample.crop_y1
    if crop_y0 is not None and crop_y1 is not None:
        span = float(crop_y1) - float(crop_y0)
        if span > 0.0:
            edge = _cfg(cfg, "crop_edge_margin", 0.03)
            limit = float(crop_y0) + edge * span
            if sample.l_wrist.y < limit or sample.r_wrist.y < limit:
                return False
    return True


# Timer comparisons tolerate float rounding: 1.4 - 1.0 is 0.39999999999999997,
# which must still satisfy a 0.4 s hold.
_TIME_EPS_S = 1e-6


class ArmsUpDetector:
    """Debounced both-arms-up state.

    ``active`` becomes True after ``arms_up`` has been continuously true
    for ``hold_s`` AND at least ``min_hold_samples`` new raw-true samples
    have arrived in the current streak. The streak counts only consecutive
    NEW samples (distinct ``PoseSample.ts``) whose timestamps are at most
    ``max_sample_gap_s`` apart. A larger gap, or an ``update()`` call more
    than ``max_sample_gap_s`` after the previous call, restarts the streak.

    It becomes False after ``release_s`` of false/None, or when no new
    sample has arrived for ``stale_s``.

    After a rising edge, another rising edge waits for
    ``min_hold_samples`` consecutive NEW samples that are raw False.
    None, a stale dropout, and a new raw-True sample reset that count.
    ``reset()`` clears the count and keeps the re-arm latch.
    """

    def __init__(self, cfg: Any = None) -> None:
        self._cfg = cfg
        self.active: bool = False
        self.raw: bool = False
        self.streak_s: float = 0.0
        self.last_sample_age_s: Optional[float] = None
        self.rising_edge: bool = False
        self._true_since_ts: Optional[float] = None
        self._true_count: int = 0
        self._false_count: int = 0
        self._false_since_now: Optional[float] = None
        self._last_seen_ts: Optional[float] = None
        self._last_new_sample_now: Optional[float] = None
        self._last_update_now: Optional[float] = None
        # True until a rising edge. min_hold_samples consecutive raw-False
        # NEW samples set it again.
        self._rearmed: bool = True

    def reset(self) -> None:
        """Clear the streak, active state, and gap trackers.

        The re-arm latch is kept: a rising edge that already fired stays
        consumed across a calibration / RC-stale reset. The consecutive
        raw-False count is cleared.
        """
        self.active = False
        self.raw = False
        self.streak_s = 0.0
        self.last_sample_age_s = None
        self.rising_edge = False
        self._true_since_ts = None
        self._true_count = 0
        self._false_count = 0
        self._false_since_now = None
        self._last_seen_ts = None
        self._last_new_sample_now = None
        self._last_update_now = None

    def _reset_streak(self) -> None:
        self._true_since_ts = None
        self._true_count = 0
        self.streak_s = 0.0

    def update(self, sample: Optional[PoseSample], now: float) -> dict:
        """Advance debounce state. ``now`` is monotonic seconds."""
        self.rising_edge = False
        max_gap = _cfg(self._cfg, "max_sample_gap_s", 0.25)
        if (
            self._last_update_now is not None
            and (now - self._last_update_now) > max_gap
        ):
            self._reset_streak()
            self._false_count = 0
        self._last_update_now = now

        is_new_sample = False
        if sample is not None:
            self.raw = arms_up(sample, self._cfg)
            if sample.ts != self._last_seen_ts:
                is_new_sample = True
                if (
                    self._last_seen_ts is not None
                    and (sample.ts - self._last_seen_ts) > max_gap
                ):
                    self._reset_streak()
                    self._false_count = 0
                self._last_seen_ts = sample.ts
                self._last_new_sample_now = now
        else:
            self.raw = False

        if self._last_new_sample_now is None:
            self.last_sample_age_s = None
        else:
            self.last_sample_age_s = now - self._last_new_sample_now

        hold_s = _cfg(self._cfg, "hold_s", 0.4)
        release_s = _cfg(self._cfg, "release_s", 0.3)
        stale_s = _cfg(self._cfg, "stale_s", 0.5)

        stale = (
            self._last_new_sample_now is not None
            and (now - self._last_new_sample_now) >= stale_s - _TIME_EPS_S
        )
        if stale:
            # Stale drops active. It does not re-arm the rising edge, and
            # it breaks the consecutive raw-False run.
            self.active = False
            self._reset_streak()
            self._false_count = 0
        elif is_new_sample:
            self._apply_observation(sample.ts if sample is not None else now, now, hold_s, release_s)
        elif sample is None:
            # None is not a timestamped sample; it still counts as false
            # for the release timer (wall clock). It does not re-arm, and
            # it breaks the consecutive raw-False run.
            self._false_count = 0
            self._apply_false(now, release_s)

        return {
            "active": self.active,
            "raw": self.raw,
            "streak_s": self.streak_s,
            "last_sample_age_s": self.last_sample_age_s,
            "rising_edge": self.rising_edge,
        }

    def _min_hold_samples(self) -> int:
        min_n = _cfg_int(self._cfg, "min_hold_samples", 3)
        if min_n < 1:
            return 1
        return min_n

    def _apply_observation(self, sample_ts: float, now: float, hold_s: float, release_s: float) -> None:
        if self.raw:
            self._false_count = 0
            self._false_since_now = None
            if self._true_since_ts is None:
                self._true_since_ts = sample_ts
                self._true_count = 0
            self._true_count += 1
            self.streak_s = max(0.0, sample_ts - self._true_since_ts)
            min_n = self._min_hold_samples()
            if (
                (not self.active)
                and self._rearmed
                and self._true_count >= min_n
                and self.streak_s >= hold_s - _TIME_EPS_S
            ):
                self.active = True
                self.rising_edge = True
                self._rearmed = False
        else:
            # Re-arm only after a sustained arms-down, not one glitch frame.
            self._false_count += 1
            if self._false_count >= self._min_hold_samples():
                self._rearmed = True
            self._apply_false(now, release_s)

    def _apply_false(self, now: float, release_s: float) -> None:
        self._true_since_ts = None
        self._true_count = 0
        self.streak_s = 0.0
        if self._false_since_now is None:
            self._false_since_now = now
        if self.active and (now - self._false_since_now) >= release_s - _TIME_EPS_S:
            self.active = False
