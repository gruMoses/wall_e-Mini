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


def _cfg(cfg: Any, name: str, default: float) -> float:
    if cfg is None:
        return default
    return float(getattr(cfg, name, default))


def arms_up(sample: Optional[PoseSample], cfg: Any = None) -> bool:
    """True only when BOTH wrists are raised above BOTH shoulders.

    Requires wrist and shoulder visibility >= min_visibility on both
    sides, each wrist_y < shoulder_y - margin, and (when visible) each
    elbow not hanging more than margin below its shoulder. One arm up
    is not a trigger. Degenerate shoulder width returns False.
    """
    if sample is None:
        return False
    min_vis = _cfg(cfg, "min_visibility", 0.6)
    frac = _cfg(cfg, "wrist_above_shoulder_frac", 0.25)

    if sample.l_wrist.visibility < min_vis or sample.r_wrist.visibility < min_vis:
        return False
    if sample.l_shoulder.visibility < min_vis or sample.r_shoulder.visibility < min_vis:
        return False

    shoulder_width = abs(sample.l_shoulder.x - sample.r_shoulder.x)
    if shoulder_width < _MIN_SHOULDER_WIDTH:
        return False
    margin = frac * shoulder_width

    if sample.l_wrist.y >= sample.l_shoulder.y - margin:
        return False
    if sample.r_wrist.y >= sample.r_shoulder.y - margin:
        return False

    # Elbows, when visible, must not hang more than `margin` below the
    # shoulder. Rejects a landmark glitch that puts wrists "up" while the
    # arms hang; a box carried on the shoulder still has both wrists up
    # and elbows near the shoulders, which is acceptable.
    if sample.l_elbow.visibility >= min_vis:
        if sample.l_elbow.y > sample.l_shoulder.y + margin:
            return False
    if sample.r_elbow.visibility >= min_vis:
        if sample.r_elbow.y > sample.r_shoulder.y + margin:
            return False
    return True


# Timer comparisons tolerate float rounding: 1.4 - 1.0 is 0.39999999999999997,
# which must still satisfy a 0.4 s hold.
_TIME_EPS_S = 1e-6


class ArmsUpDetector:
    """Debounced both-arms-up state.

    ``active`` becomes True after ``arms_up`` has been continuously true
    for ``hold_s``, counting only NEW samples (distinct ``PoseSample.ts``).
    It becomes False after ``release_s`` of false/None, or when no new
    sample has arrived for ``stale_s``.
    """

    def __init__(self, cfg: Any = None) -> None:
        self._cfg = cfg
        self.active: bool = False
        self.raw: bool = False
        self.streak_s: float = 0.0
        self.last_sample_age_s: Optional[float] = None
        self.rising_edge: bool = False
        self._true_since_ts: Optional[float] = None
        self._false_since_now: Optional[float] = None
        self._last_seen_ts: Optional[float] = None
        self._last_new_sample_now: Optional[float] = None

    def update(self, sample: Optional[PoseSample], now: float) -> dict:
        """Advance debounce state. ``now`` is monotonic seconds."""
        self.rising_edge = False
        is_new_sample = False

        if sample is not None:
            self.raw = arms_up(sample, self._cfg)
            if sample.ts != self._last_seen_ts:
                is_new_sample = True
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
            self.active = False
            self._true_since_ts = None
            self.streak_s = 0.0
        elif is_new_sample:
            self._apply_observation(sample.ts if sample is not None else now, now, hold_s, release_s)
        elif sample is None:
            # None is not a timestamped sample; it still counts as false
            # for the release timer (wall clock).
            self._apply_false(now, release_s)

        return {
            "active": self.active,
            "raw": self.raw,
            "streak_s": self.streak_s,
            "last_sample_age_s": self.last_sample_age_s,
            "rising_edge": self.rising_edge,
        }

    def _apply_observation(self, sample_ts: float, now: float, hold_s: float, release_s: float) -> None:
        if self.raw:
            self._false_since_now = None
            if self._true_since_ts is None:
                self._true_since_ts = sample_ts
            self.streak_s = max(0.0, sample_ts - self._true_since_ts)
            if (not self.active) and self.streak_s >= hold_s - _TIME_EPS_S:
                self.active = True
                self.rising_edge = True
        else:
            self._apply_false(now, release_s)

    def _apply_false(self, now: float, release_s: float) -> None:
        self._true_since_ts = None
        self.streak_s = 0.0
        if self._false_since_now is None:
            self._false_since_now = now
        if self.active and (now - self._false_since_now) >= release_s - _TIME_EPS_S:
            self.active = False
