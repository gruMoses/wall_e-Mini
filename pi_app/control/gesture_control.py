"""
Hand-gesture state machine for Follow Me activation/deactivation.

Consumes hand-landmark data produced by the OAK-D hand-tracking pipeline
and emits ACTIVATE / DEACTIVATE events based on a configurable finger-count
sequence (default 3-4-3 to start, open palm to stop).

All processing is trivial arithmetic on 21 landmark coordinates -- no NN
inference or image processing happens here.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from math import acos, degrees, sqrt
from typing import Optional, Sequence

logger = logging.getLogger(__name__)

# MediaPipe hand landmark indices (21 keypoints)
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 1, 2, 3, 4
INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 5, 6, 7, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP = 9, 10, 11, 12
RING_MCP, RING_PIP, RING_DIP, RING_TIP = 13, 14, 15, 16
PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP = 17, 18, 19, 20


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class HandData:
    """Hand landmark data received from the OAK-D pipeline."""
    norm_landmarks: list  # 21 × [x, y, z] in rotated bounding-box coords
    lm_score: float = 0.0


class GestureEvent(Enum):
    ACTIVATE = auto()
    DEACTIVATE = auto()


# Recognised gesture labels (subset matching depthai_hand_tracker)
GESTURE_LABELS = {
    "FIST", "ONE", "TWO", "THREE", "FOUR", "FIVE", "OK", "PEACE",
}

# Gesture label → finger count
_GESTURE_FINGER_COUNT = {
    "FIST": 0, "ONE": 1, "TWO": 2, "THREE": 3, "FOUR": 4, "FIVE": 5,
}


# ---------------------------------------------------------------------------
# Geometry helpers (pure-Python, no numpy dependency for Pi-side lightweight)
# ---------------------------------------------------------------------------

def _dist(a, b) -> float:
    return sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _angle_deg(a, b, c) -> float:
    """Angle at vertex *b* in degrees."""
    ba = [ai - bi for ai, bi in zip(a, b)]
    bc = [ci - bi for ci, bi in zip(c, b)]
    dot = sum(x * y for x, y in zip(ba, bc))
    mag_ba = sqrt(sum(x * x for x in ba))
    mag_bc = sqrt(sum(x * x for x in bc))
    denom = mag_ba * mag_bc
    if denom < 1e-9:
        return 0.0
    cos_val = max(-1.0, min(1.0, dot / denom))
    return degrees(acos(cos_val))


# ---------------------------------------------------------------------------
# Gesture recognition (ported from geaxgx/depthai_hand_tracker)
# ---------------------------------------------------------------------------

def recognize_gesture(hand: HandData) -> tuple[str | None, int]:
    """Classify the hand pose and return (gesture_label, finger_count).

    Returns (None, -1) when landmarks are insufficient or ambiguous.
    """
    lm = hand.norm_landmarks
    if lm is None or len(lm) < 21:
        return None, -1

    # -- Thumb state via joint angles + distance heuristic --
    d_3_5 = _dist(lm[THUMB_IP], lm[INDEX_MCP])
    d_2_3 = _dist(lm[THUMB_MCP], lm[THUMB_IP])
    angle0 = _angle_deg(lm[WRIST], lm[THUMB_CMC], lm[THUMB_MCP])
    angle1 = _angle_deg(lm[THUMB_CMC], lm[THUMB_MCP], lm[THUMB_IP])
    angle2 = _angle_deg(lm[THUMB_MCP], lm[THUMB_IP], lm[THUMB_TIP])
    thumb_angle = angle0 + angle1 + angle2
    if thumb_angle > 460 and (d_2_3 < 1e-9 or d_3_5 / d_2_3 > 1.2):
        thumb = 1
    else:
        thumb = 0

    # -- Finger states via joint angles (orientation-independent) --
    def _finger_open(tip, dip, pip, mcp) -> int:
        angle_pip = _angle_deg(lm[mcp], lm[pip], lm[dip])
        angle_dip = _angle_deg(lm[pip], lm[dip], lm[tip])
        extended = angle_pip > 120 and angle_dip > 120
        curled = angle_pip < 90 or angle_dip < 90
        if extended:
            return 1
        if curled:
            return 0
        return -1

    index = _finger_open(INDEX_TIP, INDEX_DIP, INDEX_PIP, INDEX_MCP)
    middle = _finger_open(MIDDLE_TIP, MIDDLE_DIP, MIDDLE_PIP, MIDDLE_MCP)
    ring = _finger_open(RING_TIP, RING_DIP, RING_PIP, RING_MCP)
    pinky = _finger_open(PINKY_TIP, PINKY_DIP, PINKY_PIP, PINKY_MCP)

    states = (thumb, index, middle, ring, pinky)

    # Count extended fingers directly (robust to any finger combination)
    open_count = sum(1 for s in states if s == 1)
    unknown_count = sum(1 for s in states if s == -1)

    # Only trust the count if at most 1 finger is ambiguous
    if unknown_count > 1:
        finger_count = -1
    else:
        finger_count = open_count

    _COUNT_TO_GESTURE = {0: "FIST", 1: "ONE", 2: "TWO", 3: "THREE", 4: "FOUR", 5: "FIVE"}
    gesture = _COUNT_TO_GESTURE.get(finger_count)

    logger.debug("GESTURE_DBG: states=%s fingers=%d", states, finger_count)
    return gesture, finger_count


# ---------------------------------------------------------------------------
# Gesture state machine
# ---------------------------------------------------------------------------

class _Phase(Enum):
    """Internal sequencing phases."""
    IDLE = auto()
    SEQ_0 = auto()  # first element of activation_sequence matched
    SEQ_1 = auto()  # second element matched
    ACTIVE = auto()  # Follow Me activated via gesture


class GestureStateMachine:
    """Detects activation/deactivation gesture sequences.

    Call ``update()`` every frame with the latest ``HandData``.
    Returns a ``GestureEvent`` when a transition fires, else ``None``.
    """

    def __init__(
        self,
        activation_sequence: Sequence[int] = (3, 4, 3),
        stop_gesture: str = "FIVE",
        hold_frames: int = 4,
        sequence_timeout_s: float = 3.0,
        cooldown_s: float = 2.0,
    ) -> None:
        self._seq = tuple(activation_sequence)
        self._stop_gesture = stop_gesture
        self._hold_frames = max(1, hold_frames)
        self._timeout_s = sequence_timeout_s
        self._cooldown_s = cooldown_s

        self._phase = _Phase.IDLE
        self._consecutive = 0  # frames current gesture has been held
        self._last_gesture: str | None = None
        self._last_finger_count: int = -1
        self._last_step_time: float = 0.0
        self._cooldown_until: float = 0.0
        self._awaiting_change: str | None = None  # gesture to ignore until it changes

    # -- Public API --

    @property
    def phase_name(self) -> str:
        return self._phase.name

    @property
    def is_active(self) -> bool:
        return self._phase is _Phase.ACTIVE

    def notify_external_deactivation(self) -> None:
        """Called when Follow Me is stopped via RC or web UI."""
        if self._phase is _Phase.ACTIVE:
            self._phase = _Phase.IDLE
            self._reset_hold()
            self._cooldown_until = time.monotonic() + self._cooldown_s

    def update(self, hand: HandData | None) -> GestureEvent | None:
        """Feed one frame of hand data; returns event or None."""
        now = time.monotonic()

        if now < self._cooldown_until:
            self._reset_hold()
            return None

        if hand is None:
            self._reset_hold()
            return None

        gesture, finger_count = recognize_gesture(hand)

        # Clear transition guard once the gesture changes
        if self._awaiting_change is not None and gesture != self._awaiting_change:
            self._awaiting_change = None

        # Debounce: track consecutive frames with same gesture
        if gesture == self._last_gesture:
            self._consecutive += 1
        else:
            self._consecutive = 1
            self._last_gesture = gesture
            self._last_finger_count = finger_count

        stable = self._consecutive >= self._hold_frames

        # --- ACTIVE phase: look for stop gesture ---
        if self._phase is _Phase.ACTIVE:
            if stable and gesture == self._stop_gesture:
                self._phase = _Phase.IDLE
                self._reset_hold()
                self._cooldown_until = now + self._cooldown_s
                logger.warning("Gesture: DEACTIVATE (stop gesture %s)", gesture)
                return GestureEvent.DEACTIVATE
            return None

        # --- Sequence matching (IDLE → SEQ_0 → SEQ_1 → ACTIVE) ---
        timed_out = (
            self._phase is not _Phase.IDLE
            and (now - self._last_step_time) > self._timeout_s
        )
        if timed_out:
            if self._phase is not _Phase.IDLE:
                logger.warning("SEQ: timed out in %s", self._phase.name)
            self._phase = _Phase.IDLE
            self._reset_hold()

        if not stable:
            return None

        # Don't advance or break while still showing the gesture that
        # triggered the previous step -- wait for the user to change.
        if self._awaiting_change is not None:
            return None

        target_idx = {
            _Phase.IDLE: 0,
            _Phase.SEQ_0: 1,
            _Phase.SEQ_1: 2,
        }.get(self._phase)

        if target_idx is None:
            return None

        if finger_count == self._seq[target_idx]:
            next_phases = [_Phase.SEQ_0, _Phase.SEQ_1, _Phase.ACTIVE]
            next_phase = next_phases[target_idx]
            self._phase = next_phase
            self._last_step_time = now
            self._awaiting_change = gesture
            self._reset_hold()
            logger.warning(
                "SEQ: step %d/%d matched (fingers=%d) -> %s",
                target_idx + 1, len(self._seq), finger_count, next_phase.name,
            )
            if next_phase is _Phase.ACTIVE:
                self._cooldown_until = now + self._cooldown_s
                self._awaiting_change = None
                logger.warning("Gesture: ACTIVATE (sequence %s completed)", self._seq)
                return GestureEvent.ACTIVATE
        else:
            if gesture is not None and self._phase is not _Phase.IDLE:
                logger.warning(
                    "SEQ: BROKEN by fingers=%d (expected %d) in %s",
                    finger_count, self._seq[target_idx], self._phase.name,
                )
                self._phase = _Phase.IDLE
                self._reset_hold()

        return None

    def _reset_hold(self) -> None:
        self._consecutive = 0
        self._last_gesture = None
        self._last_finger_count = -1
