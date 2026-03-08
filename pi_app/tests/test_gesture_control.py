"""Tests for the hand-gesture state machine and finger-counting logic."""

import time
import unittest
from unittest.mock import patch

from pi_app.control.gesture_control import (
    GestureEvent,
    GestureStateMachine,
    HandData,
    recognize_gesture,
)


# ---------------------------------------------------------------------------
# Helpers — synthetic landmark sets for known poses
# ---------------------------------------------------------------------------

def _make_fist_landmarks() -> list:
    """All fingers curled: all tips below their MCP joints."""
    lm = [[0.5, 0.5, 0.0]] * 21
    # Thumb: angles sum < 460 → closed
    lm[0] = [0.3, 0.8, 0]   # WRIST
    lm[1] = [0.35, 0.7, 0]  # THUMB_CMC
    lm[2] = [0.38, 0.6, 0]  # THUMB_MCP
    lm[3] = [0.39, 0.55, 0] # THUMB_IP
    lm[4] = [0.38, 0.58, 0] # THUMB_TIP (curled back)
    # Index: tip(8) below MCP(5) → closed
    lm[5] = [0.4, 0.45, 0]  # INDEX_MCP
    lm[6] = [0.42, 0.5, 0]
    lm[7] = [0.43, 0.55, 0]
    lm[8] = [0.42, 0.6, 0]  # INDEX_TIP (below MCP)
    # Middle
    lm[9] = [0.45, 0.44, 0]
    lm[10] = [0.46, 0.5, 0]
    lm[11] = [0.47, 0.55, 0]
    lm[12] = [0.46, 0.6, 0]
    # Ring
    lm[13] = [0.5, 0.44, 0]
    lm[14] = [0.51, 0.5, 0]
    lm[15] = [0.52, 0.55, 0]
    lm[16] = [0.51, 0.6, 0]
    # Pinky
    lm[17] = [0.55, 0.45, 0]
    lm[18] = [0.56, 0.5, 0]
    lm[19] = [0.57, 0.55, 0]
    lm[20] = [0.56, 0.6, 0]
    return lm


def _make_open_hand_landmarks() -> list:
    """All five fingers extended (tips above PIPs above MCPs)."""
    lm = [[0.5, 0.5, 0.0]] * 21
    lm[0] = [0.3, 0.9, 0]   # WRIST
    lm[1] = [0.2, 0.7, 0]   # THUMB_CMC
    lm[2] = [0.15, 0.5, 0]  # THUMB_MCP
    lm[3] = [0.1, 0.35, 0]  # THUMB_IP
    lm[4] = [0.05, 0.2, 0]  # THUMB_TIP
    lm[5] = [0.3, 0.4, 0]   # INDEX_MCP
    lm[6] = [0.3, 0.3, 0]   # INDEX_PIP
    lm[7] = [0.3, 0.2, 0]   # INDEX_DIP
    lm[8] = [0.3, 0.1, 0]   # INDEX_TIP
    lm[9] = [0.4, 0.38, 0]
    lm[10] = [0.4, 0.28, 0]
    lm[11] = [0.4, 0.18, 0]
    lm[12] = [0.4, 0.08, 0]
    lm[13] = [0.5, 0.4, 0]
    lm[14] = [0.5, 0.3, 0]
    lm[15] = [0.5, 0.2, 0]
    lm[16] = [0.5, 0.1, 0]
    lm[17] = [0.6, 0.42, 0]
    lm[18] = [0.6, 0.32, 0]
    lm[19] = [0.6, 0.22, 0]
    lm[20] = [0.6, 0.12, 0]
    return lm


def _make_three_fingers_landmarks() -> list:
    """Thumb + index + middle extended (THREE gesture)."""
    lm = _make_open_hand_landmarks()
    # Close ring and pinky (tip below MCP)
    lm[13] = [0.5, 0.4, 0]   # RING_MCP
    lm[14] = [0.51, 0.45, 0]
    lm[15] = [0.52, 0.5, 0]
    lm[16] = [0.51, 0.55, 0] # RING_TIP below MCP
    lm[17] = [0.6, 0.42, 0]  # PINKY_MCP
    lm[18] = [0.61, 0.47, 0]
    lm[19] = [0.62, 0.52, 0]
    lm[20] = [0.61, 0.57, 0] # PINKY_TIP below MCP
    return lm


def _make_four_fingers_landmarks() -> list:
    """Index + middle + ring + pinky extended, thumb closed (FOUR gesture)."""
    lm = _make_open_hand_landmarks()
    # Close thumb (make angles sum < 460)
    lm[1] = [0.28, 0.75, 0]
    lm[2] = [0.3, 0.65, 0]
    lm[3] = [0.32, 0.6, 0]
    lm[4] = [0.33, 0.62, 0]  # TIP curled back
    return lm


def _hand(landmarks: list) -> HandData:
    return HandData(norm_landmarks=landmarks, lm_score=0.9)


# ---------------------------------------------------------------------------
# Tests: recognize_gesture
# ---------------------------------------------------------------------------

class TestRecognizeGesture(unittest.TestCase):

    def test_fist(self):
        gesture, count = recognize_gesture(_hand(_make_fist_landmarks()))
        self.assertEqual(gesture, "FIST")
        self.assertEqual(count, 0)

    def test_five(self):
        gesture, count = recognize_gesture(_hand(_make_open_hand_landmarks()))
        self.assertEqual(gesture, "FIVE")
        self.assertEqual(count, 5)

    def test_three(self):
        gesture, count = recognize_gesture(_hand(_make_three_fingers_landmarks()))
        self.assertEqual(gesture, "THREE")
        self.assertEqual(count, 3)

    def test_four(self):
        gesture, count = recognize_gesture(_hand(_make_four_fingers_landmarks()))
        self.assertEqual(gesture, "FOUR")
        self.assertEqual(count, 4)

    def test_none_on_empty(self):
        gesture, count = recognize_gesture(HandData(norm_landmarks=None))
        self.assertIsNone(gesture)
        self.assertEqual(count, -1)

    def test_none_on_short_landmarks(self):
        gesture, count = recognize_gesture(HandData(norm_landmarks=[[0, 0, 0]] * 5))
        self.assertIsNone(gesture)
        self.assertEqual(count, -1)


# ---------------------------------------------------------------------------
# Tests: GestureStateMachine
# ---------------------------------------------------------------------------

class TestGestureStateMachine(unittest.TestCase):

    def _make_sm(self, **kw) -> GestureStateMachine:
        defaults = dict(
            activation_sequence=(3, 4, 3),
            stop_gesture="FIVE",
            hold_frames=2,
            sequence_timeout_s=3.0,
            cooldown_s=0.5,
        )
        defaults.update(kw)
        return GestureStateMachine(**defaults)

    def _feed(self, sm: GestureStateMachine, landmarks: list, n: int = 1):
        """Feed *n* frames of the same gesture. Return last event."""
        ev = None
        for _ in range(n):
            ev = sm.update(_hand(landmarks))
        return ev

    # -- Full activation sequence --

    def test_full_activation_sequence(self):
        sm = self._make_sm(hold_frames=2, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()
        four = _make_four_fingers_landmarks()

        ev = self._feed(sm, three, 2)
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "SEQ_0")

        ev = self._feed(sm, four, 2)
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "SEQ_1")

        ev = self._feed(sm, three, 2)
        self.assertEqual(ev, GestureEvent.ACTIVATE)
        self.assertTrue(sm.is_active)

    # -- Stop gesture --

    def test_stop_after_activation(self):
        sm = self._make_sm(hold_frames=2, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()
        four = _make_four_fingers_landmarks()
        five = _make_open_hand_landmarks()

        self._feed(sm, three, 2)
        self._feed(sm, four, 2)
        self._feed(sm, three, 2)
        self.assertTrue(sm.is_active)

        ev = self._feed(sm, five, 2)
        self.assertEqual(ev, GestureEvent.DEACTIVATE)
        self.assertFalse(sm.is_active)

    # -- Debounce --

    def test_single_frame_not_enough(self):
        sm = self._make_sm(hold_frames=3)
        three = _make_three_fingers_landmarks()
        ev = self._feed(sm, three, 2)
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "IDLE")

    # -- Sequence timeout --

    @patch("pi_app.control.gesture_control.time")
    def test_sequence_timeout_resets(self, mock_time):
        t = 100.0
        mock_time.monotonic.return_value = t

        sm = self._make_sm(hold_frames=1, sequence_timeout_s=2.0, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()
        four = _make_four_fingers_landmarks()

        sm.update(_hand(three))
        self.assertEqual(sm.phase_name, "SEQ_0")

        # Jump beyond timeout
        t += 5.0
        mock_time.monotonic.return_value = t

        ev = sm.update(_hand(four))
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "IDLE")

    # -- Wrong gesture breaks sequence --

    def test_wrong_gesture_resets(self):
        sm = self._make_sm(hold_frames=1, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()
        fist = _make_fist_landmarks()

        sm.update(_hand(three))
        self.assertEqual(sm.phase_name, "SEQ_0")

        sm.update(_hand(fist))
        sm.update(_hand(fist))
        self.assertEqual(sm.phase_name, "IDLE")

    # -- None hand data --

    def test_none_hand_resets_hold(self):
        sm = self._make_sm(hold_frames=2, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()

        sm.update(_hand(three))
        ev = sm.update(None)
        self.assertIsNone(ev)
        # Consecutive count was reset; next three should NOT complete a hold
        ev = sm.update(_hand(three))
        self.assertIsNone(ev)
        self.assertEqual(sm.phase_name, "IDLE")

    # -- Cooldown --

    @patch("pi_app.control.gesture_control.time")
    def test_cooldown_ignores_gestures(self, mock_time):
        t = 100.0
        mock_time.monotonic.return_value = t

        sm = self._make_sm(hold_frames=1, cooldown_s=2.0)
        three = _make_three_fingers_landmarks()
        four = _make_four_fingers_landmarks()

        sm.update(_hand(three))
        sm.update(_hand(four))
        ev = sm.update(_hand(three))
        self.assertEqual(ev, GestureEvent.ACTIVATE)

        # Immediately after activation, gestures should be ignored (cooldown)
        t += 0.5
        mock_time.monotonic.return_value = t
        five = _make_open_hand_landmarks()
        ev = sm.update(_hand(five))
        self.assertIsNone(ev)
        self.assertTrue(sm.is_active)

        # After cooldown expires, stop gesture should work on the first stable frame
        t += 2.0
        mock_time.monotonic.return_value = t
        ev = sm.update(_hand(five))
        self.assertEqual(ev, GestureEvent.DEACTIVATE)
        self.assertFalse(sm.is_active)

    # -- External deactivation --

    def test_external_deactivation(self):
        sm = self._make_sm(hold_frames=1, cooldown_s=0.0)
        three = _make_three_fingers_landmarks()
        four = _make_four_fingers_landmarks()

        sm.update(_hand(three))
        sm.update(_hand(four))
        sm.update(_hand(three))
        self.assertTrue(sm.is_active)

        sm.notify_external_deactivation()
        self.assertFalse(sm.is_active)
        self.assertEqual(sm.phase_name, "IDLE")


if __name__ == "__main__":
    unittest.main()
