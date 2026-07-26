"""Tests for gating host-side MediaPipe Hands on arm state.

MediaPipe Hands runs on the Pi CPU on every poll iteration and is the most
expensive per-frame host consumer in the OAK poll loop. It competes directly
with the ~30 Hz control loop.

No gesture can produce an effect while the robot is disarmed:

  * GestureEvent.ACTIVATE is discarded unless ``safety_state.is_armed``
    (Controller.compute).
  * GestureEvent.DEACTIVATE only applies when mode == FOLLOW_ME, which itself
    can only be entered while armed.

Disarmed is most of WALL-E's uptime, so skipping the inference while disarmed
is free CPU with no behavior change. These tests cover the reader-side contract:

  1. The flag defaults True (standalone / CLI behavior unchanged).
  2. Disabling clears cached hand data, so a half-finished gesture sequence
     cannot survive across an arm cycle.
  3. Re-enabling does not resurrect the stale hand data.
  4. Toggling to the SAME value does not clear live data (no spurious wipes
     from the per-tick call in main.py).
  5. GestureStateMachine treats the resulting None as "no hand" and resets its
     hold counter, so a partial 3-4-3 sequence does not carry over.
"""

import sys
import types
import unittest

# Stub depthai — no OAK hardware in CI. Mirrors test_oak_reconnect.py.
if "depthai" not in sys.modules:
    _fake_dai = types.ModuleType("depthai")

    class _FakeDevice:
        @staticmethod
        def getAllAvailableDevices():
            return []

    _fake_dai.Device = _FakeDevice
    sys.modules["depthai"] = _fake_dai

from config import FollowMeConfig, ObstacleAvoidanceConfig
from pi_app.control.gesture_control import GestureStateMachine
from pi_app.hardware.oak_depth import OakDepthReader


def _make_reader() -> OakDepthReader:
    return OakDepthReader(
        obstacle_config=ObstacleAvoidanceConfig(),
        follow_me_config=FollowMeConfig(),
    )


class _StubHand:
    """Stand-in for HandData; the reader only stores and returns it."""


class TestHandPollGating(unittest.TestCase):
    def test_defaults_enabled(self):
        reader = _make_reader()
        self.assertTrue(reader._hand_poll_enabled)

    def test_disable_clears_cached_hand_data(self):
        reader = _make_reader()
        with reader._lock:
            reader._hand_state.hand_data = _StubHand()
        self.assertIsNotNone(reader.get_hand_data())

        reader.set_hand_poll_enabled(False)

        self.assertFalse(reader._hand_poll_enabled)
        self.assertIsNone(
            reader.get_hand_data(),
            "stale hand data must not survive a disarm — it could complete a "
            "gesture sequence on the next arm",
        )

    def test_reenable_does_not_resurrect_stale_data(self):
        reader = _make_reader()
        with reader._lock:
            reader._hand_state.hand_data = _StubHand()
        reader.set_hand_poll_enabled(False)
        reader.set_hand_poll_enabled(True)

        self.assertTrue(reader._hand_poll_enabled)
        self.assertIsNone(reader.get_hand_data())

    def test_repeated_enable_does_not_clear_live_data(self):
        """main.py calls this every tick; a no-op call must not wipe data."""
        reader = _make_reader()
        reader.set_hand_poll_enabled(True)
        hand = _StubHand()
        with reader._lock:
            reader._hand_state.hand_data = hand

        for _ in range(5):
            reader.set_hand_poll_enabled(True)

        self.assertIs(reader.get_hand_data(), hand)

    def test_repeated_disable_is_idempotent(self):
        reader = _make_reader()
        reader.set_hand_poll_enabled(False)
        for _ in range(5):
            reader.set_hand_poll_enabled(False)
        self.assertIsNone(reader.get_hand_data())
        self.assertFalse(reader._hand_poll_enabled)


class TestGestureMachineHandlesGatedNone(unittest.TestCase):
    """A gated-off reader yields None; the state machine must reset on it."""

    def test_none_resets_hold_counter(self):
        gsm = GestureStateMachine(
            activation_sequence=(3, 4, 3),
            stop_gesture="FIVE",
            hold_frames=3,
            sequence_timeout_s=3.0,
            cooldown_s=0.0,
        )
        # Feed None (what get_hand_data returns once gating is off) and confirm
        # it is inert: no event, and the hold counter does not accumulate.
        for _ in range(20):
            self.assertIsNone(gsm.update(None))
        self.assertEqual(gsm._consecutive, 0)


if __name__ == "__main__":
    unittest.main()
