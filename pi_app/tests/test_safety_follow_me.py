import unittest

from pi_app.control.safety import (
    SafetyParams,
    SafetyState,
    SafetyEvent,
    update_safety,
)


class TestSafetyFollowMe(unittest.TestCase):
    """Tests for the Follow Me tap-detection logic in update_safety()."""

    def _params(self, **overrides) -> SafetyParams:
        defaults = dict(
            debounce_seconds=0.0,  # instant transitions for easier testing
            follow_me_tap_window_s=2.0,
            follow_me_tap_count=4,
        )
        defaults.update(overrides)
        return SafetyParams(**defaults)

    def _arm_disarm_cycle(self, state, t, params):
        """Simulate one arm->disarm tap starting at time t.

        Returns (new_state, all_events, time_after_cycle).
        """
        all_events = []
        # Arm
        state, events = update_safety(state, ch3_us=1900, ch5_us=1000,
                                      now_epoch_s=t, params=params)
        all_events.extend(events)
        # Disarm (0.1s later)
        state, events = update_safety(state, ch3_us=1000, ch5_us=1000,
                                      now_epoch_s=t + 0.1, params=params)
        all_events.extend(events)
        return state, all_events, t + 0.2

    def test_four_taps_enters_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0
        all_events = []
        for _ in range(4):
            state, events, t = self._arm_disarm_cycle(state, t, params)
            all_events.extend(events)

        self.assertTrue(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_ENTERED, all_events)

    def test_three_taps_does_not_enter(self):
        params = self._params()
        state = SafetyState()
        t = 10.0
        all_events = []
        for _ in range(3):
            state, events, t = self._arm_disarm_cycle(state, t, params)
            all_events.extend(events)

        self.assertFalse(state.follow_me_active)
        self.assertNotIn(SafetyEvent.FOLLOW_ME_ENTERED, all_events)

    def test_taps_outside_window_do_not_enter(self):
        params = self._params(follow_me_tap_window_s=1.0)
        state = SafetyState()
        t = 10.0
        # First two taps
        for _ in range(2):
            state, _, t = self._arm_disarm_cycle(state, t, params)
        # Wait beyond window
        t += 2.0
        # Two more taps (total 4, but only 2 within window)
        for _ in range(2):
            state, _, t = self._arm_disarm_cycle(state, t, params)

        self.assertFalse(state.follow_me_active)

    def test_single_disarm_exits_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Enter Follow Me
        for _ in range(4):
            state, _, t = self._arm_disarm_cycle(state, t, params)
        self.assertTrue(state.follow_me_active)

        # Arm
        state, _, = update_safety(state, ch3_us=1900, ch5_us=1000,
                                  now_epoch_s=t, params=params)
        t += 0.1
        # Single disarm -> exit
        state, events = update_safety(state, ch3_us=1000, ch5_us=1000,
                                      now_epoch_s=t, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertIn(SafetyEvent.DISARMED, events)

    def test_emergency_exits_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Enter Follow Me
        for _ in range(4):
            state, _, t = self._arm_disarm_cycle(state, t, params)
        self.assertTrue(state.follow_me_active)

        # Arm up
        state, _ = update_safety(state, ch3_us=1900, ch5_us=1000,
                                 now_epoch_s=t, params=params)
        t += 0.1

        # Emergency while in Follow Me
        state, events = update_safety(state, ch3_us=1900, ch5_us=1900,
                                      now_epoch_s=t, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)

    def test_tap_counter_resets_after_entering(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Enter Follow Me with 4 taps
        for _ in range(4):
            state, _, t = self._arm_disarm_cycle(state, t, params)
        self.assertTrue(state.follow_me_active)
        self.assertEqual(state.tap_timestamps, [])

    def test_existing_arm_disarm_still_works(self):
        """Verify normal arm/disarm without taps doesn't break."""
        params = self._params()
        state = SafetyState()

        state, events = update_safety(state, ch3_us=1900, ch5_us=1000,
                                      now_epoch_s=1.0, params=params)
        self.assertTrue(state.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)

        state, events = update_safety(state, ch3_us=1000, ch5_us=1000,
                                      now_epoch_s=2.0, params=params)
        self.assertFalse(state.is_armed)
        self.assertIn(SafetyEvent.DISARMED, events)
        self.assertFalse(state.follow_me_active)


if __name__ == "__main__":
    unittest.main()
