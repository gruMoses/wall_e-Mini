import unittest

from pi_app.control.safety import (
    SafetyParams,
    SafetyState,
    SafetyEvent,
    update_safety,
)

CH4_LOW = 1000
CH4_HIGH = 1900


class TestSafetyFollowMe(unittest.TestCase):
    """Tests for ch4-based Follow Me activation in update_safety()."""

    def _params(self, **overrides) -> SafetyParams:
        defaults = dict(
            debounce_seconds=0.0,
            follow_me_high_threshold_us=1800,
            follow_me_low_threshold_us=1200,
        )
        defaults.update(overrides)
        return SafetyParams(**defaults)

    def test_ch4_high_while_armed_enters_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Arm
        state, events = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                      now_epoch_s=t, params=params)
        self.assertTrue(state.is_armed)
        self.assertFalse(state.follow_me_active)

        # Ch4 goes high -> enter Follow Me
        state, events = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1000,
                                      now_epoch_s=t + 0.1, params=params)
        self.assertTrue(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_ENTERED, events)

    def test_ch4_high_while_disarmed_does_not_enter(self):
        params = self._params()
        state = SafetyState()

        state, events = update_safety(state, ch3_us=1000, ch4_us=CH4_HIGH, ch5_us=1000,
                                      now_epoch_s=1.0, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertNotIn(SafetyEvent.FOLLOW_ME_ENTERED, events)

    def test_ch4_low_exits_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Arm
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                 now_epoch_s=t, params=params)
        # Enter Follow Me
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1000,
                                 now_epoch_s=t + 0.1, params=params)
        self.assertTrue(state.follow_me_active)

        # Ch4 low -> exit
        state, events = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                      now_epoch_s=t + 0.2, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)

    def test_disarm_exits_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Arm + enter Follow Me
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                 now_epoch_s=t, params=params)
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1000,
                                 now_epoch_s=t + 0.1, params=params)
        self.assertTrue(state.follow_me_active)

        # Disarm -> Follow Me exits
        state, events = update_safety(state, ch3_us=1000, ch4_us=CH4_HIGH, ch5_us=1000,
                                      now_epoch_s=t + 0.2, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertIn(SafetyEvent.DISARMED, events)

    def test_emergency_exits_follow_me(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Arm + enter Follow Me
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                 now_epoch_s=t, params=params)
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1000,
                                 now_epoch_s=t + 0.1, params=params)
        self.assertTrue(state.follow_me_active)

        # Emergency
        state, events = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1900,
                                      now_epoch_s=t + 0.2, params=params)
        self.assertFalse(state.follow_me_active)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)

    def test_ch4_midband_holds_state(self):
        params = self._params()
        state = SafetyState()
        t = 10.0

        # Arm + enter Follow Me
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                 now_epoch_s=t, params=params)
        state, _ = update_safety(state, ch3_us=1900, ch4_us=CH4_HIGH, ch5_us=1000,
                                 now_epoch_s=t + 0.1, params=params)
        self.assertTrue(state.follow_me_active)

        # Ch4 in midband (between thresholds) -> state holds
        state, events = update_safety(state, ch3_us=1900, ch4_us=1500, ch5_us=1000,
                                      now_epoch_s=t + 0.2, params=params)
        self.assertTrue(state.follow_me_active)
        self.assertNotIn(SafetyEvent.FOLLOW_ME_EXITED, events)

    def test_existing_arm_disarm_still_works(self):
        """Verify normal arm/disarm without ch4 doesn't break."""
        params = self._params()
        state = SafetyState()

        state, events = update_safety(state, ch3_us=1900, ch4_us=CH4_LOW, ch5_us=1000,
                                      now_epoch_s=1.0, params=params)
        self.assertTrue(state.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)

        state, events = update_safety(state, ch3_us=1000, ch4_us=CH4_LOW, ch5_us=1000,
                                      now_epoch_s=2.0, params=params)
        self.assertFalse(state.is_armed)
        self.assertIn(SafetyEvent.DISARMED, events)
        self.assertFalse(state.follow_me_active)


if __name__ == "__main__":
    unittest.main()
