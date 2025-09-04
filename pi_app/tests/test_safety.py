import unittest

from pi_app.control.safety import (
    SafetyParams,
    SafetyState,
    SafetyEvent,
    update_safety,
)


class TestSafety(unittest.TestCase):
    def test_arm_disarm_with_debounce(self):
        params = SafetyParams()
        s = SafetyState(is_armed=False, last_transition_epoch_s=0.0)
        now = 100.0

        # Below low threshold -> disarmed (already false, no event)
        s, events = update_safety(s, ch3_us=1100, ch5_us=1000, now_epoch_s=now, params=params)
        self.assertFalse(s.is_armed)
        self.assertEqual(events, [])

        # Jump above high threshold -> arm pending, but debounce required (first transition OK since last_transition=0)
        s, events = update_safety(s, ch3_us=1900, ch5_us=1000, now_epoch_s=now, params=params)
        # Debounce uses last_transition_epoch_s; initial 0 allows immediate change
        self.assertTrue(s.is_armed)
        self.assertEqual(events, [SafetyEvent.ARMED])

        # Try to disarm immediately within debounce window -> no change
        s, events = update_safety(s, ch3_us=1000, ch5_us=1000, now_epoch_s=now + 0.1, params=params)
        self.assertTrue(s.is_armed)
        self.assertEqual(events, [])

        # After debounce window -> disarm event
        s, events = update_safety(s, ch3_us=1000, ch5_us=1000, now_epoch_s=now + 0.4, params=params)
        self.assertFalse(s.is_armed)
        self.assertEqual(events, [SafetyEvent.DISARMED])

    def test_emergency_rising_edge_latches(self):
        params = SafetyParams()
        s = SafetyState(is_armed=True, last_transition_epoch_s=10.0, emergency_active=False, last_ch5_high=False)
        now = 20.0

        # Rising edge triggers emergency, forces disarm, emits events
        s, events = update_safety(s, ch3_us=1500, ch5_us=1900, now_epoch_s=now, params=params)
        self.assertTrue(s.emergency_active)
        self.assertFalse(s.is_armed)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertIn(SafetyEvent.DISARMED, events)

        # Subsequent arm attempts do nothing while emergency is active
        s2, events = update_safety(s, ch3_us=1900, ch5_us=1500, now_epoch_s=now + 1.0, params=params)
        self.assertTrue(s2.emergency_active)
        self.assertFalse(s2.is_armed)
        self.assertEqual(events, [])

    def test_ch3_midband_holds_state(self):
        params = SafetyParams()
        s = SafetyState(is_armed=False, last_transition_epoch_s=0.0)
        now = 0.0
        # Midband should hold current state
        s, events = update_safety(s, ch3_us=1500, ch5_us=1000, now_epoch_s=now, params=params)
        self.assertFalse(s.is_armed)
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()


