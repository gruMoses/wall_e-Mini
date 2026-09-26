"""Auto-disarm latch in update_safety / force_disarm.

2026-09-24 the robot stayed armed for 23 hours because ch3 is level-based:
a software disarm with the switch still up re-arms after the debounce.
force_disarm latches until ch3 is seen at or below arm_low_threshold_us.
"""
import unittest

from pi_app.control.safety import (
    SafetyEvent,
    SafetyParams,
    SafetyState,
    force_disarm,
    update_safety,
)


CH3_LOW = 1000
CH3_MID = 1500
CH3_HIGH = 1900
CH4_LOW = 1000
CH5_LOW = 1000
CH5_HIGH = 1900


def _step(state, ch3, now, params=None, ch4=CH4_LOW, ch5=CH5_LOW):
    return update_safety(
        state,
        ch3_us=ch3,
        ch4_us=ch4,
        ch5_us=ch5,
        now_epoch_s=now,
        params=params or SafetyParams(),
    )


class TestForceDisarm(unittest.TestCase):
    def test_armed_state_disarms_latches_and_emits_disarmed(self):
        state = SafetyState(
            is_armed=True,
            last_transition_epoch_s=10.0,
            last_ch5_high=False,
            last_ch4_high=True,
            emergency_active=False,
            follow_me_active=False,
        )
        new_state, events = force_disarm(state, 50.0)

        self.assertTrue(state.is_armed)  # input is not mutated
        self.assertFalse(new_state.is_armed)
        self.assertTrue(new_state.rearm_requires_switch_cycle)
        self.assertEqual(new_state.last_transition_epoch_s, 50.0)
        self.assertFalse(new_state.emergency_active)
        self.assertFalse(new_state.last_ch5_high)
        self.assertTrue(new_state.last_ch4_high)
        self.assertEqual(events, [SafetyEvent.DISARMED])

    def test_follow_me_active_exits_and_disarms(self):
        state = SafetyState(is_armed=True, follow_me_active=True, emergency_active=True)
        new_state, events = force_disarm(state, 12.0)

        self.assertFalse(new_state.is_armed)
        self.assertFalse(new_state.follow_me_active)
        self.assertTrue(new_state.rearm_requires_switch_cycle)
        self.assertTrue(new_state.emergency_active)  # latch untouched
        self.assertEqual(
            events,
            [SafetyEvent.FOLLOW_ME_EXITED, SafetyEvent.DISARMED],
        )

    def test_already_disarmed_emits_no_disarmed_event(self):
        state = SafetyState(is_armed=False, follow_me_active=False, last_transition_epoch_s=1.0)
        new_state, events = force_disarm(state, 4.0)

        self.assertFalse(new_state.is_armed)
        self.assertTrue(new_state.rearm_requires_switch_cycle)
        self.assertEqual(new_state.last_transition_epoch_s, 4.0)
        self.assertEqual(events, [])

    def test_disarmed_with_follow_me_exits_without_disarmed_event(self):
        state = SafetyState(is_armed=False, follow_me_active=True)
        new_state, events = force_disarm(state, 4.0)

        self.assertFalse(new_state.follow_me_active)
        self.assertEqual(events, [SafetyEvent.FOLLOW_ME_EXITED])


class TestRearmLatch(unittest.TestCase):
    def test_ch3_held_high_stays_disarmed_past_debounce(self):
        params = SafetyParams()  # debounce 0.3 s
        state, _ = force_disarm(
            SafetyState(is_armed=True, last_transition_epoch_s=0.0),
            100.0,
        )
        self.assertTrue(state.rearm_requires_switch_cycle)

        for dt in (0.05, 0.3, 0.31, 1.0, 5.0, 30.0):
            state, events = _step(state, CH3_HIGH, 100.0 + dt, params)
            self.assertFalse(state.is_armed, dt)
            self.assertTrue(state.rearm_requires_switch_cycle, dt)
            self.assertNotIn(SafetyEvent.ARMED, events)
            self.assertNotIn(SafetyEvent.DISARMED, events)

    def test_ch3_low_clears_latch_then_high_arms_after_debounce(self):
        params = SafetyParams()
        state, _ = force_disarm(SafetyState(is_armed=True), 0.0)

        state, events = _step(state, CH3_LOW, 0.05, params)
        self.assertFalse(state.rearm_requires_switch_cycle)
        self.assertFalse(state.is_armed)
        self.assertEqual(events, [])

        # Still inside the 0.3 s debounce measured from force_disarm.
        state, events = _step(state, CH3_HIGH, 0.10, params)
        self.assertFalse(state.is_armed)
        self.assertEqual(events, [])

        state, events = _step(state, CH3_HIGH, 0.30, params)
        self.assertTrue(state.is_armed)
        self.assertEqual(events, [SafetyEvent.ARMED])
        self.assertFalse(state.rearm_requires_switch_cycle)

    def test_low_threshold_exactly_clears_latch(self):
        params = SafetyParams()
        state, _ = force_disarm(SafetyState(is_armed=True), 0.0)
        state, _ = _step(state, params.arm_low_threshold_us, 1.0, params)
        self.assertFalse(state.rearm_requires_switch_cycle)
        self.assertFalse(state.is_armed)

    def test_between_thresholds_does_not_clear_latch(self):
        params = SafetyParams()
        state, _ = force_disarm(SafetyState(is_armed=True), 0.0)

        for ch3 in (params.arm_low_threshold_us + 1, CH3_MID, params.arm_high_threshold_us - 1):
            state, events = _step(state, ch3, 5.0, params)
            self.assertTrue(state.rearm_requires_switch_cycle, ch3)
            self.assertFalse(state.is_armed, ch3)
            self.assertEqual(events, [])

        # High after a mid-band sample still cannot arm.
        state, events = _step(state, CH3_HIGH, 10.0, params)
        self.assertTrue(state.rearm_requires_switch_cycle)
        self.assertFalse(state.is_armed)
        self.assertNotIn(SafetyEvent.ARMED, events)

    def test_emergency_rising_edge_keeps_latch(self):
        params = SafetyParams()
        state = SafetyState(
            is_armed=True,
            rearm_requires_switch_cycle=True,
            follow_me_active=True,
            last_ch5_high=False,
            emergency_active=False,
            last_transition_epoch_s=0.0,
        )
        state, events = _step(state, CH3_HIGH, 5.0, params, ch5=CH5_HIGH)

        self.assertTrue(state.emergency_active)
        self.assertTrue(state.rearm_requires_switch_cycle)
        self.assertFalse(state.is_armed)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertIn(SafetyEvent.DISARMED, events)

    def test_emergency_already_latched_keeps_rearm_latch_even_if_ch3_low(self):
        # The already-active early return must not clear the rearm latch.
        # ch3 low would clear it if that code ran.
        state = SafetyState(
            is_armed=False,
            emergency_active=True,
            rearm_requires_switch_cycle=True,
            last_ch5_high=True,
        )
        state, events = _step(state, CH3_LOW, 8.0, ch5=CH5_LOW)
        self.assertTrue(state.emergency_active)
        self.assertTrue(state.rearm_requires_switch_cycle)
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()
