"""Every FOLLOW_ME exit latches; an RC-stale link latches (Kevin, 2026-11-13).

Kevin's answers on 2026-11-13: "Yes, held low" (every FOLLOW_ME exit needs
the 0.2 s held low before a restart) and "Latch it" (an RC link that goes
stale must not re-arm by itself when it comes back with the switch up).

For each exit path: FOLLOW_ME runs; it ends; CH4 comes back high after one
low frame (a bounce, no held low) -> no restart; CH4 held low for
latch_clear_low_persist_s, then high -> restart.

Times are multiples of 1/16 s so the comparisons are exact.
"""

import unittest
from unittest.mock import patch

from config import FollowMeConfig
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.gesture_control import GestureEvent
from pi_app.control.rc_signal import RC_NO_SIGNAL_US as NS
from pi_app.control.safety import SafetyEvent, SafetyParams, SafetyState, force_disarm

DT = 1.0 / 16
HIGH, LOW, MID = 1900, 1000, 1500
PARAMS = SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0)
PERSON = PersonDetection(x_m=0.0, z_m=3.0, confidence=0.9, bbox=(0.45, 0.0, 0.55, 0.8))
HELD_LOW_TICKS = 5  # 0.25 s at DT, more than latch_clear_low_persist_s (0.2 s)


class _Motor:
    def __init__(self):
        self.status = {"latched": False, "restricted": False, "post_latch": False,
                       "reason": None, "voltage_v": 50.0, "voltage_state": "trusted_both"}

    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        pass

    def set_drive_context(self, armed, manual, request_neutral=None):
        pass

    def get_drive_limit_bytes(self):
        return None

    def get_pack_guard_status(self):
        return dict(self.status)


class _Relay:
    def set_armed(self, armed):
        pass


class _Shutdown:
    def __init__(self):
        self.scheduled = []

    def schedule_shutdown(self, delay_seconds):
        self.scheduled.append(delay_seconds)


class _Gesture:
    def __init__(self):
        self.next_event = None
        self.phase_name = "IDLE"

    @property
    def is_active(self):
        return False

    def update(self, hand):
        ev, self.next_event = self.next_event, None
        return ev

    def get_status(self):
        return {"phase": "IDLE"}

    def notify_external_deactivation(self):
        pass


class _Base(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": 10.0}
        self.vision_stale = False
        for target in ("pi_app.control.controller.time.monotonic",
                       "pi_app.control.follow_me.time.monotonic"):
            p = patch(target, side_effect=lambda: self.clock["t"])
            p.start()
            self.addCleanup(p.stop)
        self.motor = _Motor()
        self.gesture = _Gesture()
        self.shutdown = _Shutdown()
        self.c = Controller(
            motor_driver=self.motor, arm_relay=_Relay(), shutdown_scheduler=self.shutdown,
            safety_params=PARAMS, follow_me=FollowMeController(FollowMeConfig()),
            gesture_controller=self.gesture,
        )
        self.c.set_person_detections([PERSON])
        self.c.set_vision_freshness_getter(
            lambda: {"age_s": 9.0 if self.vision_stale else 0.1, "stale": self.vision_stale,
                     "seq": 1, "seq_stuck_packets": 0}
        )
        self.frame_t = None  # None: a fresh frame each tick

    def tick(self, ch3=HIGH, ch4=LOW, ch5=LOW, ch1=1500, n=1, bt=None):
        out = None
        for _ in range(n):
            self.clock["t"] += DT
            stamp = self.clock["t"] if self.frame_t is None else self.frame_t
            rc = RCInputs(ch1_us=ch1, ch2_us=ch1, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5,
                          last_update_mono_s=stamp)
            out = self.c.process(rc, now_epoch_s=self.clock["t"], bt_override_bytes=bt)
        return out

    @property
    def mode(self):
        return self.c._mode

    def start_follow_me(self):
        cmd, _, _ = self.tick(ch4=LOW, n=2)
        self.assertTrue(cmd.is_armed)
        self.tick(ch4=HIGH)
        self.assertEqual(self.mode, "FOLLOW_ME")

    def rearm(self):
        """ch3 held low (clears any re-arm latch), then high; CH4 stays up."""
        self.tick(ch3=LOW, ch4=HIGH, n=HELD_LOW_TICKS)
        cmd, _, _ = self.tick(ch3=HIGH, ch4=HIGH, n=2)
        self.assertTrue(cmd.is_armed)

    def assert_restart_needs_held_low(self):
        """The exit latched: a bounce does not restart; a held low does."""
        self.assertNotEqual(self.mode, "FOLLOW_ME")
        self.assertTrue(self.c._safety_state.ch4_requires_low)
        # CH4 up again after one low frame (no held low): no restart.
        self.tick(ch4=LOW)
        for _ in range(10):
            self.tick(ch4=HIGH)
            self.assertNotEqual(self.mode, "FOLLOW_ME")
        # Held low for 0.25 s, then up: restart.
        self.tick(ch4=LOW, n=HELD_LOW_TICKS)
        self.assertFalse(self.c._safety_state.ch4_requires_low)
        _, events, _ = self.tick(ch4=HIGH)
        self.assertIn(SafetyEvent.FOLLOW_ME_ENTERED, events)
        self.assertEqual(self.mode, "FOLLOW_ME")


class TestEveryFollowMeExitLatches(_Base):
    def test_ch4_low_one_frame(self):
        """Grok 8e79c28e: one 1100 us frame ended FOLLOW_ME and the next high
        frame restarted it."""
        self.start_follow_me()
        self.tick(ch4=1100)
        self.assertEqual(self.mode, "MANUAL")
        _, events, _ = self.tick(ch4=HIGH)
        self.assertNotIn(SafetyEvent.FOLLOW_ME_ENTERED, events)
        self.assert_restart_needs_held_low()

    def test_ch4_no_signal(self):
        self.start_follow_me()
        self.tick(ch4=NS)
        self.assert_restart_needs_held_low()

    def test_disarm_by_ch3_low(self):
        self.start_follow_me()
        cmd, _, _ = self.tick(ch3=LOW, ch4=HIGH)
        self.assertFalse(cmd.is_armed)
        self.rearm()
        self.assert_restart_needs_held_low()

    def test_disarm_by_ch3_no_signal(self):
        self.start_follow_me()
        self.tick(ch3=NS, ch4=HIGH)
        self.rearm()
        self.assert_restart_needs_held_low()

    def test_disarm_by_ch5_no_signal(self):
        self.start_follow_me()
        self.tick(ch5=NS, ch4=HIGH)
        self.rearm()
        self.assert_restart_needs_held_low()

    def test_midband_disarm(self):
        self.start_follow_me()
        cmd, events, _ = self.tick(ch3=MID, ch4=HIGH, n=10)
        self.assertFalse(cmd.is_armed)
        self.rearm()
        self.assert_restart_needs_held_low()

    def test_estop(self):
        self.start_follow_me()
        _, events, _ = self.tick(ch5=2000, ch4=HIGH)
        self.assertIn(SafetyEvent.EMERGENCY_TRIGGERED, events)
        self.assertEqual(self.mode, "MANUAL")
        self.assertTrue(self.c._safety_state.ch4_requires_low)
        self.assertEqual(self.shutdown.scheduled, [5.0])

    def test_vision_stale_exit(self):
        self.start_follow_me()
        self.vision_stale = True
        with self.assertLogs("pi_app.control.controller", level="WARNING"):
            _, _, telem = self.tick(ch4=HIGH)
        self.assertEqual(telem.get("follow_me_exit_reason"), "vision_stale")
        self.vision_stale = False
        self.assert_restart_needs_held_low()

    def test_pack_guard_exit(self):
        self.start_follow_me()
        self.motor.status["restricted"] = True
        _, _, telem = self.tick(ch4=HIGH)
        self.assertEqual(telem.get("follow_me_exit_reason"), "pack_guard")
        self.motor.status["restricted"] = False
        self.assert_restart_needs_held_low()

    def test_web_stop(self):
        self.start_follow_me()
        self.c.deactivate_follow_me()
        self.assert_restart_needs_held_low()

    def test_gesture_stop(self):
        self.start_follow_me()
        self.gesture.next_event = GestureEvent.DEACTIVATE
        self.tick(ch4=HIGH)
        self.assert_restart_needs_held_low()

    def test_calibration_wizard(self):
        self.start_follow_me()
        self.c.enter_calibration_mode()
        self.tick(ch4=HIGH, n=3)
        self.c.exit_calibration_mode()
        self.assert_restart_needs_held_low()

    def test_rc_stale(self):
        self.start_follow_me()
        self.frame_t = self.clock["t"]
        _, events, _ = self.tick(ch4=HIGH, n=20)  # 1.25 s with no new frame
        self.assertEqual(events, [SafetyEvent.RC_STALE])
        self.frame_t = None
        self.rearm()
        self.assert_restart_needs_held_low()

    def test_refused_start_no_target(self):
        cmd, _, _ = self.tick(ch4=LOW, n=2)
        self.c.set_person_detections([])
        _, _, telem = self.tick(ch4=HIGH)
        self.assertEqual(telem.get("follow_me_activation_blocked"), "no_target")
        self.c.set_person_detections([PERSON])
        self.assert_restart_needs_held_low()

    def test_refused_start_pack_guard(self):
        self.tick(ch4=LOW, n=2)
        self.motor.status["post_latch"] = True
        with self.assertLogs("pi_app.control.controller", level="WARNING"):
            _, _, telem = self.tick(ch4=HIGH)
        self.assertEqual(telem.get("follow_me_activation_blocked"), "pack_guard")
        self.motor.status["post_latch"] = False
        self.assert_restart_needs_held_low()

    def test_force_disarm_sets_the_latch(self):
        """The auto-disarm (force_disarm). FOLLOW_ME never auto-disarms in the
        controller, so this is the safety function on its own."""
        st = SafetyState(is_armed=True, follow_me_active=True)
        st2, events = force_disarm(st, 1.0)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertTrue(st2.ch4_requires_low and st2.rearm_requires_switch_cycle)

    def test_the_held_low_must_come_after_the_exit(self):
        """Grok 55dfba46: CH4 resting low for seconds, a web start (it does
        not check the latch), the CH4-low exit on the next tick. The low run
        that was already 0.2 s old must not clear the new latch on the tick
        after: the hold has to be measured after the exit."""
        self.tick(ch4=LOW, n=10)  # CH4 low for 0.625 s
        self.assertTrue(self.c.activate_follow_me())
        _, events, _ = self.tick(ch4=LOW)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertTrue(self.c._safety_state.ch4_requires_low)
        self.tick(ch4=LOW)  # one more low frame: not 0.2 s after the exit
        self.assertTrue(self.c._safety_state.ch4_requires_low)
        _, events, _ = self.tick(ch4=HIGH)
        self.assertNotIn(SafetyEvent.FOLLOW_ME_ENTERED, events)
        self.assertNotEqual(self.mode, "FOLLOW_ME")
        self.tick(ch4=LOW, n=HELD_LOW_TICKS)
        _, events, _ = self.tick(ch4=HIGH)
        self.assertIn(SafetyEvent.FOLLOW_ME_ENTERED, events)

    def test_web_and_gesture_starts_are_not_blocked(self):
        """The latch blocks a CH4 restart only. A start from the web UI or a
        gesture is an operator act."""
        self.start_follow_me()
        self.c.deactivate_follow_me()
        self.tick(ch4=HIGH)
        self.assertTrue(self.c.activate_follow_me())
        self.assertEqual(self.mode, "FOLLOW_ME")


class TestUpdateSafetyLatchesOnItsOwn(unittest.TestCase):
    """The safety function sets the latch itself, without the controller."""

    def _fm(self):
        from pi_app.control.safety import update_safety
        s = SafetyState(is_armed=True, follow_me_active=True, last_ch4_high=True,
                        ch3_held_high_mono_s=10.0, last_transition_mono_s=0.0)
        return update_safety, s

    def test_each_exit_in_update_safety(self):
        cases = {
            "ch4 low": dict(ch3_us=HIGH, ch4_us=1100, ch5_us=LOW),
            "ch4 no signal": dict(ch3_us=HIGH, ch4_us=NS, ch5_us=LOW),
            "ch3 low": dict(ch3_us=LOW, ch4_us=HIGH, ch5_us=LOW),
            "ch3 no signal": dict(ch3_us=NS, ch4_us=HIGH, ch5_us=LOW),
            "ch5 no signal": dict(ch3_us=HIGH, ch4_us=HIGH, ch5_us=NS),
            "e-stop": dict(ch3_us=HIGH, ch4_us=HIGH, ch5_us=2000),
        }
        for name, kw in cases.items():
            with self.subTest(name):
                update_safety, s = self._fm()
                s2, events = update_safety(s, now_epoch_s=10.0625, params=PARAMS,
                                           now_mono_s=10.0625, **kw)
                self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
                self.assertTrue(s2.ch4_requires_low)

    def test_midband_exit_in_update_safety(self):
        update_safety, s = self._fm()
        s2, events = update_safety(s, ch3_us=MID, ch4_us=HIGH, ch5_us=LOW, now_epoch_s=10.625,
                                   params=PARAMS, now_mono_s=10.625)
        self.assertIn(SafetyEvent.ARM_MIDBAND_DISARM, events)
        self.assertTrue(s2.ch4_requires_low)

    def test_staying_in_follow_me_does_not_latch(self):
        update_safety, s = self._fm()
        s2, events = update_safety(s, ch3_us=HIGH, ch4_us=HIGH, ch5_us=LOW, now_epoch_s=10.0625,
                                   params=PARAMS, now_mono_s=10.0625)
        self.assertEqual(events, [])
        self.assertTrue(s2.follow_me_active and not s2.ch4_requires_low)


class TestRcStaleLatches(_Base):
    def _arm_and_drop_link(self):
        cmd, _, _ = self.tick(n=2)
        self.assertTrue(cmd.is_armed)
        self.frame_t = self.clock["t"]
        cmd, events, _ = self.tick(n=20)  # 1.25 s: stale after 1.0 s
        self.assertFalse(cmd.is_armed)
        self.assertEqual(events, [SafetyEvent.RC_STALE])
        self.assertTrue(self.c._safety_state.rearm_requires_switch_cycle)
        self.frame_t = None

    def test_link_back_with_the_switch_up_does_not_rearm(self):
        self._arm_and_drop_link()
        for _ in range(40):  # 2.5 s, switch up the whole time
            cmd, events, telem = self.tick(ch3=HIGH)
            self.assertFalse(cmd.is_armed)
            self.assertNotIn(SafetyEvent.ARMED, events)
        self.assertTrue(telem["rearm_requires_switch_cycle"])
        # One bounced low frame is not a switch cycle.
        self.tick(ch3=LOW)
        cmd, _, _ = self.tick(ch3=HIGH, n=10)
        self.assertFalse(cmd.is_armed)
        # Held low, then high: arms.
        self.tick(ch3=LOW, n=HELD_LOW_TICKS)
        cmd, events, _ = self.tick(ch3=HIGH)
        self.assertTrue(cmd.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)

    def test_phone_cannot_drive_past_the_latch(self):
        """Phone teleop sends bt_override_bytes; process() forces neutral
        while disarmed, and nothing but update_safety arms."""
        self._arm_and_drop_link()
        for _ in range(20):
            cmd, _, _ = self.tick(ch3=HIGH, bt=(200, 200))
            self.assertFalse(cmd.is_armed)
            self.assertEqual((cmd.left_byte, cmd.right_byte), (Controller.NEUTRAL, Controller.NEUTRAL))

    def test_link_back_with_the_switch_low_needs_only_the_held_low(self):
        self._arm_and_drop_link()
        self.tick(ch3=LOW, n=HELD_LOW_TICKS)
        self.assertFalse(self.c._safety_state.rearm_requires_switch_cycle)
        cmd, _, _ = self.tick(ch3=HIGH)
        self.assertTrue(cmd.is_armed)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
