"""Controller armed-idle auto-disarm.

time.monotonic is patched for the whole case (same shape as the slew and
pack-low controller tests). process() and Controller.__init__ must not
gain extra monotonic calls; this clock is a lambda so a call-count change
does not exhaust a side_effect list.
"""
import unittest
from dataclasses import replace
from unittest.mock import MagicMock, patch

from pi_app.control.controller import RCInputs, Controller
from pi_app.control.mapping import map_pulse_to_byte_saturated
from pi_app.control.safety import SafetyEvent, SafetyParams
from config import SafetyConfig
from config import config as default_config


LIMIT_S = 600.0


class FakeMotor:
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        self.commands.append((left_byte, right_byte))

    def stop(self) -> None:
        self.stops += 1


class FakeRelay:
    def __init__(self):
        self.states = []

    def set_armed(self, armed: bool) -> None:
        self.states.append(armed)


class FakeShutdown:
    def __init__(self):
        self.scheduled = []

    def schedule_shutdown(self, delay_seconds: float) -> None:
        self.scheduled.append(delay_seconds)


class TestAutoDisarmController(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": 0.0}
        self._mono = patch(
            "pi_app.control.controller.time.monotonic",
            side_effect=lambda: self.clock["t"],
        )
        self._mono.start()
        self.motor = FakeMotor()
        self.relay = FakeRelay()
        self.shutdown = FakeShutdown()
        self.controller = Controller(
            motor_driver=self.motor,
            arm_relay=self.relay,
            shutdown_scheduler=self.shutdown,
            safety_params=SafetyParams(debounce_seconds=0.0),
        )

    def tearDown(self):
        self._mono.stop()

    def _tick(self, t, ch1=1500, ch2=1500, ch3=1900, ch4=1000, ch5=1000, bt=None):
        self.clock["t"] = float(t)
        rc = RCInputs(
            ch1_us=ch1,
            ch2_us=ch2,
            ch3_us=ch3,
            ch4_us=ch4,
            ch5_us=ch5,
            last_update_epoch_s=float(t),
        )
        return self.controller.process(rc, now_epoch_s=float(t), bt_override_bytes=bt)

    def _arm_idle(self):
        cmd, events, telem = self._tick(0.0)
        self.assertTrue(cmd.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)
        self.assertEqual(telem["armed_idle_s"], 0.0)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        return cmd

    def test_disarms_at_600s_latches_and_rearms_on_switch_cycle(self):
        self._arm_idle()
        follow = MagicMock()
        self.controller._follow_me = follow

        cmd, events, telem = self._tick(599.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 599.0)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        self.assertEqual(self.motor.stops, 0)

        # Set after the 599 s tick. ch4 is low, so leaving the flag set
        # across that tick would exit follow-me early and the 600 s tick
        # would not be the one that emits FOLLOW_ME_EXITED.
        self.controller._safety_state.follow_me_active = True
        stops_before = self.motor.stops
        relay_before = len(self.relay.states)
        with self.assertLogs("pi_app.control.controller", level="WARNING") as captured:
            cmd, events, telem = self._tick(600.0)
            # Still disarmed on the next ticks; the warning must not repeat.
            held, held_events, held_telem = self._tick(700.0)
            low, _, low_telem = self._tick(701.0, ch3=1000)
            rearmed, rearm_events, rearm_telem = self._tick(702.0)
            still, _, still_telem = self._tick(702.0 + 599.0)

        auto_lines = [line for line in captured.output if "AUTO-DISARM" in line]
        self.assertEqual(len(auto_lines), 1)
        self.assertIn(
            "AUTO-DISARM: armed and idle for 600 s; flip the arm switch off and on to re-arm",
            auto_lines[0],
        )

        self.assertFalse(cmd.is_armed)
        self.assertEqual(cmd.left_byte, 126)
        self.assertEqual(cmd.right_byte, 126)
        self.assertIn(SafetyEvent.DISARMED, events)
        self.assertIn(SafetyEvent.FOLLOW_ME_EXITED, events)
        self.assertGreater(self.motor.stops, stops_before)
        self.assertIn(False, self.relay.states[relay_before:])
        self.assertTrue(telem["rearm_requires_switch_cycle"])
        self.assertEqual(telem["armed_idle_s"], 0.0)
        self.assertEqual(self.shutdown.scheduled, [])
        follow.stop_recorder.assert_called()

        self.assertFalse(held.is_armed)
        self.assertNotIn(SafetyEvent.ARMED, held_events)
        self.assertTrue(held_telem["rearm_requires_switch_cycle"])

        self.assertFalse(low.is_armed)
        self.assertFalse(low_telem["rearm_requires_switch_cycle"])

        self.assertTrue(rearmed.is_armed)
        self.assertIn(SafetyEvent.ARMED, rearm_events)
        self.assertFalse(rearm_telem["rearm_requires_switch_cycle"])
        self.assertEqual(rearm_telem["armed_idle_s"], 0.0)

        # Timer started at the re-arm tick, not at the original arm.
        self.assertTrue(still.is_armed)
        self.assertEqual(still_telem["armed_idle_s"], 599.0)

    def test_stick_outside_deadband_restarts_timer(self):
        self._arm_idle()
        deadband = default_config.safety.auto_disarm_stick_deadband_us
        self._tick(300.0, ch1=1500 + deadband + 1)
        self._tick(301.0)  # sticks back to centre; timer starts here

        cmd, events, telem = self._tick(600.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 299.0)
        self.assertFalse(telem["rearm_requires_switch_cycle"])

        # 600 s after the restart, not after the original arm.
        cmd, events, telem = self._tick(301.0 + LIMIT_S)
        self.assertFalse(cmd.is_armed)
        self.assertIn(SafetyEvent.DISARMED, events)
        self.assertTrue(telem["rearm_requires_switch_cycle"])

    def test_stick_just_past_the_mapper_deadband_is_not_idle(self):
        # A stick 26 us off centre maps to a non-neutral byte, so it is
        # driving, not idle: it must not fire on the expiry tick.
        from pi_app.control.mapping import DEADBAND_US
        self.assertEqual(
            default_config.safety.auto_disarm_stick_deadband_us, DEADBAND_US
        )
        self._arm_idle()
        cmd, events, telem = self._tick(LIMIT_S, ch1=1500 + DEADBAND_US + 1)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 0.0)

    def test_slew_ramp_down_does_not_count_as_idle(self):
        # Drive, release the sticks, and step in small increments: while the
        # slew limiter is still ramping the final bytes toward neutral, the
        # idle timer must not run.
        self._arm_idle()
        for i in range(1, 30):
            self._tick(i * 0.02, ch1=2000, ch2=2000)
        t = 30 * 0.02
        saw_ramp = False
        for i in range(60):
            t += 0.02
            cmd, _events, telem = self._tick(t)
            if (cmd.left_byte, cmd.right_byte) != (126, 126):
                saw_ramp = True
                self.assertEqual(telem["armed_idle_s"], 0.0)
        self.assertTrue(saw_ramp, "the slew limiter never ramped; the test proves nothing")

    def test_follow_me_stationary_never_disarms(self):
        self._arm_idle()
        self._tick(599.0)
        self.controller._mode = "FOLLOW_ME"
        self.controller._safety_state.follow_me_active = True
        self.controller._safety_state.last_ch4_high = True

        for t in (600.0, 900.0, 1200.0):
            cmd, events, telem = self._tick(t, ch4=1900)
            self.assertTrue(cmd.is_armed, t)
            self.assertEqual(self.controller._mode, "FOLLOW_ME")
            self.assertNotIn(SafetyEvent.DISARMED, events)
            self.assertFalse(telem["rearm_requires_switch_cycle"])
            self.assertEqual(telem["armed_idle_s"], 0.0)

    def test_waypoint_nav_stationary_never_disarms(self):
        self._arm_idle()
        self._tick(599.0)
        self.controller._mode = "WAYPOINT_NAV"

        cmd, events, telem = self._tick(1200.0)
        self.assertTrue(cmd.is_armed)
        self.assertEqual(self.controller._mode, "WAYPOINT_NAV")
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        self.assertEqual(telem["armed_idle_s"], 0.0)

    def test_bt_override_resets_timer(self):
        self._arm_idle()
        self._tick(599.0)
        # Crossing the limit on a BT tick must not disarm. The timer clears
        # on that tick; the next centred tick starts it over.
        cmd, _, telem = self._tick(600.0, bt=(200, 200))
        self.assertTrue(cmd.is_armed)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        self.assertEqual(telem["armed_idle_s"], 0.0)

        cmd, events, telem = self._tick(601.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 0.0)

        cmd, events, telem = self._tick(601.0 + LIMIT_S - 1.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], LIMIT_S - 1.0)

    def test_calibration_mode_resets_timer(self):
        self._arm_idle()
        self._tick(599.0)
        self.clock["t"] = 600.0
        self.controller.enter_calibration_mode()
        cmd, events, telem = self._tick(600.0)
        self.assertTrue(cmd.is_armed)
        self.assertTrue(telem.get("calibration"))
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 0.0)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        self.assertIsNone(self.controller._armed_idle_since)

        cmd, _, telem = self._tick(2000.0)
        self.assertTrue(cmd.is_armed)
        self.assertEqual(telem["armed_idle_s"], 0.0)

        self.controller.exit_calibration_mode()
        cmd, events, telem = self._tick(2001.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertEqual(telem["armed_idle_s"], 0.0)

    def test_zero_limit_disables_auto_disarm(self):
        disabled = replace(default_config, safety=SafetyConfig(auto_disarm_idle_s=0.0))
        with patch("pi_app.control.controller.config", disabled):
            self._arm_idle()
            cmd, events, telem = self._tick(10000.0)
        self.assertTrue(cmd.is_armed)
        self.assertNotIn(SafetyEvent.DISARMED, events)
        self.assertFalse(telem["rearm_requires_switch_cycle"])

    def test_manual_drive_matches_mapping(self):
        # Sticks move every tick, so the idle timer never reaches the limit.
        # dt is 1 s; manual slew (250 byte/s accel, 350 byte/s decel) finishes
        # any full-range step in that interval, including the first snap.
        # The emitted bytes are the mapped stick command.
        self._arm_idle()
        f_full = default_config.rc_map.forward_full_us
        r_full = default_config.rc_map.reverse_full_us
        pairs = [(1800, 1700), (1200, 1300), (2100, 900), (1600, 1900)]
        for i, (ch1, ch2) in enumerate(pairs, start=1):
            cmd, _, telem = self._tick(float(i), ch1=ch1, ch2=ch2)
            expected = (
                map_pulse_to_byte_saturated(ch1, f_full, r_full),
                map_pulse_to_byte_saturated(ch2, f_full, r_full),
            )
            self.assertEqual((cmd.left_byte, cmd.right_byte), expected)
            self.assertEqual(self.motor.commands[-1], expected)
            self.assertTrue(cmd.is_armed)
            self.assertFalse(telem["rearm_requires_switch_cycle"])
            self.assertEqual(telem["armed_idle_s"], 0.0)

    def test_rc_stale_keeps_rearm_latch_and_reports_telemetry(self):
        self._arm_idle()
        self._tick(600.0)
        self.assertTrue(self.controller._safety_state.rearm_requires_switch_cycle)

        self.clock["t"] = 610.0
        stale = RCInputs(
            ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
            last_update_epoch_s=600.0,
        )
        cmd, events, telem = self.controller.process(stale, now_epoch_s=610.0)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertFalse(cmd.is_armed)
        self.assertTrue(telem["rearm_requires_switch_cycle"])
        self.assertEqual(telem["armed_idle_s"], 0.0)
        self.assertIsNone(self.controller._armed_idle_since)

        # Link returns with the switch still up. The latch holds.
        cmd, events, telem = self._tick(611.0)
        self.assertFalse(cmd.is_armed)
        self.assertNotIn(SafetyEvent.ARMED, events)
        self.assertTrue(telem["rearm_requires_switch_cycle"])


if __name__ == "__main__":
    unittest.main()
