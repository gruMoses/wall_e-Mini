"""Startup arm interlock: a fresh Controller arms only after the arm switch
has been seen OFF (2026-09-25, follow-up to the overnight-armed incident).

Without it, a service restart after an auto-disarm (a deploy, a crash)
would re-arm on an arm switch that was left up. main.py enables it from
SafetyConfig.require_switch_off_at_startup; a Controller built directly
keeps the old behaviour.
"""

import unittest
from unittest.mock import patch

from config import SafetyConfig
from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.safety import SafetyEvent, SafetyParams


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
    def schedule_shutdown(self, delay_seconds: float) -> None:
        raise AssertionError("the interlock must never schedule a shutdown")


class TestArmInterlock(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": 0.0}
        self._mono = patch(
            "pi_app.control.controller.time.monotonic",
            side_effect=lambda: self.clock["t"],
        )
        self._mono.start()
        self.motor = FakeMotor()
        self.relay = FakeRelay()

    def tearDown(self):
        self._mono.stop()

    def _make(self, startup_interlock):
        return Controller(
            motor_driver=self.motor,
            arm_relay=self.relay,
            shutdown_scheduler=FakeShutdown(),
            safety_params=SafetyParams(debounce_seconds=0.0),
            startup_interlock=startup_interlock,
        )

    def _tick(self, ctrl, t, ch3=1900, rc_time=None):
        self.clock["t"] = float(t)
        rc = RCInputs(
            ch1_us=1500, ch2_us=1500, ch3_us=ch3, ch4_us=1000, ch5_us=1000,
            last_update_epoch_s=float(t if rc_time is None else rc_time),
        )
        return ctrl.process(rc, now_epoch_s=float(t))

    def test_config_default_is_on(self):
        self.assertIs(SafetyConfig().require_switch_off_at_startup, True)
        self.assertIs(default_config.safety.require_switch_off_at_startup, True)

    def test_without_interlock_the_old_behaviour_stays(self):
        ctrl = self._make(startup_interlock=False)
        cmd, events, telem = self._tick(ctrl, 0.0)
        self.assertTrue(cmd.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)
        self.assertFalse(telem["rearm_requires_switch_cycle"])

    def test_switch_up_at_startup_does_not_arm_until_cycled(self):
        with self.assertLogs("pi_app.control.controller", level="WARNING") as captured:
            ctrl = self._make(startup_interlock=True)
            for t in (0.0, 1.0, 5.0, 30.0):
                cmd, events, telem = self._tick(ctrl, t)
                self.assertFalse(cmd.is_armed, t)
                self.assertNotIn(SafetyEvent.ARMED, events)
                self.assertTrue(telem["rearm_requires_switch_cycle"])
                self.assertEqual(
                    (cmd.left_byte, cmd.right_byte), (126, 126)
                )
            # Switch off: the latch clears (logged), nothing arms yet.
            cmd, events, telem = self._tick(ctrl, 31.0, ch3=1000)
            self.assertFalse(cmd.is_armed)
            self.assertFalse(telem["rearm_requires_switch_cycle"])
            # Switch on again: normal arming.
            cmd, events, telem = self._tick(ctrl, 32.0, ch3=1900)
            self.assertTrue(cmd.is_armed)
            self.assertIn(SafetyEvent.ARMED, events)
        lines = [r.getMessage() for r in captured.records]
        self.assertIn(
            "Arm interlock: the arm switch must be seen OFF before the robot can arm",
            lines,
        )
        self.assertEqual(lines.count("Arm latch cleared: arm switch seen OFF"), 1)

    def test_switch_off_at_startup_is_the_normal_workflow(self):
        ctrl = self._make(startup_interlock=True)
        cmd, _events, telem = self._tick(ctrl, 0.0, ch3=1000)
        self.assertFalse(cmd.is_armed)
        self.assertFalse(telem["rearm_requires_switch_cycle"])
        cmd, events, _telem = self._tick(ctrl, 0.5, ch3=1900)
        self.assertTrue(cmd.is_armed)
        self.assertIn(SafetyEvent.ARMED, events)

    def test_mid_band_switch_does_not_clear_the_latch(self):
        ctrl = self._make(startup_interlock=True)
        self._tick(ctrl, 0.0, ch3=1500)
        cmd, _events, telem = self._tick(ctrl, 1.0, ch3=1900)
        self.assertFalse(cmd.is_armed)
        self.assertTrue(telem["rearm_requires_switch_cycle"])

    def test_rc_stale_at_startup_keeps_the_latch(self):
        # Transmitter off at boot: the RC-stale path must not drop the latch.
        # (A last_update of 0.0 means "no timestamp" and is never stale, so
        # use a real, old one.)
        ctrl = self._make(startup_interlock=True)
        _cmd, events, telem = self._tick(ctrl, 10.0, ch3=1900, rc_time=5.0)
        self.assertIn(SafetyEvent.RC_STALE, events)
        self.assertTrue(telem["rearm_requires_switch_cycle"])
        cmd, _events, telem = self._tick(ctrl, 11.0, ch3=1900)
        self.assertFalse(cmd.is_armed)
        self.assertTrue(telem["rearm_requires_switch_cycle"])


if __name__ == "__main__":
    unittest.main()
