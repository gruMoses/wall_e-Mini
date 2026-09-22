"""Controller bench-twitch tests for the arms-up detector.

Slew limiter is disabled so we observe the injection-point bytes, except
the slew test. Hold is 0 s and min_hold_samples is 1, so a single new
True sample is a rising edge. The detector's gap and sample-count rules
are covered in test_arms_up.py; this file widens max_sample_gap_s so a
controller scenario can step a few tenths of a second.

The bench latch is runtime state. Each test that expects a pulse calls
request_twitch_test(True) while armed in MANUAL.
"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from config import config as default_config
from pi_app.control.controller import (
    Controller, RCInputs, MotorDriver, ArmRelay, ShutdownScheduler,
)
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyParams
from pi_app.tests.test_arms_up import _pose


class FakeMotor(MotorDriver):
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        self.commands.append((left_byte, right_byte))

    def stop(self) -> None:
        self.stops += 1

    def get_telemetry(self):
        return None


class FakeRelay(ArmRelay):
    def __init__(self):
        self.states = []

    def set_armed(self, armed: bool) -> None:
        self.states.append(armed)


class FakeShutdown(ShutdownScheduler):
    def __init__(self):
        self.scheduled = []

    def schedule_shutdown(self, delay_seconds: float) -> None:
        self.scheduled.append(delay_seconds)


NEUTRAL = CENTER_OUTPUT_VALUE  # 126
REVERSE_N = 22
TWITCH_BYTE = NEUTRAL - REVERSE_N  # 104


def _cfg(**arms_kw):
    fields = dict(
        hold_s=0.0,
        release_s=0.0,
        stale_s=10.0,
        min_hold_samples=1,
        max_sample_gap_s=1.0,
        twitch_min_still_s=0.0,
        twitch_test_budget=3,
        twitch_test_max_s=300.0,
        twitch_reverse_byte=REVERSE_N,
        twitch_duration_s=0.25,
        twitch_cooldown_s=3.0,
    )
    fields.update(arms_kw)
    arms = replace(default_config.arms_up, **fields)
    slew = replace(default_config.slew_limiter, enabled=False)
    return replace(default_config, arms_up=arms, slew_limiter=slew)


def _down(ts):
    return _pose(ts=ts, l_wr_y=0.80, r_wr_y=0.80)


def _arm_rc(ch3=1900, ch4=1000, ch5=1000):
    return RCInputs(
        ch1_us=1500, ch2_us=1500, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5,
        last_update_epoch_s=0.0,
    )


class SpyFollowMe(FollowMeController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.emitted = []

    def update_telemetry(self, *args, **kwargs):
        self.emitted.append(kwargs.get("emitted_forward_byte"))
        return super().update_telemetry(*args, **kwargs)


class TestControllerArmsUpTwitch(unittest.TestCase):
    def _make(self, cfg, follow_me=None):
        fake_now = {"t": 0.0}
        params = SafetyParams(debounce_seconds=0.0)
        motor = FakeMotor()
        with patch("pi_app.control.controller.config", cfg):
            with patch(
                "pi_app.control.controller.time.monotonic",
                side_effect=lambda: fake_now["t"],
            ):
                c = Controller(
                    motor_driver=motor,
                    arm_relay=FakeRelay(),
                    shutdown_scheduler=FakeShutdown(),
                    safety_params=params,
                    follow_me=follow_me,
                )
        return c, motor, fake_now

    def _request(self, c, cfg, clock, enabled, t=None):
        if t is not None:
            clock["t"] = t
        with patch("pi_app.control.controller.config", cfg):
            with patch(
                "pi_app.control.controller.time.monotonic",
                side_effect=lambda: clock["t"],
            ):
                return c.request_twitch_test(enabled)

    def _state(self, c, clock, t=None):
        if t is not None:
            clock["t"] = t
        with patch(
            "pi_app.control.controller.time.monotonic",
            side_effect=lambda: clock["t"],
        ):
            return c.get_twitch_test_state()

    def _arm_and_latch(self, c, cfg, clock, t=1.0):
        """One armed neutral tick, then latch. The tick starts the still timer."""
        rc = _arm_rc()
        self._process(c, cfg, clock, t, rc)
        ok, reason = self._request(c, cfg, clock, True, t)
        self.assertEqual((ok, reason), (True, "ok"))
        self.assertTrue(c.pose_wanted())
        return rc

    _KEEP = object()  # leave the controller's current pose sample untouched

    def _process(self, c, cfg, fake_now, t, rc, bt=None, pose=_KEEP):
        fake_now["t"] = t
        # pose=None must reach the controller: it means "no pose this frame"
        # and is what releases the debounce. Only an omitted kwarg keeps the
        # previous sample (main.py re-sends the latest sample every tick).
        if pose is not self._KEEP:
            c.set_pose_sample(pose)
        with patch("pi_app.control.controller.config", cfg):
            with patch(
                "pi_app.control.controller.time.monotonic",
                side_effect=lambda: fake_now["t"],
            ):
                return c.process(rc, now_epoch_s=t, bt_override_bytes=bt)

    def test_armed_manual_rising_edge_twitches_then_returns_to_mode(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)

        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        au = telem["arms_up"]
        self.assertTrue(au["twitch_active"])
        self.assertEqual(au["twitch_count"], 1)
        self.assertEqual(au["test_budget_left"], 2)
        self.assertIsNone(au["twitch_blocked_reason"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)
        self.assertEqual(cmd.right_byte, TWITCH_BYTE)
        self.assertEqual(cmd.left_byte, cmd.right_byte)
        self.assertLess(cmd.left_byte, NEUTRAL)

        # Still inside duration (0.25 s).
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.2),
        )
        self.assertEqual((cmd.left_byte, cmd.right_byte), (TWITCH_BYTE, TWITCH_BYTE))

        # Duration elapsed → back to the MANUAL stick command (neutral).
        cmd, _, telem = self._process(
            c, cfg, clock, 1.4, rc, pose=_pose(ts=1.4),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))

    def test_second_gesture_inside_cooldown_blocked(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        self._process(c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1))
        # Pulse ends at 1.35. Arms-down after that re-arms and leaves the
        # tracks at neutral, so the next edge is blocked by cooldown rather
        # than by the still-timer.
        self._process(c, cfg, clock, 1.4, rc, pose=_down(1.4))
        cmd, _, telem = self._process(
            c, cfg, clock, 1.5, rc, pose=_pose(ts=1.5),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "cooldown")
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))
        # After the cooldown the consumed edge does not fire.
        cmd, _, telem = self._process(
            c, cfg, clock, 5.0, rc, pose=_pose(ts=1.5),
        )
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertFalse(telem["arms_up"]["twitch_active"])

    def test_disarmed_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, _arm_rc(ch3=1000), pose=_pose(ts=1.1),
        )
        self.assertFalse(cmd.is_armed)
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "disarmed")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))
        # Re-arm and re-latch. The consumed edge does not fire.
        self._process(c, cfg, clock, 1.3, rc, pose=_pose(ts=1.1))
        self._request(c, cfg, clock, True, 1.3)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.4, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)

    def test_waypoint_nav_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        c._mode = "WAYPOINT_NAV"
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "mode")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)
        c._mode = "MANUAL"
        self._request(c, cfg, clock, True, 1.2)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertFalse(telem["arms_up"]["twitch_active"])

    def test_emergency_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, _arm_rc(ch5=1900), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "emergency")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))

    def test_charger_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        c.set_charger_inhibit(True)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "charger")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))
        c.set_charger_inhibit(False)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)

    def test_disarm_mid_twitch_outputs_neutral(self):
        cfg = _cfg()
        c, motor, clock = self._make(cfg)
        rc_arm = self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc_arm, pose=_pose(ts=1.1),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)

        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, _arm_rc(ch3=1000), pose=_pose(ts=1.2),
        )
        self.assertFalse(cmd.is_armed)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))
        self.assertGreaterEqual(motor.stops, 1)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "disarmed")

    def test_follow_me_never_starts_a_pulse(self):
        cfg = _cfg()
        fm = SpyFollowMe(default_config.follow_me)
        c, _, clock = self._make(cfg, follow_me=fm)
        target = PersonDetection(
            x_m=0.0, z_m=3.0, confidence=0.95,
            bbox=(0.40, 0.0, 0.60, 0.85), track_id=7,
        )
        self._arm_and_latch(c, cfg, clock, 1.0)
        c.set_person_detections([target])
        rc_fm = _arm_rc(ch4=1900)
        self._process(c, cfg, clock, 1.1, rc_fm)
        self.assertEqual(c._mode, "FOLLOW_ME")
        self.assertFalse(c.pose_wanted())
        ok, reason = self._request(c, cfg, clock, True, 1.1)
        self.assertEqual((ok, reason), (False, "mode"))

        cmd, _, telem = self._process(c, cfg, clock, 1.2, rc_fm)
        self.assertTrue(telem.get("follow_me_tracking"))
        lock_id = telem.get("follow_me_target_track_id")
        self.assertEqual(lock_id, 7)

        fm.emitted.clear()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, rc_fm, pose=_pose(ts=1.3),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertTrue(telem.get("follow_me_tracking"))
        self.assertEqual(telem.get("follow_me_target_track_id"), lock_id)
        self.assertEqual(
            c._last_follow_me_emitted_forward_byte,
            (cmd.left_byte + cmd.right_byte) / 2.0 - NEUTRAL,
        )
        self.assertTrue(fm.emitted)
        self.assertNotIn(-float(REVERSE_N), fm.emitted)


class TestTwitchStartGates(TestControllerArmsUpTwitch):
    def _later(self, c, cfg, clock, rc, t, pose_ts, count):
        cmd, _, telem = self._process(
            c, cfg, clock, t, rc, pose=_pose(ts=pose_ts),
        )
        self.assertEqual(telem["arms_up"]["twitch_count"], count)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)
        return cmd, telem

    def test_latch_off_blocks_and_does_not_fire_later(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "latch")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self._request(c, cfg, clock, True, 1.1)
        self._later(c, cfg, clock, rc, 1.2, 1.1, 0)

    def test_budget_exhausted_blocks_fourth_edge(self):
        cfg = _cfg(twitch_cooldown_s=0.0, twitch_test_budget=3)
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 0.0)
        self._process(c, cfg, clock, 0.10, rc, pose=_pose(ts=0.10))
        self._process(c, cfg, clock, 0.40, rc, pose=_down(0.40))
        self._process(c, cfg, clock, 0.50, rc, pose=_pose(ts=0.50))
        self._process(c, cfg, clock, 0.80, rc, pose=_down(0.80))
        cmd, _, telem = self._process(
            c, cfg, clock, 0.90, rc, pose=_pose(ts=0.90),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_count"], 3)
        self.assertEqual(telem["arms_up"]["test_budget_left"], 0)
        # Re-arm during the last pulse, then the next edge meets a spent budget.
        self._process(c, cfg, clock, 1.00, rc, pose=_down(1.00))
        cmd, _, telem = self._process(
            c, cfg, clock, 1.20, rc, pose=_pose(ts=1.20),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "budget")
        self.assertEqual(telem["arms_up"]["twitch_count"], 3)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self.assertFalse(c.pose_wanted())
        self._request(c, cfg, clock, True, 1.20)
        self._later(c, cfg, clock, rc, 1.30, 1.20, 3)

    def test_pack_low_blocks_and_does_not_fire_later(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        c._vesc_pack_low_latched = True
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "pack_low")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        c._vesc_pack_low_latched = False
        self._later(c, cfg, clock, rc, 1.2, 1.1, 0)

    def test_calibration_blocks_start(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        self._arm_and_latch(c, cfg, clock, 1.0)
        c.enter_calibration_mode()
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "calibration"))
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, _arm_rc(), pose=_pose(ts=1.1),
        )
        self.assertTrue(telem.get("calibration"))
        self.assertEqual(cmd.left_byte, NEUTRAL)
        state = self._state(c, clock, 1.1)
        self.assertFalse(state["latched"])
        self.assertFalse(state["twitch_active"])
        self.assertEqual(state["twitch_count"], 0)
        c.exit_calibration_mode()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, _arm_rc(), pose=_pose(ts=1.1),
        )
        self.assertFalse(c.pose_wanted())
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertFalse(telem["arms_up"]["twitch_active"])

    def test_bt_override_blocks_and_does_not_fire_later(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, bt=(NEUTRAL, NEUTRAL), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "bt_override")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self._later(c, cfg, clock, rc, 1.2, 1.1, 0)

    def test_stick_out_of_band_blocks_and_does_not_fire_later(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        rc_out = _arm_rc()
        rc_out = replace_rc(rc_out, ch1_us=1600)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc_out, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "stick")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)
        self._later(c, cfg, clock, rc, 1.2, 1.1, 0)

    def test_not_still_long_enough_blocks_and_does_not_fire_later(self):
        cfg = _cfg(twitch_min_still_s=0.5)
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.2),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "not_still")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self._later(c, cfg, clock, rc, 2.0, 1.2, 0)

    def test_rpm_above_still_cap_blocks_and_does_not_fire_later(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        c._actual_left_rpm = 1000
        c._actual_right_rpm = 0
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "not_still")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        c._actual_left_rpm = None
        c._actual_right_rpm = None
        self._later(c, cfg, clock, rc, 1.2, 1.1, 0)


def replace_rc(rc, **kwargs):
    return RCInputs(
        ch1_us=kwargs.get("ch1_us", rc.ch1_us),
        ch2_us=kwargs.get("ch2_us", rc.ch2_us),
        ch3_us=kwargs.get("ch3_us", rc.ch3_us),
        ch4_us=kwargs.get("ch4_us", rc.ch4_us),
        ch5_us=kwargs.get("ch5_us", rc.ch5_us),
        last_update_epoch_s=kwargs.get("last_update_epoch_s", rc.last_update_epoch_s),
    )


class TestTwitchContinue(TestControllerArmsUpTwitch):
    def _start(self, cfg=None):
        cfg = cfg or _cfg()
        c, motor, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, pose=_pose(ts=1.1),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"], telem["arms_up"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)
        return c, motor, clock, cfg, rc

    def _no_resume(self, c, cfg, clock, rc):
        # 1.3 is still inside the original 0.25 s window that began at 1.1.
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, rc, pose=_pose(ts=1.1),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self.assertEqual(cmd.right_byte, NEUTRAL)

    def test_stick_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, replace_rc(rc, ch1_us=1600), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "stick")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)
        self.assertTrue(c.pose_wanted())
        self._no_resume(c, cfg, clock, rc)

    def test_bt_override_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, bt=(NEUTRAL, NEUTRAL), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "bt_override")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self._no_resume(c, cfg, clock, rc)

    def test_follow_me_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        c._mode = "FOLLOW_ME"
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "mode")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)
        c._mode = "MANUAL"
        self._request(c, cfg, clock, True, 1.25)
        self._no_resume(c, cfg, clock, rc)

    def test_disarm_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, _arm_rc(ch3=1000), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "disarmed")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self._process(c, cfg, clock, 1.25, rc, pose=_pose(ts=1.1))
        self._request(c, cfg, clock, True, 1.25)
        self._no_resume(c, cfg, clock, rc)

    def test_emergency_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, _arm_rc(ch5=1900), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "emergency")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        c._safety_state.emergency_active = False
        self._process(c, cfg, clock, 1.25, rc, pose=_pose(ts=1.1))
        self._request(c, cfg, clock, True, 1.25)
        self._no_resume(c, cfg, clock, rc)

    def test_charger_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        c.set_charger_inhibit(True)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "charger")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self.assertTrue(c.pose_wanted())
        c.set_charger_inhibit(False)
        self._no_resume(c, cfg, clock, rc)

    def test_pack_low_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        c._vesc_pack_low_latched = True
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_cancel_reason"], "pack_low")
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, NEUTRAL)
        c._vesc_pack_low_latched = False
        self._no_resume(c, cfg, clock, rc)

    def test_calibration_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        c.enter_calibration_mode()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, pose=_pose(ts=1.1),
        )
        self.assertTrue(telem.get("calibration"))
        self.assertEqual(cmd.left_byte, NEUTRAL)
        state = self._state(c, clock, 1.2)
        self.assertEqual(state["twitch_cancel_reason"], "calibration")
        self.assertFalse(state["twitch_active"])
        self.assertFalse(state["latched"])
        c.exit_calibration_mode()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, rc, pose=_pose(ts=1.1),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertEqual(cmd.left_byte, NEUTRAL)

    def test_rc_stale_cancels_and_does_not_resume(self):
        c, _, clock, cfg, rc = self._start()
        stale = replace_rc(rc, last_update_epoch_s=0.05)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, stale, pose=_pose(ts=1.1),
        )
        self.assertTrue(telem.get("rc_stale"))
        self.assertEqual(cmd.left_byte, NEUTRAL)
        self.assertFalse(cmd.is_armed)
        state = self._state(c, clock, 1.2)
        self.assertEqual(state["twitch_cancel_reason"], "rc_stale")
        self.assertFalse(state["twitch_active"])
        self.assertFalse(state["latched"])
        fresh = replace_rc(rc, last_update_epoch_s=1.3)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, fresh, pose=_pose(ts=1.1),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertEqual(cmd.left_byte, NEUTRAL)

    def test_post_false_cancels_active_pulse(self):
        c, _, clock, cfg, rc = self._start()
        ok, reason = self._request(c, cfg, clock, False, 1.15)
        self.assertEqual((ok, reason), (True, "ok"))
        state = self._state(c, clock, 1.15)
        self.assertFalse(state["latched"])
        self.assertFalse(state["twitch_active"])
        self.assertEqual(state["twitch_cancel_reason"], "api")
        self._no_resume(c, cfg, clock, rc)


class TestTwitchLatch(TestControllerArmsUpTwitch):
    def test_refused_when_disarmed_not_manual_charger_pack_low(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "disarmed"))
        self.assertFalse(c.pose_wanted())

        self._process(c, cfg, clock, 1.0, _arm_rc())
        c._mode = "FOLLOW_ME"
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "mode"))
        c._mode = "WAYPOINT_NAV"
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "mode"))

        c._mode = "MANUAL"
        c.set_charger_inhibit(True)
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "charger"))
        c.set_charger_inhibit(False)

        c._vesc_pack_low_latched = True
        ok, reason = self._request(c, cfg, clock, True, 1.0)
        self.assertEqual((ok, reason), (False, "pack_low"))
        self.assertFalse(c.pose_wanted())

    def test_expiry_clears_latch(self):
        cfg = _cfg(twitch_test_max_s=1.0)
        c, _, clock = self._make(cfg)
        rc = self._arm_and_latch(c, cfg, clock, 1.0)
        state = self._state(c, clock, 1.5)
        self.assertTrue(state["latched"])
        self.assertAlmostEqual(state["expires_in_s"], 0.5)
        cmd, _, telem = self._process(
            c, cfg, clock, 2.0, rc, pose=_pose(ts=2.0),
        )
        self.assertFalse(c.pose_wanted())
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "expiry")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual(cmd.left_byte, NEUTRAL)
        state = self._state(c, clock, 2.0)
        self.assertFalse(state["latched"])
        self.assertEqual(state["budget_left"], 0)


class TestPoseWanted(TestControllerArmsUpTwitch):
    def test_false_unless_latched_in_manual(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        self.assertFalse(c.pose_wanted())
        rc = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc)
        self.assertFalse(c.pose_wanted())
        self._request(c, cfg, clock, True, 1.0)
        self.assertTrue(c.pose_wanted())

        self._process(c, cfg, clock, 1.1, _arm_rc(ch3=1000))
        self.assertFalse(c.pose_wanted())

        rc = self._arm_and_latch(c, cfg, clock, 1.2)
        c._mode = "FOLLOW_ME"
        self._process(c, cfg, clock, 1.3, rc)
        self.assertFalse(c.pose_wanted())

        c._mode = "MANUAL"
        self._request(c, cfg, clock, True, 1.4)
        c._mode = "WAYPOINT_NAV"
        self._process(c, cfg, clock, 1.5, rc)
        self.assertFalse(c.pose_wanted())

        self._request(c, cfg, clock, False, 1.6)
        self.assertFalse(c.pose_wanted())


class TestTwitchSlew(TestControllerArmsUpTwitch):
    def test_slew_never_below_offset_and_returns_within_tail(self):
        cfg = _cfg(twitch_min_still_s=0.5, twitch_duration_s=0.25)
        cfg = replace(
            cfg,
            slew_limiter=replace(default_config.slew_limiter, enabled=True),
        )
        c, _, clock = self._make(cfg)
        rc = _arm_rc()
        for i in range(26):
            self._process(c, cfg, clock, i * 0.02, rc)
        self._request(c, cfg, clock, True, 0.50)
        samples = []
        cmd, _, telem = self._process(
            c, cfg, clock, 0.52, rc, pose=_pose(ts=0.52),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"])
        samples.append((0.52, cmd.left_byte, cmd.right_byte))
        t = 0.54
        while t <= 0.90 + 1e-9:
            cmd, _, _ = self._process(c, cfg, clock, t, rc, pose=_pose(ts=0.52))
            samples.append((t, cmd.left_byte, cmd.right_byte))
            t = round(t + 0.02, 10)
        for _, left, right in samples:
            self.assertGreaterEqual(left, TWITCH_BYTE)
            self.assertGreaterEqual(right, TWITCH_BYTE)
            self.assertEqual(left, right)
        tail = REVERSE_N / float(default_config.slew_limiter.manual_decel_bps)
        deadline = 0.52 + 0.25 + tail
        after = [left for ts, left, _ in samples if ts + 1e-9 >= deadline]
        self.assertTrue(after)
        self.assertEqual(after[0], NEUTRAL)
