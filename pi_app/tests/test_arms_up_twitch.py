"""Controller bench-twitch tests for the arms-up detector.

Slew limiter is disabled so we observe the injection-point bytes.
Hold is 0 s so a single new True sample is a rising edge.
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
    arms = replace(
        default_config.arms_up,
        hold_s=0.0,
        release_s=0.0,
        stale_s=10.0,
        twitch_test_enabled=True,
        twitch_reverse_byte=REVERSE_N,
        twitch_duration_s=0.25,
        twitch_cooldown_s=3.0,
        **arms_kw,
    )
    slew = replace(default_config.slew_limiter, enabled=False)
    return replace(default_config, arms_up=arms, slew_limiter=slew)


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
        c, motor, clock = self._make(cfg)
        rc = _arm_rc()
        mode_cmd = (180, 180)

        cmd, _, telem = self._process(c, cfg, clock, 1.0, rc, bt=mode_cmd)
        self.assertTrue(cmd.is_armed)
        self.assertEqual((cmd.left_byte, cmd.right_byte), mode_cmd)

        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, bt=mode_cmd, pose=_pose(ts=1.1),
        )
        au = telem["arms_up"]
        self.assertTrue(au["twitch_active"])
        self.assertEqual(au["twitch_count"], 1)
        self.assertIsNone(au["twitch_blocked_reason"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)
        self.assertEqual(cmd.right_byte, TWITCH_BYTE)
        self.assertEqual(cmd.left_byte, cmd.right_byte)
        self.assertLess(cmd.left_byte, NEUTRAL)

        # Still inside duration (0.25 s).
        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, rc, bt=mode_cmd, pose=_pose(ts=1.2),
        )
        self.assertEqual((cmd.left_byte, cmd.right_byte), (TWITCH_BYTE, TWITCH_BYTE))

        # Duration elapsed → back to the mode's command.
        cmd, _, telem = self._process(
            c, cfg, clock, 1.4, rc, bt=mode_cmd, pose=_pose(ts=1.4),
        )
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual((cmd.left_byte, cmd.right_byte), mode_cmd)

    def test_second_gesture_inside_cooldown_blocked(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc, bt=(180, 180))
        self._process(c, cfg, clock, 1.1, rc, bt=(180, 180), pose=_pose(ts=1.1))
        # Release (release_s=0) then a new rising edge inside cooldown.
        self._process(c, cfg, clock, 1.4, rc, bt=(180, 180), pose=None)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.5, rc, bt=(180, 180), pose=_pose(ts=1.5),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "cooldown")
        self.assertEqual(telem["arms_up"]["twitch_count"], 1)
        self.assertFalse(telem["arms_up"]["twitch_active"])
        self.assertEqual((cmd.left_byte, cmd.right_byte), (180, 180))

    def test_disarmed_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        # Never arm.
        rc = _arm_rc(ch3=1000)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.0, rc, pose=_pose(ts=1.0),
        )
        self.assertFalse(cmd.is_armed)
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "disarmed")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))

    def test_waypoint_nav_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        rc = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc, bt=(180, 180))
        c._mode = "WAYPOINT_NAV"
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc, bt=(180, 180), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "mode")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertNotEqual(cmd.left_byte, TWITCH_BYTE)

    def test_emergency_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        self._process(c, cfg, clock, 1.0, _arm_rc(), bt=(180, 180))
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, _arm_rc(ch5=1900), bt=(180, 180), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "emergency")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))

    def test_charger_blocks_twitch(self):
        cfg = _cfg()
        c, _, clock = self._make(cfg)
        self._process(c, cfg, clock, 1.0, _arm_rc(), bt=(180, 180))
        c.set_charger_inhibit(True)
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, _arm_rc(), bt=(180, 180), pose=_pose(ts=1.1),
        )
        self.assertEqual(telem["arms_up"]["twitch_blocked_reason"], "charger")
        self.assertEqual(telem["arms_up"]["twitch_count"], 0)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))

    def test_disarm_mid_twitch_outputs_neutral(self):
        cfg = _cfg()
        c, motor, clock = self._make(cfg)
        rc_arm = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc_arm, bt=(180, 180))
        cmd, _, telem = self._process(
            c, cfg, clock, 1.1, rc_arm, bt=(180, 180), pose=_pose(ts=1.1),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)

        cmd, _, telem = self._process(
            c, cfg, clock, 1.2, _arm_rc(ch3=1000), bt=(180, 180), pose=_pose(ts=1.2),
        )
        self.assertFalse(cmd.is_armed)
        self.assertEqual((cmd.left_byte, cmd.right_byte), (NEUTRAL, NEUTRAL))
        self.assertGreaterEqual(motor.stops, 1)

    def test_follow_me_twitch_does_not_feed_reverse_or_drop_lock(self):
        cfg = _cfg()
        fm = SpyFollowMe(default_config.follow_me)
        c, _, clock = self._make(cfg, follow_me=fm)
        target = PersonDetection(
            x_m=0.0, z_m=3.0, confidence=0.95,
            bbox=(0.40, 0.0, 0.60, 0.85), track_id=7,
        )
        rc_arm = _arm_rc()
        self._process(c, cfg, clock, 1.0, rc_arm)
        c.set_person_detections([target])
        rc_fm = _arm_rc(ch4=1900)
        cmd, _, telem = self._process(c, cfg, clock, 1.1, rc_fm)
        self.assertEqual(c._mode, "FOLLOW_ME")

        # Let follow-me lock and command forward.
        cmd, _, telem = self._process(c, cfg, clock, 1.2, rc_fm)
        self.assertTrue(telem.get("follow_me_tracking"))
        lock_id = telem.get("follow_me_target_track_id")
        self.assertEqual(lock_id, 7)

        fm.emitted.clear()
        cmd, _, telem = self._process(
            c, cfg, clock, 1.3, rc_fm, pose=_pose(ts=1.3),
        )
        self.assertTrue(telem["arms_up"]["twitch_active"])
        self.assertEqual(cmd.left_byte, TWITCH_BYTE)
        self.assertEqual(cmd.right_byte, TWITCH_BYTE)
        self.assertTrue(telem.get("follow_me_tracking"))
        self.assertEqual(telem.get("follow_me_target_track_id"), lock_id)
        # Speed PID must see 0 forward, never the reverse common-mode (-22).
        self.assertTrue(fm.emitted)
        self.assertEqual(fm.emitted[-1], 0.0)
        self.assertNotIn(-float(REVERSE_N), fm.emitted)
        self.assertEqual(c._last_follow_me_emitted_forward_byte, 0.0)

        # Next tick still twitching: still 0, lock unchanged.
        cmd, _, telem = self._process(
            c, cfg, clock, 1.4, rc_fm, pose=_pose(ts=1.4),
        )
        self.assertEqual(fm.emitted[-1], 0.0)
        self.assertEqual(telem.get("follow_me_target_track_id"), lock_id)
        self.assertTrue(telem.get("follow_me_tracking"))
