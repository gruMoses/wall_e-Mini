"""Controller wiring of the VESC pack guard (2026-11-11 review follow-up).

1. FOLLOW_ME / WAYPOINT_NAV starts are refused while the driver's
   get_pack_guard_status() reports latched, restricted or post_latch (or the
   getter fails: fail closed); a running session that is RESTRICTED or in a
   post-latch session stops at once and leaves to MANUAL (a held stick ramps
   up from neutral); a LATCH keeps the mode and ramps it to zero; nothing
   resumes by itself.
2. The armed flag pushed to set_drive_context() goes False only on a
   deliberate disarm (ch3 low on a live RC link, e-stop, auto-disarm), never
   on an RC dropout, so a dropout cannot clear a pack latch into limp home.
3. request_neutral is the operator's MANUAL request BEFORE the obstacle /
   person-stop layer, and always False in an autonomous mode.
5. A drive limit that is not an int, or outside 0..127, is limit 0.
"""

import unittest
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

from config import FollowMeConfig, ObstacleAvoidanceConfig
from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.gesture_control import GestureEvent
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController
from pi_app.control.safety import SafetyParams

DRIVE_PERSON = PersonDetection(
    x_m=0.0, z_m=3.0, confidence=0.9, bbox=(0.45, 0.0, 0.55, 0.8),
)
DT = 0.1
PARAMS = SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0)


def _rc(now, ch=1500, ch3=1900, ch4=1000, ch5=1000):
    return RCInputs(ch1_us=ch, ch2_us=ch, ch3_us=ch3, ch4_us=ch4, ch5_us=ch5,
                    last_update_mono_s=now)


class GuardMotor:
    """A VESC-like fake: pack guard state, drive limit, drive context log."""

    def __init__(self):
        self.commands = []
        self.stops = 0
        self.contexts = []
        self.limit = None
        self.status = {"latched": False, "restricted": False, "post_latch": False,
                       "reason": None, "voltage_v": 50.0, "voltage_state": "trusted_both"}

    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))

    def stop(self):
        self.stops += 1

    def set_drive_context(self, armed, manual, request_neutral=None):
        self.contexts.append((armed, manual, request_neutral))

    def get_drive_limit_bytes(self):
        return self.limit

    def get_pack_guard_status(self):
        return dict(self.status)

    def guard(self, **flags):
        self.status.update(flags)


class _Relay:
    def set_armed(self, armed):
        pass


class _Shutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


class _Gesture:
    def __init__(self):
        self.next_event = None
        self.notified = 0
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
        self.notified += 1


class _Base(unittest.TestCase):
    def setUp(self):
        self.clock = {"t": 10.0}

        def mono():
            return self.clock["t"]
        for target in ("pi_app.control.controller.time.monotonic",
                       "pi_app.control.follow_me.time.monotonic"):
            p = patch(target, side_effect=mono)
            p.start()
            self.addCleanup(p.stop)

    def make(self, motor, **kw):
        kw.setdefault("arm_relay", _Relay())
        kw.setdefault("shutdown_scheduler", _Shutdown())
        kw.setdefault("safety_params", PARAMS)
        return Controller(motor_driver=motor, **kw)

    def tick(self, c, rc_kwargs=None, stamp=None):
        self.clock["t"] += DT
        now = self.clock["t"]
        rc = _rc(now if stamp is None else stamp, **(rc_kwargs or {}))
        return c.process(rc, now_epoch_s=now)


class TestPackGuardStartGate(_Base):
    def _armed_fm(self, gesture=None):
        motor = GuardMotor()
        c = self.make(motor, follow_me=FollowMeController(FollowMeConfig()),
                      gesture_controller=gesture)
        c.set_person_detections([DRIVE_PERSON])
        cmd, _, _ = self.tick(c)
        self.assertTrue(cmd.is_armed)
        return motor, c

    def test_rc_ch4_start_refused_for_each_guard_state(self):
        for flag in ("latched", "restricted", "post_latch"):
            with self.subTest(flag=flag):
                motor, c = self._armed_fm()
                motor.guard(**{flag: True})
                with self.assertLogs("pi_app.control.controller", level="WARNING"):
                    _, _, telem = self.tick(c, {"ch4": 1900})
                self.assertEqual(c._mode, "MANUAL")
                self.assertFalse(c._safety_state.follow_me_active)
                self.assertEqual(telem.get("follow_me_activation_blocked"), "pack_guard")

    def test_web_follow_me_start_refused_while_post_latch(self):
        motor, c = self._armed_fm()
        motor.guard(post_latch=True)
        self.assertFalse(c.activate_follow_me())
        self.assertEqual(c._mode, "MANUAL")
        motor.guard(post_latch=False)
        self.assertTrue(c.activate_follow_me())

    def test_gesture_start_refused_while_restricted(self):
        gesture = _Gesture()
        motor, c = self._armed_fm(gesture)
        motor.guard(restricted=True)
        gesture.next_event = GestureEvent.ACTIVATE
        _, _, telem = self.tick(c)
        self.assertEqual(c._mode, "MANUAL")
        self.assertEqual(telem["gesture"].get("event_reason"), "blocked_pack_guard")

    def test_unreadable_guard_blocks_follow_me(self):
        motor, c = self._armed_fm()

        def boom():
            raise RuntimeError("guard unreadable")
        motor.get_pack_guard_status = boom
        with self.assertLogs("pi_app.control.controller", level="ERROR"):
            self.tick(c, {"ch4": 1900})
        self.assertEqual(c._mode, "MANUAL")
        self.assertFalse(c.activate_follow_me())

    def test_waypoint_start_refused_with_pack_guard_reason(self):
        motor = GuardMotor()
        c = self.make(motor, waypoint_nav=MagicMock())
        motor.guard(restricted=True)
        self.assertEqual(c.waypoint_nav_activation_blocked(), "pack_guard")
        self.assertEqual(c.activate_waypoint_nav(), "pack_guard")
        self.assertNotEqual(c._mode, "WAYPOINT_NAV")

        def boom():
            raise RuntimeError("guard unreadable")
        motor.guard(restricted=False)
        motor.get_pack_guard_status = boom
        with self.assertLogs("pi_app.control.controller", level="ERROR"):
            self.assertEqual(c.waypoint_nav_activation_blocked(), "pack_guard")

    def test_driver_without_guard_status_is_not_gated(self):
        motor = GuardMotor()
        del motor.status
        motor.get_pack_guard_status = None   # not callable: Arduino fallback shape
        c = self.make(motor, waypoint_nav=MagicMock())
        self.assertEqual(c.waypoint_nav_activation_blocked(), "gps_state_unknown")


class TestPackGuardSessionExit(_Base):
    def test_running_follow_me_leaves_through_the_slew_and_does_not_resume(self):
        motor = GuardMotor()
        c = self.make(motor, follow_me=FollowMeController(FollowMeConfig()))
        c.set_person_detections([DRIVE_PERSON])
        self.tick(c)                                   # arm
        self.tick(c, {"ch4": 1900})                    # FOLLOW_ME
        self.assertEqual(c._mode, "FOLLOW_ME")
        for _ in range(30):
            cmd, _, _ = self.tick(c, {"ch4": 1900})
        moving = cmd.left_byte
        self.assertGreater(moving, CENTER_OUTPUT_VALUE + 20)

        motor.guard(restricted=True)
        motor.limit = 32                               # MANUAL cap while restricted
        with self.assertLogs("pi_app.control.controller", level="WARNING") as cm:
            cmd, _, telem = self.tick(c, {"ch4": 1900})
        self.assertEqual(c._mode, "MANUAL")
        self.assertFalse(c._safety_state.follow_me_active)
        self.assertEqual(telem.get("follow_me_exit_reason"), "pack_guard")
        ev = telem.get("pack_guard_mode_exit")
        self.assertEqual((ev["mode"], ev["reason"]), ("FOLLOW_ME", "restricted"))
        self.assertTrue(any("FOLLOW_ME stopped: pack guard restricted" in m for m in cm.output))
        # Stop AT ONCE on the exit tick (Grok 01a1ccd5): a slew decel toward
        # centred sticks coasted ~0.37 s with the person stop blind.
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
        self.assertEqual(motor.commands[-1], (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
        self.assertEqual(motor.contexts[-1][1], True)   # driver told MANUAL
        self.assertEqual(motor.contexts[-1][2], True)   # sticks centred, output neutral

        # Guard clear again: the session does not come back by itself.
        motor.guard(restricted=False)
        motor.limit = None
        for _ in range(20):
            cmd, _, _ = self.tick(c, {"ch4": 1900})
        self.assertEqual(c._mode, "MANUAL")
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))

    def test_held_stick_after_the_exit_ramps_up_from_neutral(self):
        motor = GuardMotor()
        c = self.make(motor, follow_me=FollowMeController(FollowMeConfig()))
        c.set_person_detections([DRIVE_PERSON])
        self.tick(c)
        self.tick(c, {"ch4": 1900})
        for _ in range(30):
            self.tick(c, {"ch4": 1900})
        motor.guard(restricted=True)
        motor.limit = 32
        with self.assertLogs("pi_app.control.controller", level="WARNING"):
            cmd, _, _ = self.tick(c, {"ch4": 1900, "ch": 2000})   # sticks held forward
        self.assertEqual(c._mode, "MANUAL")
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))   # stop first
        outs = []
        for _ in range(5):
            cmd, _, _ = self.tick(c, {"ch4": 1900, "ch": 2000})
            outs.append(cmd.left_byte)
        accel = float(default_config.slew_limiter.manual_accel_bps) * DT
        self.assertGreater(outs[-1], CENTER_OUTPUT_VALUE)              # the stick drives again
        self.assertLessEqual(outs[0], CENTER_OUTPUT_VALUE + accel + 1)  # ramped, no jump
        self.assertLessEqual(max(outs), CENTER_OUTPUT_VALUE + 32)      # inside the cap

    def test_latch_keeps_the_mode_and_ramps_to_zero(self):
        motor = GuardMotor()
        c = self.make(motor, follow_me=FollowMeController(FollowMeConfig()))
        c.set_person_detections([DRIVE_PERSON])
        self.tick(c)
        self.tick(c, {"ch4": 1900})
        for _ in range(30):
            self.tick(c, {"ch4": 1900})
        motor.guard(latched=True, reason="pack_voltage")
        motor.limit = 0
        outs = []
        for _ in range(15):
            cmd, _, telem = self.tick(c, {"ch4": 1900})
            outs.append(cmd.left_byte)
        self.assertEqual(c._mode, "FOLLOW_ME")              # no exit on a latch
        self.assertNotIn("pack_guard_mode_exit", telem)
        self.assertEqual(outs[-1], CENTER_OUTPUT_VALUE)     # ramped to zero
        self.assertGreater(outs[0], CENTER_OUTPUT_VALUE)    # a ramp, not a step

    def test_running_waypoint_nav_leaves_on_post_latch(self):
        motor = GuardMotor()
        c = self.make(motor)
        self.tick(c)                                   # arm
        c._mode = "WAYPOINT_NAV"
        motor.guard(post_latch=True)
        with self.assertLogs("pi_app.control.controller", level="WARNING"):
            _, _, telem = self.tick(c)
        self.assertEqual(c._mode, "MANUAL")
        self.assertEqual(telem.get("waypoint_nav_exit_reason"), "pack_guard")
        self.assertEqual(telem["pack_guard_mode_exit"]["mode"], "WAYPOINT_NAV")


class TestDeliberateDisarmToDriver(_Base):
    def test_rc_dropout_keeps_armed_until_ch3_low_on_a_live_link(self):
        motor = GuardMotor()
        c = self.make(motor)
        self.tick(c)
        self.assertEqual(motor.contexts[-1][0], True)
        last_frame = self.clock["t"]
        for _ in range(15):                            # 1.5 s with no new frame
            cmd, events, _ = self.tick(c, stamp=last_frame)
        self.assertFalse(cmd.is_armed)                 # the controller disarmed
        n = len(motor.contexts)
        self.assertTrue(all(ctx[0] for ctx in motor.contexts), motor.contexts)
        # Link back, switch still up: re-arms; the driver never saw a disarm.
        for _ in range(3):
            cmd, _, _ = self.tick(c)
        self.assertTrue(cmd.is_armed)
        self.assertTrue(all(ctx[0] for ctx in motor.contexts[n:]))
        # Deliberate: ch3 low on a live link.
        cmd, _, _ = self.tick(c, {"ch3": 1000})
        self.assertFalse(cmd.is_armed)
        self.assertEqual(motor.contexts[-1][0], False)

    def test_estop_and_auto_disarm_are_deliberate(self):
        motor = GuardMotor()
        c = self.make(motor)
        self.tick(c)
        self.tick(c, {"ch5": 1900})
        self.assertEqual(motor.contexts[-1][0], False)

        motor2 = GuardMotor()
        c2 = self.make(motor2)
        self.tick(c2)
        self.assertEqual(motor2.contexts[-1][0], True)
        limit_s = float(default_config.safety.auto_disarm_idle_s)
        self.assertGreater(limit_s, 0.0)
        c2._armed_idle_since = self.clock["t"] - limit_s - 1.0
        with self.assertLogs("pi_app.control.controller", level="WARNING"):
            cmd, _, _ = self.tick(c2)                  # ch3 still high
        self.assertFalse(cmd.is_armed)
        self.assertEqual(motor2.contexts[-1][0], False)


class TestRequestNeutral(_Base):
    def test_stick_held_into_the_person_stop_is_not_zero_throttle(self):
        motor = GuardMotor()
        c = self.make(motor, obstacle_avoidance=ObstacleAvoidanceController(ObstacleAvoidanceConfig()))
        c.set_obstacle_data(distance_m=5.0, age_s=0.0)
        self.tick(c)
        c.set_obstacle_data(distance_m=0.0, age_s=0.0)     # YOLO person stop
        cmd, _, telem = self.tick(c, {"ch": 2000})
        self.assertEqual((cmd.left_byte, cmd.right_byte),
                         (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE))
        self.assertEqual(motor.contexts[-1], (True, True, False))
        cmd, _, _ = self.tick(c, {"ch": 1500})
        self.assertEqual(motor.contexts[-1], (True, True, True))

    def test_autonomous_mode_is_never_zero_throttle(self):
        motor = GuardMotor()
        c = self.make(motor)
        self.tick(c)
        c._mode = "FOLLOW_ME"          # no follow-me controller: neutral request
        self.tick(c)
        self.assertEqual(motor.contexts[-1], (True, False, False))


class TestLimitValueValidation(_Base):
    def test_garbage_limit_is_limit_zero(self):
        for bad in (500, 128, -3, 32.5, True, "32", float("nan")):
            with self.subTest(bad=bad):
                motor = GuardMotor()
                c = self.make(motor)
                self.tick(c)
                motor.limit = bad
                with patch("pi_app.control.controller._logger"):
                    cmd, _, telem = self.tick(c, {"ch": 2000})
                self.assertEqual(telem.get("vesc_pack_drive_limit_bytes"), 0)
                self.assertEqual(cmd.left_byte, CENTER_OUTPUT_VALUE)

    def test_garbage_limit_logs_once_per_episode(self):
        motor = GuardMotor()
        c = self.make(motor)
        self.tick(c)
        motor.limit = 500
        with self.assertLogs("pi_app.control.controller", level="ERROR") as cm:
            for _ in range(5):
                self.tick(c, {"ch": 2000})
        self.assertEqual(sum("get_drive_limit_bytes returned" in m for m in cm.output), 1)

    def test_valid_limits_pass(self):
        for good, want in ((None, None), (0, 0), (32, 32), (127, 127)):
            with self.subTest(good=good):
                motor = GuardMotor()
                c = self.make(motor)
                self.tick(c)
                motor.limit = good
                _, _, telem = self.tick(c, {"ch": 2000})
                self.assertEqual(telem.get("vesc_pack_drive_limit_bytes"), want)


if __name__ == "__main__":
    unittest.main()
