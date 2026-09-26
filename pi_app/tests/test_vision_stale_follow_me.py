"""Follow-me must not drive on a stale detection stream (2026-09-25).

The OAK colour camera stalled. The last person list stayed non-empty for
40 minutes and follow-me drove a left circle. With no freshness getter the
old behaviour stays. With a getter, a stale stream blocks entry and drops
an active FOLLOW_ME session on the tick the age crosses the limit. That
tick's target is the stick command, which the slew limiter ramps to at
the MANUAL rate; no follow-me command is computed on it.
"""

import unittest
from contextlib import contextmanager
from unittest.mock import patch

from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyParams
from config import FollowMeConfig


# The frozen detection from the 18:23 incident: x -0.8 m, z 2.52 m, track 453.
# The box is only slightly left of centre and touches the top edge (the height
# gate skips a top-clipped box) so this is a forward command with a small
# left bias — the same shape as the circle, without depending on a full lock.
INCIDENT_PERSON = PersonDetection(
    x_m=-0.8, z_m=2.52, confidence=0.9,
    bbox=(0.30, 0.0, 0.48, 0.80), track_id=453,
)
# Centred target used by the existing follow-me tests. Far enough to drive.
DRIVE_PERSON = PersonDetection(
    x_m=0.0, z_m=3.0, confidence=0.9,
    bbox=(0.45, 0.0, 0.55, 0.8),
)

ARM_RC = RCInputs(
    ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
    last_update_epoch_s=0.0,
)
FM_RC = RCInputs(
    ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1900, ch5_us=1000,
    last_update_epoch_s=0.0,
)


class FakeMotor:
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_tracks(self, left_byte, right_byte):
        self.commands.append((left_byte, right_byte))

    def stop(self):
        self.stops += 1


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


class FakeGesture:
    def __init__(self):
        self.notified = 0
        self.phase_name = "IDLE"

    @property
    def is_active(self):
        return False

    def update(self, hand):
        return None

    def get_status(self):
        return {"phase": "IDLE"}

    def notify_external_deactivation(self):
        self.notified += 1


def _make_controller(gesture=None):
    fm = FollowMeController(FollowMeConfig())
    ctrl = Controller(
        motor_driver=FakeMotor(),
        arm_relay=FakeRelay(),
        shutdown_scheduler=FakeShutdown(),
        safety_params=SafetyParams(debounce_seconds=0.0),
        follow_me=fm,
        gesture_controller=gesture,
    )
    return ctrl, fm


@contextmanager
def _clock(clock):
    def mono():
        return clock["t"]

    with patch("pi_app.control.controller.time.monotonic", side_effect=mono), \
         patch("pi_app.control.follow_me.time.monotonic", side_effect=mono):
        yield


def _fresh(age, stuck=0, seq=453):
    return {
        "age_s": age,
        "stale": age > 1.5,
        "seq": seq,
        "seq_stuck_packets": stuck,
    }


def _net_forward(cmd) -> bool:
    return (cmd.left_byte + cmd.right_byte) > (2 * CENTER_OUTPUT_VALUE)


class TestVisionGateInactive(unittest.TestCase):
    def test_no_getter_keeps_existing_entry(self):
        ctrl, _fm = _make_controller()
        ctrl.process(ARM_RC, now_epoch_s=10.0)
        ctrl.set_person_detections([INCIDENT_PERSON])
        _cmd, _events, telem = ctrl.process(FM_RC, now_epoch_s=10.1)
        self.assertEqual(ctrl._mode, "FOLLOW_ME")
        self.assertNotIn("vision_age_s", telem)
        self.assertNotIn("vision_stale", telem)
        self.assertNotIn("follow_me_exit_reason", telem)
        self.assertNotIn("follow_me_activation_blocked", telem)

    def test_web_activate_without_getter(self):
        ctrl, _fm = _make_controller()
        ctrl.process(ARM_RC, now_epoch_s=10.0)
        ctrl.set_person_detections([INCIDENT_PERSON])
        self.assertTrue(ctrl.activate_follow_me())
        self.assertEqual(ctrl._mode, "FOLLOW_ME")


class TestVisionStaleEntry(unittest.TestCase):
    def test_rc_entry_refused_and_web_activate_returns_false(self):
        ctrl, _fm = _make_controller()
        calls = {"n": 0}

        def getter():
            calls["n"] += 1
            return _fresh(3.0, stuck=4)

        ctrl.set_vision_freshness_getter(getter)
        ctrl.set_person_detections([INCIDENT_PERSON])
        ctrl.process(ARM_RC, now_epoch_s=10.0)
        _cmd, _events, telem = ctrl.process(FM_RC, now_epoch_s=10.1)
        self.assertEqual(ctrl._mode, "MANUAL")
        self.assertFalse(ctrl._safety_state.follow_me_active)
        self.assertEqual(telem.get("follow_me_activation_blocked"), "vision_stale")
        self.assertIs(telem.get("vision_stale"), True)
        self.assertEqual(telem.get("vision_age_s"), 3.0)
        self.assertFalse(ctrl.activate_follow_me())
        self.assertEqual(ctrl._mode, "MANUAL")
        # One sample per process() tick, plus one live read from activate.
        self.assertEqual(calls["n"], 3)

    def test_stale_and_empty_reports_vision_stale(self):
        ctrl, _fm = _make_controller()
        ctrl.set_vision_freshness_getter(lambda: _fresh(float("inf")))
        ctrl.process(ARM_RC, now_epoch_s=10.0)
        _cmd, _events, telem = ctrl.process(FM_RC, now_epoch_s=10.1)
        self.assertEqual(telem.get("follow_me_activation_blocked"), "vision_stale")
        self.assertIsNone(telem.get("vision_age_s"))
        self.assertIs(telem.get("vision_stale"), True)

    def test_getter_raising_is_stale(self):
        ctrl, fm = _make_controller()
        computes = {"n": 0}
        real_compute = fm.compute

        def wrapped(dets):
            computes["n"] += 1
            return real_compute(dets)

        fm.compute = wrapped

        def boom():
            raise RuntimeError("oak read failed")

        ctrl.set_vision_freshness_getter(boom)
        ctrl.set_person_detections([INCIDENT_PERSON])
        with self.assertLogs("pi_app.control.controller", level="WARNING") as cm:
            ctrl.process(ARM_RC, now_epoch_s=10.0)
            _cmd, _events, telem = ctrl.process(FM_RC, now_epoch_s=10.1)
            # Same failure on the next tick must not log again.
            ctrl.process(FM_RC, now_epoch_s=10.2)
        self.assertEqual(ctrl._mode, "MANUAL")
        self.assertEqual(telem.get("follow_me_activation_blocked"), "vision_stale")
        self.assertIs(telem.get("vision_stale"), True)
        self.assertIsNone(telem.get("vision_age_s"))
        failed = [r.getMessage() for r in cm.records if "getter failed" in r.getMessage()]
        self.assertEqual(len(failed), 1)

        # A healthy read clears the failure episode. Entry works.
        ctrl.set_vision_freshness_getter(lambda: _fresh(0.2))
        ctrl.process(ARM_RC, now_epoch_s=10.3)
        ctrl.process(FM_RC, now_epoch_s=10.4)
        self.assertEqual(ctrl._mode, "FOLLOW_ME")

        # The getter dies again while FOLLOW_ME is active: this tick exits,
        # and both the failure and the stop are logged once.
        ctrl.set_vision_freshness_getter(boom)
        before = computes["n"]
        with self.assertLogs("pi_app.control.controller", level="WARNING") as cm2:
            _cmd, _events, telem = ctrl.process(FM_RC, now_epoch_s=10.5)
        self.assertEqual(computes["n"], before)
        self.assertEqual(ctrl._mode, "MANUAL")
        self.assertEqual(telem.get("follow_me_exit_reason"), "vision_stale")
        self.assertEqual(telem.get("mode"), "MANUAL")
        # This tick's target is the stick command (neutral). The slew
        # limiter ramps the output to it at the MANUAL rate and is not
        # bypassed (this test runs on the real clock, so dt is tiny).
        self.assertEqual(
            (telem.get("slew_in_left"), telem.get("slew_in_right")),
            (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE),
        )
        messages = [r.getMessage() for r in cm2.records]
        self.assertEqual(
            [m for m in messages if "getter failed" in m],
            ["vision freshness getter failed; treating vision as stale"],
        )
        self.assertTrue(any(m.startswith("FOLLOW_ME stopped:") for m in messages))


class TestIncidentRegression(unittest.TestCase):
    def test_frozen_person_drives_until_age_crosses_limit_then_sticks(self):
        gesture = FakeGesture()
        clock = {"t": 1000.0}
        state = {"age": 0.2, "stuck": 0}
        calls = {"n": 0}

        def getter():
            calls["n"] += 1
            return _fresh(state["age"], stuck=state["stuck"])

        with _clock(clock), \
             self.assertLogs("pi_app.control.controller", level="WARNING") as cm:
            ctrl, fm = _make_controller(gesture)
            ctrl.set_vision_freshness_getter(getter)
            ctrl.set_person_detections([INCIDENT_PERSON])
            resets = {"n": 0}
            computes = {"n": 0}
            real_reset = fm.reset_tracking
            real_compute = fm.compute

            def spy_reset():
                resets["n"] += 1
                real_reset()

            def spy_compute(dets):
                computes["n"] += 1
                return real_compute(dets)

            fm.reset_tracking = spy_reset
            fm.compute = spy_compute
            procs = {"n": 0}

            def step(rc, age, stuck):
                state["age"] = age
                state["stuck"] = stuck
                clock["t"] += 0.5
                procs["n"] += 1
                cmd, _events, telem = ctrl.process(rc, now_epoch_s=50.0)
                return cmd, telem

            step(ARM_RC, 0.2, 0)
            cmd, telem = step(FM_RC, 0.2, 0)
            self.assertEqual(telem["mode"], "FOLLOW_ME")
            fresh_forward = _net_forward(cmd)
            for age in (0.4, 1.0, 1.5):
                cmd, telem = step(FM_RC, age, 0)
                self.assertEqual(telem["mode"], "FOLLOW_ME", age)
                self.assertIs(telem["vision_stale"], False)
                fresh_forward = fresh_forward or _net_forward(cmd)
            self.assertTrue(fresh_forward, "follow-me never drove while the stream was fresh")

            before = computes["n"]
            cmd, telem = step(FM_RC, 1.6, 7)
            self.assertEqual(computes["n"], before)
            self.assertEqual(telem["mode"], "MANUAL")
            self.assertEqual(ctrl._mode, "MANUAL")
            self.assertFalse(ctrl._safety_state.follow_me_active)
            self.assertEqual(telem.get("follow_me_exit_reason"), "vision_stale")
            self.assertEqual(
                (cmd.left_byte, cmd.right_byte),
                (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE),
            )
            self.assertEqual(resets["n"], 1)
            self.assertEqual(gesture.notified, 1)

            # Still stale: no second warning, still the sticks, still no compute.
            before = computes["n"]
            cmd, telem = step(FM_RC, 5.0, 8)
            self.assertEqual(computes["n"], before)
            self.assertEqual(telem["mode"], "MANUAL")
            self.assertNotIn("follow_me_exit_reason", telem)
            self.assertEqual(
                (cmd.left_byte, cmd.right_byte),
                (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE),
            )

            # A new RC entry while stale is refused. ch4 has to fall first
            # so the rising edge is real.
            step(ARM_RC, 5.0, 8)
            before = computes["n"]
            _cmd, telem = step(FM_RC, 5.0, 8)
            self.assertEqual(computes["n"], before)
            self.assertEqual(ctrl._mode, "MANUAL")
            self.assertEqual(telem.get("follow_me_activation_blocked"), "vision_stale")

            # Recovery. The stale exit already reset tracking; a new entry drives.
            step(ARM_RC, 0.2, 0)
            self.assertEqual(resets["n"], 1)
            cmd, telem = step(FM_RC, 0.2, 0)
            self.assertEqual(telem["mode"], "FOLLOW_ME")
            self.assertTrue(ctrl._safety_state.follow_me_active)
            cmd, telem = step(FM_RC, 0.2, 0)
            self.assertTrue(_net_forward(cmd), "re-entry did not drive")

            before = computes["n"]
            cmd, telem = step(FM_RC, 2.0, 9)
            self.assertEqual(computes["n"], before)
            self.assertEqual(telem["mode"], "MANUAL")
            self.assertEqual(telem.get("follow_me_exit_reason"), "vision_stale")
            self.assertEqual(
                (cmd.left_byte, cmd.right_byte),
                (CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE),
            )
            self.assertEqual(resets["n"], 2)
            self.assertEqual(gesture.notified, 2)
            self.assertEqual(calls["n"], procs["n"])

        stopped = [
            r.getMessage() for r in cm.records
            if r.getMessage().startswith("FOLLOW_ME stopped:")
        ]
        self.assertEqual(stopped, [
            "FOLLOW_ME stopped: vision stale (age 1.6 s, seq stuck 7)",
            "FOLLOW_ME stopped: vision stale (age 2.0 s, seq stuck 9)",
        ])


class TestHealthyStreamUnchanged(unittest.TestCase):
    def test_bytes_and_mode_match_a_run_with_no_getter(self):
        def run(getter):
            clock = {"t": 1000.0}
            calls = {"n": 0}

            def wrapped():
                calls["n"] += 1
                return getter()

            trace = []
            last = None
            with _clock(clock):
                ctrl, _fm = _make_controller()
                if getter is not None:
                    ctrl.set_vision_freshness_getter(wrapped)
                ctrl.set_person_detections([DRIVE_PERSON])

                def step(rc):
                    nonlocal last
                    clock["t"] += 0.5
                    cmd, _events, telem = ctrl.process(rc, now_epoch_s=50.0)
                    trace.append((telem["mode"], cmd.left_byte, cmd.right_byte))
                    last = telem

                step(ARM_RC)
                step(FM_RC)
                step(FM_RC)
                step(FM_RC)
            return trace, last, calls["n"]

        ref, ref_telem, ref_calls = run(None)
        gated, gated_telem, gated_calls = run(lambda: _fresh(0.2, seq=9))
        self.assertEqual(ref_calls, 0)
        self.assertEqual(gated_calls, 4)
        self.assertEqual(gated, ref)
        self.assertTrue(any(mode == "FOLLOW_ME" for mode, _l, _r in ref))
        self.assertTrue(any(
            (left + right) > (2 * CENTER_OUTPUT_VALUE)
            for mode, left, right in ref
            if mode == "FOLLOW_ME"
        ))
        self.assertNotIn("vision_stale", ref_telem)
        self.assertIs(gated_telem["vision_stale"], False)
        self.assertEqual(gated_telem["vision_age_s"], 0.2)


if __name__ == "__main__":
    unittest.main()
