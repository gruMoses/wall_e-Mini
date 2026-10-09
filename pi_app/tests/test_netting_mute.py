"""Operator netting mute (2026-10-08): pi_app/control/netting_mute.py,
the corridor_muted path in ObstacleAvoidanceController, the controller
wiring, and POST /api/manual/netting_mute.

Review requirements (Grok, sessions 01a11de8 and 01a11df6): no automatic
engage; the tap is accepted only with the netting already in view (no
waiting state); a finite reading beyond the band drops the mute on that
tick (never a blind hold at speed); only a dropout gets a short hold; it
drops on any person/animal, on disarm, on leaving MANUAL, on a timeout,
and never resumes without a new tap; the YOLO stop channel (distance 0.0)
and a stale depth are never muted; autonomy never uses it; the web thread
only queues and process() is the single writer.
"""

import json
import threading
import time
import unittest
import unittest.mock
from dataclasses import replace

from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.netting_mute import (
    NettingMute,
    detection_drops_mute,
    near_field_present,
    near_share,
)
from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController

try:
    import flask  # noqa: F401
    from config import OakWebViewerConfig
    from pi_app.web.oak_viewer import create_app
except ImportError:  # pragma: no cover
    flask = None

CAP = 0.55
MIN = 0.85


class TestNearFieldPresent(unittest.TestCase):
    def test_netting_shape(self):
        # The 60 netting frames: 0.40-0.41 m, near/support 0.95-0.98; the
        # drive log: 0.40-0.50 m.
        self.assertTrue(near_field_present(0.41, 38000, 39000, CAP, MIN))
        self.assertTrue(near_field_present(0.50, 38000, 39000, CAP, MIN))

    def test_yolo_stop_channel_is_not_a_near_field(self):
        self.assertFalse(near_field_present(0.0, 38000, 39000, CAP, MIN))

    def test_cap_is_on_the_data_not_above_it(self):
        # Sunlit siding reads 0.71-0.75 m at 99.5-99.8 percent share; a wall
        # at 0.65 m must not count either.
        self.assertFalse(near_field_present(0.71, 39900, 40000, CAP, MIN))
        self.assertFalse(near_field_present(0.65, 39900, 40000, CAP, MIN))
        self.assertFalse(near_field_present(0.56, 39900, 40000, CAP, MIN))
        self.assertTrue(near_field_present(0.55, 39900, 40000, CAP, MIN))

    def test_clear_and_missing(self):
        self.assertFalse(near_field_present(float("inf"), 0, 0, CAP, MIN))
        self.assertFalse(near_field_present(None, 0, 0, CAP, MIN))
        self.assertFalse(near_field_present(0.5, 0, 0, CAP, MIN))
        self.assertFalse(near_field_present(0.5, None, None, CAP, MIN))

    def test_sparse_near_field_is_not_present(self):
        self.assertFalse(near_field_present(0.5, 2000, 40000, CAP, MIN))
        self.assertFalse(near_field_present(0.5, 33000, 40000, CAP, MIN))  # 0.825
        self.assertTrue(near_field_present(0.5, 34000, 40000, CAP, MIN))   # 0.85

    def test_near_share(self):
        self.assertIsNone(near_share(0, 0))
        self.assertIsNone(near_share(None, None))
        self.assertAlmostEqual(near_share(95, 100), 0.95)


class _Det:
    def __init__(self, tier):
        self.safety_tier = tier


class TestDetectionDropsMute(unittest.TestCase):
    def test_birds_do_not_drop(self):
        self.assertFalse(detection_drops_mute([_Det("log"), _Det("log")], []))

    def test_stop_and_slow_tiers_drop(self):
        self.assertTrue(detection_drops_mute([_Det("log"), _Det("stop")], []))
        self.assertTrue(detection_drops_mute([_Det("slow")], []))

    def test_persons_drop(self):
        self.assertTrue(detection_drops_mute([], [object()]))

    def test_empty(self):
        self.assertFalse(detection_drops_mute([], []))
        self.assertFalse(detection_drops_mute(None, None))


def _mute(**kw):
    base = dict(enabled=True, timeout_s=900.0, absent_release_s=0.5,
                max_distance_m=CAP, min_near_share=MIN, hold_near_share=0.70)
    base.update(kw)
    return NettingMute(**base)


def _engage(m, now=0.0):
    ok, why = m.request_on(True, True, 0.41, 0.95)
    assert ok, why
    s = m.update(now, True, True, 0.41, 0.95, False)
    assert s.active and s.event == "active", s
    return s


class TestNettingMuteStateMachine(unittest.TestCase):
    def test_tap_needs_armed_manual_and_the_netting_in_view(self):
        m = _mute()
        self.assertEqual(m.request_on(False, True, 0.41, 0.95), (False, "robot must be armed"))
        self.assertEqual(m.request_on(True, False, 0.41, 0.95), (False, "MANUAL only"))
        ok, why = m.request_on(True, True, 2.5, 0.05)
        self.assertFalse(ok)
        self.assertIn("no netting in view", why)
        ok, why = m.request_on(True, True, 0.65, 0.99)      # a wall just outside the band
        self.assertFalse(ok)
        ok, why = m.request_on(True, True, None, 0.95)      # stale / no reading
        self.assertFalse(ok)
        ok, why = m.request_on(True, True, 0.41, 0.80)      # share below the tap threshold
        self.assertFalse(ok)
        self.assertFalse(m.engaged)
        s = m.update(0.0, True, True, 0.41, 0.95, False)   # nothing queued
        self.assertFalse(s.engaged)
        self.assertIsNone(s.event)

    def test_disabled_refuses(self):
        m = _mute(enabled=False)
        ok, why = m.request_on(True, True, 0.41, 0.95)
        self.assertFalse(ok)
        self.assertIn("disabled", why)

    def test_tap_is_applied_by_update_and_is_active_at_once(self):
        m = _mute()
        ok, why = m.request_on(True, True, 0.41, 0.95)
        self.assertEqual((ok, why), (True, "engaged"))
        self.assertFalse(m.engaged)     # queued only: the web thread never writes state
        s = m.update(10.0, True, True, 0.41, 0.95, False)
        self.assertTrue(s.engaged and s.active)
        self.assertEqual(s.event, "active")
        self.assertEqual(s.since_s, 0.0)
        self.assertEqual(m.request_on(True, True, 0.41, 0.95), (True, "already on"))

    def test_update_rechecks_the_tap(self):
        # Provisional OK on the web thread, but the field is gone by the
        # time the control thread applies it: refused, with the reason.
        m = _mute()
        self.assertTrue(m.request_on(True, True, 0.41, 0.95)[0])
        s = m.update(1.0, True, True, 1.2, 0.3, False)
        self.assertFalse(s.engaged)
        self.assertEqual(s.event, "refused")
        self.assertIn("no netting in view", s.refusal)
        s = m.update(1.1, True, True, 0.41, 0.95, False)   # no re-arm later
        self.assertFalse(s.engaged)

    def test_far_reading_drops_on_that_tick(self):
        # Netting lifts and the next frame is a fence at 0.8 m: no hold.
        m = _mute()
        _engage(m)
        s = m.update(0.1, True, True, 0.8, 0.9, False)
        self.assertFalse(s.active)
        self.assertEqual(s.drop_reason, "near field gone")
        self.assertEqual(s.event, "dropped")
        m2 = _mute()
        _engage(m2)
        s = m2.update(0.1, True, True, 1.2, 0.4, False)
        self.assertFalse(s.active)
        m3 = _mute()
        _engage(m3)
        s = m3.update(0.1, True, True, 0.56, 0.99, False)  # just past the cap
        self.assertFalse(s.active)

    def test_dropout_hold_then_drop(self):
        m = _mute(absent_release_s=0.5)
        _engage(m)
        s = m.update(0.1, True, True, None, None, False)       # no reading
        self.assertTrue(s.active)
        s = m.update(0.5, True, True, float("inf"), 0.0, False)  # clear corridor: still a dropout
        self.assertTrue(s.active)
        s = m.update(0.59, True, True, None, None, False)
        self.assertTrue(s.active)
        s = m.update(0.6, True, True, None, None, False)        # 0.5 s since 0.1
        self.assertFalse(s.active)
        self.assertEqual(s.drop_reason, "near field gone")

    def test_dropout_hold_resets_when_the_field_returns(self):
        m = _mute(absent_release_s=0.5)
        _engage(m)
        m.update(0.1, True, True, None, None, False)
        s = m.update(0.4, True, True, 0.45, 0.9, False)
        self.assertTrue(s.active)
        s = m.update(0.85, True, True, None, None, False)
        self.assertTrue(s.active)                                 # clock restarted at 0.85
        s = m.update(1.34, True, True, None, None, False)
        self.assertTrue(s.active)
        s = m.update(1.35, True, True, None, None, False)
        self.assertFalse(s.active)

    def test_share_hysteresis_holds_to_0_70_then_dropout_rules(self):
        m = _mute(absent_release_s=0.5)
        _engage(m)
        s = m.update(0.1, True, True, 0.45, 0.75, False)       # below the tap share, above the hold share
        self.assertTrue(s.active)
        self.assertIsNone(m._absent_since)
        s = m.update(0.25, True, True, 0.45, 0.65, False)      # below the hold share: in-band dip
        self.assertTrue(s.active)
        s = m.update(0.74, True, True, 0.45, 0.65, False)
        self.assertTrue(s.active)
        s = m.update(0.75, True, True, 0.45, 0.65, False)      # 0.5 s (exact in binary)
        self.assertFalse(s.active)

    def test_detection_drops_and_needs_a_new_tap(self):
        m = _mute()
        _engage(m)
        s = m.update(0.2, True, True, 0.41, 0.95, detection_present=True)
        self.assertFalse(s.active)
        self.assertEqual(s.drop_reason, "person or animal detected")
        s = m.update(5.0, True, True, 0.41, 0.95, False)         # no resume
        self.assertFalse(s.engaged)
        self.assertTrue(m.request_on(True, True, 0.41, 0.95)[0])
        s = m.update(6.0, True, True, 0.41, 0.95, False)
        self.assertTrue(s.active)

    def test_disarm_and_mode_change_drop(self):
        m = _mute()
        _engage(m)
        s = m.update(0.2, False, True, 0.41, 0.95, False)
        self.assertEqual((s.engaged, s.drop_reason), (False, "disarmed"))
        _engage(m, 1.0)
        s = m.update(1.2, True, False, 0.41, 0.95, False)
        self.assertEqual((s.engaged, s.drop_reason), (False, "left MANUAL"))

    def test_timeout(self):
        m = _mute(timeout_s=900.0)
        _engage(m)
        s = m.update(899.9, True, True, 0.41, 0.95, False)
        self.assertTrue(s.active)
        s = m.update(900.0, True, True, 0.41, 0.95, False)
        self.assertFalse(s.engaged)
        self.assertEqual(s.drop_reason, "timeout")

    def test_operator_off_applied_on_the_control_thread(self):
        m = _mute()
        _engage(m)
        self.assertEqual(m.request_off(), (True, "dropped"))
        self.assertTrue(m.engaged)                                # queued only
        s = m.update(0.2, True, True, 0.41, 0.95, False)
        self.assertFalse(s.engaged)
        self.assertFalse(s.active)
        self.assertEqual(s.drop_reason, "operator")
        self.assertEqual(s.event, "dropped")
        self.assertEqual(m.request_off(), (True, "already off"))

    def test_off_cancels_a_queued_tap(self):
        # On then off inside one control tick: nothing engages.
        m = _mute()
        self.assertTrue(m.request_on(True, True, 0.41, 0.95)[0])
        self.assertEqual(m.request_off(), (True, "cancelled"))
        s = m.update(0.1, True, True, 0.41, 0.95, False)
        self.assertFalse(s.engaged)
        self.assertIsNone(s.event)

    def test_same_tick_off_never_leaves_active_true(self):
        # The review's race: an operator drop landing while update() runs.
        m = _mute()
        _engage(m)
        stop = threading.Event()
        results = []

        def hammer():
            while not stop.is_set():
                m.request_off()
                m.request_on(True, True, 0.41, 0.95)

        t = threading.Thread(target=hammer, daemon=True)
        t.start()
        for i in range(2000):
            s = m.update(1.0 + i * 0.001, True, True, 0.41, 0.95, False)
            results.append((s.engaged, s.active))
        stop.set()
        t.join(timeout=2.0)
        for engaged, active in results:
            self.assertEqual(engaged, active)

    def test_as_dict(self):
        m = _mute()
        _engage(m)
        d = m.update(2.0, True, True, 0.41, 0.95, False).as_dict()
        self.assertEqual(d["engaged"], True)
        self.assertEqual(d["active"], True)
        self.assertEqual(d["since_s"], 2.0)
        self.assertIsNone(d["event"])
        self.assertIsNone(d["refusal"])


class TestCorridorMutedScale(unittest.TestCase):
    def setUp(self):
        self.cfg = default_config.obstacle_avoidance
        self.oa = ObstacleAvoidanceController(self.cfg)

    def test_manual_muted_ignores_the_corridor(self):
        self.assertLess(self.oa.compute_throttle_scale(0.41, 0.0, is_manual=True), 0.5)
        self.assertEqual(self.oa.compute_throttle_scale(0.41, 0.0, is_manual=True, corridor_muted=True), 1.0)
        self.assertEqual(self.oa.compute_throttle_scale(0.36, 0.0, is_manual=True, corridor_muted=True), 1.0)
        self.assertEqual(self.oa.get_status()["obstacle_distance_m"], 0.36)

    def test_autonomous_modes_never_muted(self):
        self.assertEqual(self.oa.compute_throttle_scale(0.41, 0.0, is_manual=False, corridor_muted=True),
                         self.oa.compute_throttle_scale(0.41, 0.0, is_manual=False, corridor_muted=False))
        self.assertLess(self.oa.compute_throttle_scale(0.41, 0.0, is_manual=False, corridor_muted=True), 0.05)

    def test_yolo_stop_channel_never_muted(self):
        self.assertEqual(self.oa.compute_throttle_scale(0.0, 0.0, is_manual=True, corridor_muted=True), 0.0)

    def test_stale_depth_never_muted(self):
        stale = self.cfg.stale_timeout_s + 1.0
        self.assertEqual(self.oa.compute_throttle_scale(0.41, stale, is_manual=True, corridor_muted=True),
                         self.cfg.manual_stale_throttle_scale)


class FakeMotor:
    def set_tracks(self, left_byte, right_byte):
        pass

    def stop(self):
        pass

    def get_telemetry(self):
        return None


class FakeRelay:
    def set_armed(self, armed):
        pass


class FakeShutdown:
    def schedule_shutdown(self, delay_seconds):
        pass


def _rc(ch3=1900):
    return RCInputs(ch1_us=1800, ch2_us=1800, ch3_us=ch3, ch4_us=1000, ch5_us=1000,
                    last_update_epoch_s=time.time())


class TestControllerWiring(unittest.TestCase):
    def _ctrl(self):
        ctrl = Controller(
            motor_driver=FakeMotor(), arm_relay=FakeRelay(), shutdown_scheduler=FakeShutdown(),
            obstacle_avoidance=ObstacleAvoidanceController(default_config.obstacle_avoidance),
        )
        ctrl._safety_state.is_armed = True
        return ctrl

    def test_manual_drive_with_netting(self):
        clock = {"t": 1000.0}
        with unittest.mock.patch("pi_app.control.controller.time.monotonic", lambda: clock["t"]):
            ctrl = self._ctrl()

            def tick(dist=0.41, share=0.95, age=0.0, det=False, ch3=1900):
                clock["t"] += 1.0 / 30.0
                ctrl.set_obstacle_data(dist, age, near_share=share)
                ctrl.set_detection_present(det)
                _cmd, _ev, telem = ctrl.process(replace(_rc(ch3), last_update_epoch_s=time.time()))
                return telem

            t = tick()
            self.assertLess(t["obstacle_throttle_scale"], 0.5)      # the netting inhibit
            self.assertFalse(t["netting_mute"]["engaged"])
            # A tap with nothing close is refused.
            ctrl.set_obstacle_data(2.5, 0.0, near_share=0.05)
            ok, why = ctrl.request_netting_mute(True)
            self.assertFalse(ok)
            self.assertIn("no netting in view", why)
            t = tick()
            ok, why = ctrl.request_netting_mute(True)
            self.assertTrue(ok, why)
            t = tick()
            self.assertEqual(t["obstacle_throttle_scale"], 1.0)
            self.assertTrue(t["netting_mute"]["active"])
            self.assertTrue(ctrl.netting_mute_active)
            # The YOLO stop channel still stops the robot while muted (a
            # person/animal also drops it; here only the channel).
            t = tick(dist=0.0, share=None)
            self.assertEqual(t["obstacle_throttle_scale"], 0.0)
            t = tick()
            self.assertEqual(t["obstacle_throttle_scale"], 1.0)
            # The netting lifts: a fence at 0.8 m is seen on THAT tick.
            t = tick(dist=0.8, share=0.9)
            self.assertFalse(t["netting_mute"]["active"])
            self.assertEqual(t["netting_mute"]["drop_reason"], "near field gone")
            self.assertLess(t["obstacle_throttle_scale"], 0.8)
            self.assertGreater(t["obstacle_throttle_scale"], 0.5)   # the MANUAL curve at 0.8 m
            # And no resume when the netting-like reading returns.
            t = tick()
            self.assertFalse(t["netting_mute"]["engaged"])
            self.assertLess(t["obstacle_throttle_scale"], 0.5)
            # Re-tap; a person or animal drops it.
            self.assertTrue(ctrl.request_netting_mute(True)[0])
            tick()
            t = tick(det=True)
            self.assertFalse(t["netting_mute"]["engaged"])
            self.assertEqual(t["netting_mute"]["drop_reason"], "person or animal detected")
            # Re-tap; a stale depth is a dropout: scale goes to the stale
            # floor at once (never muted), the mute drops after 0.5 s.
            self.assertTrue(ctrl.request_netting_mute(True)[0])
            tick()
            t = tick(age=5.0)
            self.assertEqual(t["obstacle_throttle_scale"],
                             default_config.obstacle_avoidance.manual_stale_throttle_scale)
            self.assertTrue(t["netting_mute"]["active"])
            for _ in range(16):
                t = tick(age=5.0)
            self.assertFalse(t["netting_mute"]["engaged"])
            # While the depth is stale a tap is refused; fresh again, it is not.
            self.assertFalse(ctrl.request_netting_mute(True)[0])
            tick()
            # Re-tap; disarm drops it.
            self.assertTrue(ctrl.request_netting_mute(True)[0])
            tick()
            t = tick(ch3=1000)
            self.assertFalse(t["netting_mute"]["engaged"])
            self.assertEqual(t["netting_mute"]["drop_reason"], "disarmed")

    def test_request_refused_outside_manual_or_disarmed(self):
        clock = {"t": 1000.0}
        with unittest.mock.patch("pi_app.control.controller.time.monotonic", lambda: clock["t"]):
            ctrl = self._ctrl()
            ctrl.set_obstacle_data(0.41, 0.0, near_share=0.95)
            ctrl._mode = "FOLLOW_ME"
            self.assertEqual(ctrl.request_netting_mute(True), (False, "MANUAL only"))
            ctrl._mode = "MANUAL"
            ctrl._safety_state.is_armed = False
            self.assertEqual(ctrl.request_netting_mute(True), (False, "robot must be armed"))
            self.assertEqual(ctrl.request_netting_mute(False), (True, "already off"))
            ctrl._safety_state.is_armed = True
            ctrl.set_obstacle_data(0.41, 5.0, near_share=0.95)   # stale depth: no tap
            ok, why = ctrl.request_netting_mute(True)
            self.assertFalse(ok)


class _FakeController:
    def __init__(self):
        self.calls = []

    def request_netting_mute(self, on):
        self.calls.append(on)
        return (True, "engaged") if on else (True, "dropped")


@unittest.skipIf(flask is None, "flask not installed")
class TestEndpoint(unittest.TestCase):
    def _client(self, controller):
        app = create_app(None, OakWebViewerConfig(), controller=controller)
        app.testing = True
        return app.test_client()

    def test_on_off_and_validation(self):
        fake = _FakeController()
        c = self._client(fake)
        r = c.post("/api/manual/netting_mute", data=json.dumps({"on": True}),
                   content_type="application/json")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(json.loads(r.data)["reason"], "engaged")
        r = c.post("/api/manual/netting_mute", data=json.dumps({"on": False}),
                   content_type="application/json")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(fake.calls, [True, False])
        r = c.post("/api/manual/netting_mute", data=json.dumps({"on": "yes"}),
                   content_type="application/json")
        self.assertEqual(r.status_code, 400)
        r = c.post("/api/manual/netting_mute", data="nope", content_type="text/plain")
        self.assertEqual(r.status_code, 400)
        self.assertEqual(fake.calls, [True, False])

    def test_refusal_is_409(self):
        class Refusing:
            def request_netting_mute(self, on):
                return (False, "robot must be armed")
        c = self._client(Refusing())
        r = c.post("/api/manual/netting_mute", data=json.dumps({"on": True}),
                   content_type="application/json")
        self.assertEqual(r.status_code, 409)
        self.assertEqual(json.loads(r.data)["reason"], "robot must be armed")

    def test_no_controller_is_503(self):
        c = self._client(None)
        r = c.post("/api/manual/netting_mute", data=json.dumps({"on": True}),
                   content_type="application/json")
        self.assertEqual(r.status_code, 503)


if __name__ == "__main__":
    unittest.main()
