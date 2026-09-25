"""Controller wiring for the log-only pump detector.

A would-start must not change motor bytes, mode, or follow-me calls.
The detector result is telemetry and a warning line.
"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from config import config as default_config
from pi_app.control.controller import Controller, RCInputs
from pi_app.control.follow_me import FollowMeController, PersonDetection
from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.control.safety import SafetyParams
from pi_app.tests.test_arms_up_twitch import FakeMotor, FakeRelay, FakeShutdown
from pi_app.tests.test_pump_gesture import (
    DT, FRAME_W, INTR, PUMP_W, REST_W, Z, bbox_for,
)


def _cfg(enabled=True):
    pump = replace(default_config.pump, enabled=enabled)
    slew = replace(default_config.slew_limiter, enabled=False)
    return replace(default_config, pump=pump, slew_limiter=slew)


def _rc():
    return RCInputs(
        ch1_us=1500, ch2_us=1500, ch3_us=1900, ch4_us=1000, ch5_us=1000,
        last_update_epoch_s=0.0,
    )


def _person(w_m, z_m, track_id, depth_status="ok", c_m=0.0):
    return PersonDetection(
        x_m=0.0,
        z_m=z_m,
        confidence=0.9,
        bbox=bbox_for(w_m, c_m, z_m),
        track_id=track_id,
        depth_status=depth_status,
    )


class SpyFollowMe:
    """Records the calls process() makes. target_track_id is a pure read."""

    def __init__(self, track_id):
        self._track_id = track_id
        self.calls = []

    def target_track_id(self):
        return self._track_id

    def set_arm_state(self, armed):
        self.calls.append(("set_arm_state", bool(armed)))

    def update_pose(self, heading, left, right, timestamp):
        self.calls.append(("update_pose", heading, left, right, timestamp))

    def update_gps(self, *args):
        self.calls.append(("update_gps", args))

    def update_telemetry(self, **kwargs):
        self.calls.append(("update_telemetry", tuple(sorted(kwargs.items()))))

    def compute(self, detections):
        self.calls.append((
            "compute",
            tuple(
                (d.track_id, d.z_m, d.bbox, d.depth_status)
                for d in detections
            ),
        ))
        return (160, 150)

    def get_status(self, now=None):
        self.calls.append(("get_status", now))
        return {"follow_me_tracking": True}

    def start_recorder(self):
        self.calls.append(("start_recorder",))

    def stop_recorder(self):
        self.calls.append(("stop_recorder",))


class TestPumpController(unittest.TestCase):
    def _make(self, cfg, follow_me=None):
        clock = {"t": 0.0}
        params = SafetyParams(debounce_seconds=0.0)
        motor = FakeMotor()
        with patch("pi_app.control.controller.config", cfg):
            with patch(
                "pi_app.control.controller.time.monotonic",
                side_effect=lambda: clock["t"],
            ):
                controller = Controller(
                    motor_driver=motor,
                    arm_relay=FakeRelay(),
                    shutdown_scheduler=FakeShutdown(),
                    safety_params=params,
                    follow_me=follow_me,
                )
        return controller, motor, clock

    def _process(self, controller, cfg, clock, t, rc, follow_mode=False):
        clock["t"] = t
        if follow_mode:
            controller._mode = "FOLLOW_ME"
        with patch("pi_app.control.controller.config", cfg):
            with patch(
                "pi_app.control.controller.time.monotonic",
                side_effect=lambda: clock["t"],
            ):
                return controller.process(rc, now_epoch_s=t)

    def test_telemetry_pump_present_with_src(self):
        cfg = _cfg(True)
        controller, _motor, clock = self._make(cfg)
        seen = []

        def getter(w, h):
            seen.append((w, h))
            return INTR

        controller.set_intrinsics_getter(getter)
        controller.set_person_detections([
            _person(REST_W, 4.0, track_id=1),
            _person(REST_W, 2.0, track_id=2),
        ])
        controller.set_person_detections_ts(1.0)
        _cmd, _events, telem = self._process(controller, cfg, clock, 1.0, _rc())
        self.assertIn("pump", telem)
        pump = telem["pump"]
        self.assertEqual(pump["src"], "nearest")
        self.assertEqual(pump["track_id"], 2)
        self.assertEqual(pump["state"], "idle")
        self.assertTrue(pump["accepted"])
        oak = default_config.oak_detection
        self.assertEqual(seen, [(oak.input_width, oak.input_height)])
        self.assertEqual(oak.input_width, FRAME_W)

    def test_follow_me_uses_target_manual_uses_nearest(self):
        cfg = _cfg(True)
        persons = [
            _person(REST_W, 4.0, track_id=7),
            _person(REST_W, 2.0, track_id=2),
            _person(REST_W, 1.1, track_id=9, depth_status="no_support"),
        ]
        spy = SpyFollowMe(7)
        controller, _motor, clock = self._make(cfg, follow_me=spy)
        controller.set_intrinsics_getter(lambda w, h: INTR)
        controller.set_person_detections(persons)
        controller.set_person_detections_ts(1.0)
        _cmd, _events, telem = self._process(
            controller, cfg, clock, 1.0, _rc(), follow_mode=True,
        )
        self.assertEqual(telem["mode"], "FOLLOW_ME")
        self.assertEqual(telem["pump"]["src"], "target")
        self.assertEqual(telem["pump"]["track_id"], 7)

        manual, _motor2, clock2 = self._make(cfg, follow_me=SpyFollowMe(7))
        manual.set_intrinsics_getter(lambda w, h: INTR)
        manual.set_person_detections(persons)
        manual.set_person_detections_ts(1.0)
        _cmd, _events, telem = self._process(manual, cfg, clock2, 1.0, _rc())
        self.assertEqual(telem["mode"], "MANUAL")
        self.assertEqual(telem["pump"]["src"], "nearest")
        self.assertEqual(telem["pump"]["track_id"], 2)

        # FOLLOW_ME with no lock yet falls through to the nearest ok detection.
        unlocked, _m, clock3 = self._make(cfg, follow_me=SpyFollowMe(None))
        unlocked.set_intrinsics_getter(lambda w, h: INTR)
        unlocked.set_person_detections(persons)
        unlocked.set_person_detections_ts(1.0)
        _cmd, _events, telem = self._process(
            unlocked, cfg, clock3, 1.0, _rc(), follow_mode=True,
        )
        self.assertEqual(telem["pump"]["src"], "nearest")
        self.assertEqual(telem["pump"]["track_id"], 2)

        # A locked id that is not in this list is "no detection", not nearest.
        missing, _m, clock4 = self._make(cfg, follow_me=SpyFollowMe(7))
        missing.set_intrinsics_getter(lambda w, h: INTR)
        missing.set_person_detections([_person(REST_W, 2.0, track_id=2)])
        missing.set_person_detections_ts(1.0)
        _cmd, _events, telem = self._process(
            missing, cfg, clock4, 1.0, _rc(), follow_mode=True,
        )
        self.assertEqual(telem["pump"]["src"], "target")
        self.assertFalse(telem["pump"]["accepted"])

    def test_would_start_does_not_change_motion(self):
        frames = [(i * DT, REST_W) for i in range(45)]
        t = 45 * DT
        for _pump in range(2):
            for _ in range(4):
                frames.append((t, PUMP_W))
                t += DT
            for _ in range(3):
                frames.append((t, REST_W))
                t += DT

        def drive(enabled):
            cfg = _cfg(enabled)
            spy = SpyFollowMe(7)
            controller, motor, clock = self._make(cfg, follow_me=spy)
            controller.set_intrinsics_getter(lambda w, h: INTR)
            rc = _rc()
            modes = []
            armed = []
            commands = []
            events = []
            pumps = []
            for ts, width in frames:
                controller.set_person_detections([
                    _person(width, Z, track_id=7),
                ])
                controller.set_person_detections_ts(ts)
                cmd, ev, telem = self._process(
                    controller, cfg, clock, ts, rc, follow_mode=True,
                )
                modes.append(telem.get("mode"))
                armed.append(cmd.is_armed)
                commands.append((cmd.left_byte, cmd.right_byte))
                events.append(tuple(e.name for e in ev))
                pumps.append(dict(telem["pump"]))
            return {
                "motor": list(motor.commands),
                "stops": motor.stops,
                "modes": modes,
                "armed": armed,
                "commands": commands,
                "events": events,
                "calls": spy.calls,
                "slew": (controller._slew_last_left, controller._slew_last_right),
                "safety_armed": controller._safety_state.is_armed,
                "safety_emergency": controller._safety_state.emergency_active,
                "mode": controller._mode,
                "pumps": pumps,
            }

        with self.assertLogs("pi_app.control.controller", level="WARNING") as logged:
            on = drive(True)
        self.assertTrue(any(
            "PUMP would-start (log only):" in rec.getMessage()
            for rec in logged.records
        ))
        starts = [p for p in on["pumps"] if p.get("start_event")]
        self.assertEqual(len(starts), 1)
        self.assertEqual(starts[0]["src"], "target")
        self.assertEqual(starts[0]["track_id"], 7)
        self.assertIn((160, 150), on["commands"])

        with self.assertNoLogs("pi_app.control.controller", level="WARNING"):
            off = drive(False)
        self.assertTrue(all(p == {"enabled": False} for p in off["pumps"]))

        self.assertEqual(on["motor"], off["motor"])
        self.assertEqual(on["stops"], off["stops"])
        self.assertEqual(on["commands"], off["commands"])
        self.assertEqual(on["modes"], off["modes"])
        self.assertEqual(on["armed"], off["armed"])
        self.assertEqual(on["events"], off["events"])
        self.assertEqual(on["calls"], off["calls"])
        self.assertEqual(on["slew"], off["slew"])
        self.assertEqual(on["safety_armed"], off["safety_armed"])
        self.assertEqual(on["safety_emergency"], off["safety_emergency"])
        self.assertEqual(on["mode"], off["mode"])
        self.assertEqual(on["mode"], "FOLLOW_ME")
        self.assertNotEqual(on["commands"], [(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)] * len(frames))

    def test_detector_exception_does_not_break_process(self):
        cfg = _cfg(True)
        healthy, motor_ok, clock_ok = self._make(cfg)
        broken, motor_bad, clock_bad = self._make(cfg)

        def boom(*_args, **_kwargs):
            raise RuntimeError("pump bug")

        broken._pump.update = boom
        rc = _rc()
        with self.assertLogs("pi_app.control.controller", level="ERROR") as logged:
            for t in (0.1, 0.2):
                cmd_ok, _ev, telem_ok = self._process(healthy, cfg, clock_ok, t, rc)
                cmd_bad, _ev, telem_bad = self._process(broken, cfg, clock_bad, t, rc)
                self.assertEqual(
                    (cmd_bad.left_byte, cmd_bad.right_byte),
                    (cmd_ok.left_byte, cmd_ok.right_byte),
                )
                self.assertEqual(telem_bad["mode"], telem_ok["mode"])
                self.assertEqual(telem_bad["pump"]["error"], True)
                self.assertIn("pump", telem_ok)
        self.assertEqual(motor_bad.commands, motor_ok.commands)
        messages = [rec.getMessage() for rec in logged.records]
        self.assertEqual(
            sum("PUMP detector failed" in m for m in messages),
            1,
        )

    def test_target_track_id_none_without_a_lock(self):
        fm = FollowMeController(default_config.follow_me)
        self.assertIsNone(fm.target_track_id())


if __name__ == "__main__":
    unittest.main()
