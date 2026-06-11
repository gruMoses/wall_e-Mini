"""Tests for the calibration wizard safety gate (Bug #1, fix a).

The calibration routines drive the motors directly (bypassing the normal
control path), so the start endpoint must refuse to run unless the rover is
armed, and the running routines must abort on disarm / e-stop / watchdog.
"""

import time
import unittest

import pytest

# Flask is a declared dependency (requirements.txt: flask>=3.0). importorskip
# guards a stripped CI image; in the normal dev/CI env it does NOT skip.
flask = pytest.importorskip("flask")

from pi_app.control.mapping import CENTER_OUTPUT_VALUE
from pi_app.web.calibration_wizard import (
    CalibrationManager,
    create_calibration_blueprint,
)


class FakeController:
    def __init__(self, armed=True, emergency=False):
        self._armed = armed
        self._emergency = emergency
        self.entered = False
        self.exited = False

    @property
    def is_armed(self):
        return self._armed

    @property
    def emergency_active(self):
        return self._emergency

    def enter_calibration_mode(self):
        self.entered = True

    def exit_calibration_mode(self):
        self.exited = True


class FakeMotor:
    def __init__(self):
        self.tracks = []

    def set_tracks(self, left, right):
        self.tracks.append((left, right))

    def stop(self):
        self.tracks.append(("stop", "stop"))


def _make_client(controller):
    app = flask.Flask(__name__)
    mgr = CalibrationManager(controller=controller, motor_driver=FakeMotor())
    app.register_blueprint(create_calibration_blueprint(mgr))
    return app.test_client(), mgr


class TestCalibrationArmGate(unittest.TestCase):
    def test_disarmed_start_returns_400_and_does_not_run(self):
        client, mgr = _make_client(FakeController(armed=False))
        resp = client.post("/api/cal/start", json={"tool": "heading_pid"})
        self.assertEqual(resp.status_code, 400)
        self.assertIn("armed", resp.get_json()["error"].lower())
        self.assertEqual(mgr.state, "IDLE")  # never entered RUNNING

    def test_estop_active_start_returns_400(self):
        client, mgr = _make_client(FakeController(armed=True, emergency=True))
        resp = client.post("/api/cal/start", json={"tool": "heading_pid"})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(mgr.state, "IDLE")

    def test_no_controller_blocks_start(self):
        app = flask.Flask(__name__)
        mgr = CalibrationManager(controller=None, motor_driver=FakeMotor())
        app.register_blueprint(create_calibration_blueprint(mgr))
        resp = app.test_client().post("/api/cal/start", json={"tool": "heading_pid"})
        self.assertEqual(resp.status_code, 400)

    def test_armed_start_is_allowed(self):
        client, mgr = _make_client(FakeController(armed=True))
        resp = client.post("/api/cal/start", json={"tool": "heading_pid"})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json()["status"], "started")
        mgr.cancel()  # stop the worker thread (no IMU → it finishes anyway)


class TestCalibrationAbort(unittest.TestCase):
    def _mgr(self, **kw):
        return CalibrationManager(controller=FakeController(**kw), motor_driver=FakeMotor())

    def test_no_abort_when_armed_and_no_watchdog(self):
        mgr = self._mgr(armed=True)
        mgr._motor_deadline = None
        self.assertFalse(mgr._cancelled())

    def test_disarm_aborts(self):
        mgr = self._mgr(armed=True)
        mgr._motor_deadline = None
        self.assertFalse(mgr._cancelled())
        mgr._controller._armed = False
        self.assertTrue(mgr._cancelled())

    def test_estop_aborts(self):
        mgr = self._mgr(armed=True)
        mgr._motor_deadline = None
        self.assertFalse(mgr._cancelled())
        mgr._controller._emergency = True
        self.assertTrue(mgr._cancelled())

    def test_watchdog_timeout_aborts_and_neutralizes_motor(self):
        mgr = self._mgr(armed=True)
        mgr._motor_deadline = time.monotonic() - 1.0  # already expired
        self.assertTrue(mgr._cancelled())
        # The abort path neutralizes the tracks.
        self.assertIn((CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE), mgr._motor.tracks)

    def test_explicit_cancel_aborts(self):
        mgr = self._mgr(armed=True)
        mgr._motor_deadline = None
        self.assertFalse(mgr._cancelled())
        mgr.cancel()
        self.assertTrue(mgr._cancelled())


if __name__ == "__main__":
    unittest.main()
