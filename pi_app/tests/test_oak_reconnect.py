"""Tests for the OAK-D USB auto-reconnect supervisor.

Documented bug (CLAUDE.md, now resolved): "OAK USB disconnect: no auto-reconnect;
service restart required if OAK re-enumerates." A USB drop mid-run used to leave
the worker thread spinning a dead queue forever (each _poll_* swallowed the
X_LINK / communication error and returned), so depth and Follow-Me stayed down
until a human restarted the service.

The fix wraps the device-session lifecycle in a supervisor loop inside the worker
thread (OakDepthReader._run_pipeline):

  build pipeline + open device  ->  run poll loops  ->  on fatal comm error:
  close defensively, mark disconnected, back off (interruptible), rebuild.

These tests mock depthai entirely (no hardware) and cover:

  1. A queue raising a communication error ends the session, the device is closed,
     and a *new* device/pipeline is built (factory-invocation count grows);
     reconnect_count increments.
  2. get_health() reports connected=False during the gap and True once a session
     is live again.
  3. stop() during the backoff sleep exits promptly (well under the backoff window).
  4. The min-distance timestamp is NOT advanced while the device is down — the
     staleness fail-safe keeps stopping autonomous motion through the outage.
"""

import sys
import threading
import time
import types
import unittest

import numpy as np

# ---------------------------------------------------------------------------
# Mock depthai entirely (no OAK hardware in CI). OakDepthReader._run_pipeline
# does `import depthai as dai` and bails out if it is missing, so a stub module
# must exist for the supervisor loop to run. These tests patch
# _run_pipeline_once with fake sessions, so the stub only needs to import — none
# of its attributes are exercised. (conftest.py uses the same sys.modules
# stubbing pattern for other optional hardware deps.)
# ---------------------------------------------------------------------------
if "depthai" not in sys.modules:
    _fake_dai = types.ModuleType("depthai")

    class _FakeDevice:
        @staticmethod
        def getAllAvailableDevices():
            return []

    _fake_dai.Device = _FakeDevice
    sys.modules["depthai"] = _fake_dai

from config import ObstacleAvoidanceConfig, FollowMeConfig
from pi_app.hardware.oak_depth import OakDepthReader


# ---------------------------------------------------------------------------
# A "communication" error that mimics how depthai surfaces a USB drop:
# a plain RuntimeError whose message contains "Communication exception".
# OakDepthReader._is_fatal_comm_error classifies these as fatal.
# ---------------------------------------------------------------------------

def _comm_error() -> RuntimeError:
    return RuntimeError(
        "Communication exception - possible device error/misconfiguration. "
        "Original message 'X_LINK_ERROR'"
    )


def _make_reader(*, stale_timeout_s=0.5, stale_policy="stop") -> OakDepthReader:
    obs_cfg = ObstacleAvoidanceConfig(
        slow_distance_m=1.5,
        stop_distance_m=0.4,
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=0.0,          # rectangular ROI path — simplest for tests
        min_depth_mm=350,
        min_valid_pct=8.0,
        update_rate_hz=1000.0,      # fast loop so sessions cycle quickly in tests
        stale_timeout_s=stale_timeout_s,
        stale_policy=stale_policy,
        safety_stop_radius_m=0.8,
        camera_hfov_deg=73.0,
    )
    return OakDepthReader(obstacle_config=obs_cfg, follow_me_config=FollowMeConfig())


# ---------------------------------------------------------------------------
# Test 1 — fatal comm error classification + propagation, and the supervisor
#          rebuilds a fresh device after each drop (factory-invocation count).
# ---------------------------------------------------------------------------

class TestFatalErrorEndsSessionAndRebuilds(unittest.TestCase):

    def test_comm_error_is_classified_fatal(self):
        self.assertTrue(OakDepthReader._is_fatal_comm_error(_comm_error()))
        self.assertTrue(
            OakDepthReader._is_fatal_comm_error(RuntimeError("X_LINK_ERROR")))
        # A frame-level glitch must NOT be treated as fatal.
        self.assertFalse(
            OakDepthReader._is_fatal_comm_error(ValueError("bad percentile input")))

    def test_poll_depth_reraises_fatal_comm_error(self):
        """A queue.tryGet() raising a comm error inside _poll_depth must propagate
        (so the session loop ends) — it must NOT be swallowed like a bad frame."""
        reader = _make_reader()

        class _RaisingQueue:
            def tryGet(self):
                raise _comm_error()

        with self.assertRaises(RuntimeError):
            reader._poll_depth(_RaisingQueue(), _RaisingQueue(), np)

    def test_poll_depth_swallows_nonfatal_error(self):
        """A non-fatal error inside _poll_depth stays swallowed (no raise)."""
        reader = _make_reader()

        class _BadFrame:
            def getFrame(self):
                raise ValueError("decode glitch")  # not a comm error

        class _OneShotQueue:
            def __init__(self):
                self._served = False

            def tryGet(self):
                if not self._served:
                    self._served = True
                    return _BadFrame()
                return None

        # Should not raise — non-fatal errors are recorded and swallowed.
        reader._poll_depth(_OneShotQueue(), _OneShotQueue(), np)
        self.assertIn("poll_depth", reader.get_health()["last_depth_error"] or "")

    def test_supervisor_rebuilds_device_after_each_drop(self):
        """Drive the real supervisor loop with a fake session that opens a fresh
        "device", polls once, then drops with a comm error. Each drop must cause
        the supervisor to build a NEW device (factory count grows) and bump
        reconnect_count."""
        reader = _make_reader()
        reader._RECONNECT_BACKOFF_S = (0.01, 0.01, 0.01)  # tiny backoff for speed

        device_builds = []          # one entry per device "opened"
        target_sessions = 3

        def fake_run_once(dai, npmod):
            # Simulate building + opening a device session.
            device_id = len(device_builds) + 1
            device_builds.append(device_id)
            # Mark the session live exactly as the real code does on open.
            now = time.monotonic()
            with reader._lock:
                reader._pipeline_running = True
                reader._connected = True
                reader._last_pipeline_loop_ts = now
            # After enough rebuilds, request stop so the loop terminates.
            if len(device_builds) >= target_sessions:
                reader._stop_event.set()
            # Session drops: defensively close, mark not-connected, return True
            # (a live device went away — the supervisor should count a reconnect).
            with reader._lock:
                reader._pipeline_running = False
                reader._connected = False
            return True

        reader._run_pipeline_once = fake_run_once  # type: ignore[assignment]
        reader._run_pipeline()

        self.assertEqual(
            len(device_builds), target_sessions,
            "Supervisor must build a fresh device after each disconnect",
        )
        # First session is not a reconnect; each subsequent rebuild is.
        self.assertEqual(reader.get_health()["reconnect_count"], target_sessions - 1)

    def test_build_failure_does_not_count_as_reconnect(self):
        """If a session never opens (build/open failure -> returns False), the
        supervisor backs off but must NOT increment reconnect_count."""
        reader = _make_reader()
        reader._RECONNECT_BACKOFF_S = (0.01, 0.01, 0.01)
        calls = []

        def fake_run_once(dai, npmod):
            calls.append(1)
            if len(calls) >= 3:
                reader._stop_event.set()
            return False  # never opened

        reader._run_pipeline_once = fake_run_once  # type: ignore[assignment]
        reader._run_pipeline()

        self.assertEqual(len(calls), 3)
        self.assertEqual(reader.get_health()["reconnect_count"], 0)


# ---------------------------------------------------------------------------
# Test 2 — health reflects connected=False during the gap, True when live.
# ---------------------------------------------------------------------------

class TestHealthConnectedFlag(unittest.TestCase):

    def test_health_has_new_fields_and_defaults(self):
        reader = _make_reader()
        h = reader.get_health()
        for key in ("connected", "reconnect_count", "last_disconnect_ts"):
            self.assertIn(key, h)
        self.assertFalse(h["connected"])
        self.assertEqual(h["reconnect_count"], 0)
        self.assertIsNone(h["last_disconnect_ts"])

    def test_connected_false_during_gap_then_true_when_live(self):
        """Step through the supervisor manually: connected must read False after a
        drop and True while a session is live."""
        reader = _make_reader()
        reader._RECONNECT_BACKOFF_S = (0.02, 0.02, 0.02)

        observed = {"connected_while_live": None}
        sessions = []

        def fake_run_once(dai, npmod):
            sessions.append(1)
            # Open: device live.
            with reader._lock:
                reader._pipeline_running = True
                reader._connected = True
                reader._last_pipeline_loop_ts = time.monotonic()
            observed["connected_while_live"] = reader.get_health()["connected"]
            # Drop. Do NOT set stop here, so the supervisor runs its disconnect
            # bookkeeping (records last_disconnect_ts). Stop on the 2nd session.
            with reader._lock:
                reader._pipeline_running = False
                reader._connected = False
            if len(sessions) >= 2:
                reader._stop_event.set()
            return True

        reader._run_pipeline_once = fake_run_once  # type: ignore[assignment]

        # Before any session: not connected.
        self.assertFalse(reader.get_health()["connected"])

        reader._run_pipeline()

        self.assertTrue(observed["connected_while_live"],
                        "health.connected must be True while a session is live")
        # After the run loop exits, connected is False and a disconnect ts is set
        # (the first session's drop was bookkept before the 2nd session stopped).
        h = reader.get_health()
        self.assertFalse(h["connected"])
        self.assertIsNotNone(h["last_disconnect_ts"])
        self.assertGreaterEqual(h["reconnect_count"], 1)


# ---------------------------------------------------------------------------
# Test 3 — stop() during backoff exits promptly (interruptible Event.wait).
# ---------------------------------------------------------------------------

class TestStopInterruptsBackoff(unittest.TestCase):

    def test_stop_during_backoff_returns_quickly(self):
        """With a long backoff window, calling stop() must wake the supervisor
        immediately rather than waiting out the full window."""
        reader = _make_reader()
        # A deliberately long backoff: if stop() did not interrupt the wait, the
        # supervisor thread would block for ~30 s.
        reader._RECONNECT_BACKOFF_S = (30.0, 30.0, 30.0)

        entered_backoff = threading.Event()

        def fake_run_once(dai, npmod):
            # One short "live" session, then drop into the long backoff once.
            with reader._lock:
                reader._connected = True
                reader._pipeline_running = True
            with reader._lock:
                reader._connected = False
                reader._pipeline_running = False
            entered_backoff.set()
            return True

        reader._run_pipeline_once = fake_run_once  # type: ignore[assignment]

        t = threading.Thread(target=reader._run_pipeline, daemon=True)
        t.start()

        # Wait until the supervisor has finished the first session and is about to
        # (or has just entered) the 30 s backoff.
        self.assertTrue(entered_backoff.wait(timeout=2.0))
        # Give it a beat to actually be inside _stop_event.wait(30).
        time.sleep(0.05)

        t0 = time.monotonic()
        reader._stop_event.set()
        t.join(timeout=2.0)
        elapsed = time.monotonic() - t0

        self.assertFalse(t.is_alive(), "supervisor thread must exit after stop()")
        self.assertLess(elapsed, 1.0,
                        "stop() during backoff must interrupt the sleep promptly")


# ---------------------------------------------------------------------------
# Test 4 — the depth timestamp does NOT advance while disconnected, so the
#          staleness fail-safe keeps stopping motion through the outage.
# ---------------------------------------------------------------------------

class TestStalenessHoldsDuringOutage(unittest.TestCase):

    def test_min_distance_timestamp_does_not_advance_while_disconnected(self):
        """Seed a fresh depth reading, then run the supervisor through a drop +
        reconnect cycle whose fake sessions never touch _depth_state. The
        min-distance age must keep growing across the outage (no fake-fresh)."""
        reader = _make_reader()
        reader._RECONNECT_BACKOFF_S = (0.02, 0.02, 0.02)

        # Seed a "fresh" depth reading as if a real frame had just arrived.
        seed_ts = time.monotonic()
        with reader._lock:
            reader._depth_state.min_distance_m = 1.0
            reader._depth_state.timestamp = seed_ts

        sessions = []

        def fake_run_once(dai, npmod):
            # A device session that opens and drops WITHOUT ever publishing a
            # depth frame (exactly what happens during a USB outage).
            sessions.append(1)
            with reader._lock:
                reader._connected = True
                reader._pipeline_running = True
            with reader._lock:
                reader._connected = False
                reader._pipeline_running = False
            if len(sessions) >= 2:
                reader._stop_event.set()
            return True

        reader._run_pipeline_once = fake_run_once  # type: ignore[assignment]
        reader._run_pipeline()

        # The depth-state timestamp must be UNCHANGED (no session faked a frame).
        with reader._lock:
            ts_after = reader._depth_state.timestamp
        self.assertEqual(ts_after, seed_ts,
                         "depth timestamp must not be advanced while the device is down")

        # get_min_distance() age has therefore grown — the staleness window will
        # fire and stop autonomous motion.
        _, age_s = reader.get_min_distance()
        self.assertGreater(age_s, 0.0)

    def test_stale_age_after_outage_stops_motion(self):
        """End-to-end: a depth reading that goes stale during the outage yields a
        full stop from the obstacle controller's stale policy."""
        from pi_app.control.obstacle_avoidance import ObstacleAvoidanceController

        reader = _make_reader(stale_timeout_s=0.2, stale_policy="stop")
        oa = ObstacleAvoidanceController(reader._obs_cfg)

        # Last real frame was a clear corridor (inf) well outside the stale window.
        with reader._lock:
            reader._depth_state.min_distance_m = float("inf")
            reader._depth_state.timestamp = time.monotonic() - 5.0

        dist_m, age_s = reader.get_min_distance()
        scale = oa.compute_throttle_scale(dist_m, age_s)
        self.assertAlmostEqual(
            scale, 0.0,
            msg="A stale depth reading during the outage must force a full stop",
        )


if __name__ == "__main__":
    unittest.main()
