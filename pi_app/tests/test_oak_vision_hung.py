"""OakDepthReader.get_health() 'vision_hung' (2026-10-08).

The worker thread hung inside depthai's teardown for four days with
pipeline_running still True, loop_age_s in the hundreds of thousands, and
nothing acting on it. get_health() now reports vision_hung when neither the
session loop nor the supervisor loop has ticked for vision_hang_s (or the
thread died) while nobody asked the reader to stop. A device-absent backoff
(supervisor ticking every 10 s, pipeline not running) must NOT count.
"""

import sys
import time
import types
import unittest
from unittest.mock import patch

import numpy as np  # noqa: F401  (conftest stubs; keep import order stable)

if "depthai" not in sys.modules:
    _fake_dai = types.ModuleType("depthai")

    class _FakeDevice:
        @staticmethod
        def getAllAvailableDevices():
            return []

    _fake_dai.Device = _FakeDevice
    sys.modules["depthai"] = _fake_dai

from config import FollowMeConfig, OakDetectionConfig, ObstacleAvoidanceConfig
from pi_app.hardware.oak_depth import OakDepthReader
from pi_app.hardware import oak_depth as oak_depth_mod


class _FakeThread:
    def __init__(self, alive: bool) -> None:
        self._alive = alive

    def is_alive(self) -> bool:
        return self._alive


def _make_reader(hang_s=30.0) -> OakDepthReader:
    obs = ObstacleAvoidanceConfig(
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=0.0,
        min_depth_mm=350,
        min_valid_pct=8.0,
        update_rate_hz=15.0,
        camera_hfov_deg=70.0,
    )
    return OakDepthReader(
        obstacle_config=obs,
        follow_me_config=FollowMeConfig(),
        detection_config=OakDetectionConfig(vision_hang_s=hang_s),
    )


class TestVisionHung(unittest.TestCase):
    def test_config_defaults(self):
        cfg = OakDetectionConfig()
        self.assertEqual(cfg.vision_hang_s, 30.0)
        self.assertEqual(cfg.vision_hang_restart_s, 60.0)
        self.assertEqual(cfg.vision_hang_log_interval_s, 60.0)
        # The restart waits longer than the detection, and the detection is
        # far above any legitimate session-loop or device-boot pause.
        self.assertGreater(cfg.vision_hang_restart_s, 0.0)
        self.assertGreater(cfg.vision_hang_s, 10.0)

    def test_not_started_is_not_hung(self):
        r = _make_reader()
        h = r.get_health()
        self.assertFalse(h["vision_hung"])
        self.assertFalse(h["vision_thread_alive"])
        self.assertIsNone(h["vision_worker_age_s"])

    def test_the_incident_shape_is_hung(self):
        # pipeline_running frozen True, session loop last ticked long ago,
        # supervisor never ran since (it cannot: the thread is in C++).
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = True
            r._connected = True
            r._last_pipeline_loop_ts = now - 100.0
            r._last_supervisor_ts = now - 200.0
        h = r.get_health()
        self.assertTrue(h["vision_hung"])
        self.assertTrue(h["pipeline_running"])
        self.assertTrue(h["vision_thread_alive"])
        self.assertGreaterEqual(h["vision_worker_age_s"], 99.0)

    def test_live_session_is_not_hung(self):
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = True
            r._last_pipeline_loop_ts = now - 0.05
            r._last_supervisor_ts = now - 500.0
        self.assertFalse(r.get_health()["vision_hung"])

    def test_device_absent_backoff_is_not_hung(self):
        # Pipeline not running, session loop stale for minutes, but the
        # supervisor ticked 5 s ago (10 s re-enumeration poll).
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = False
            r._last_pipeline_loop_ts = now - 300.0
            r._last_supervisor_ts = now - 5.0
        h = r.get_health()
        self.assertFalse(h["vision_hung"])
        self.assertTrue(h["loop_stale"])  # still stale for the fail-safe

    def test_supervisor_stuck_in_build_is_hung(self):
        # Neither loop ticked for 2 minutes, pipeline not running: a hang in
        # the build/open path (or a stop() that never returned).
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = False
            r._last_pipeline_loop_ts = now - 120.0
            r._last_supervisor_ts = now - 120.0
        self.assertTrue(r.get_health()["vision_hung"])

    def test_dead_thread_is_hung(self):
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=False)
        with r._lock:
            r._pipeline_running = False
            r._last_pipeline_loop_ts = now - 1.0
            r._last_supervisor_ts = now - 1.0
        h = r.get_health()
        self.assertTrue(h["vision_hung"])
        self.assertFalse(h["vision_thread_alive"])

    def test_stop_requested_is_not_hung(self):
        r = _make_reader()
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = True
            r._last_pipeline_loop_ts = now - 100.0
            r._last_supervisor_ts = now - 100.0
        r._stop_event.set()
        self.assertFalse(r.get_health()["vision_hung"])

    def test_threshold_boundary(self):
        r = _make_reader(hang_s=30.0)
        now = time.monotonic()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = True
            r._last_pipeline_loop_ts = now - 29.0
            r._last_supervisor_ts = 0.0
        self.assertFalse(r.get_health()["vision_hung"])
        with r._lock:
            r._last_pipeline_loop_ts = now - 31.0
        self.assertTrue(r.get_health()["vision_hung"])

    def test_disabled_at_zero(self):
        r = _make_reader(hang_s=0.0)
        now = time.monotonic()
        r._thread = _FakeThread(alive=False)
        with r._lock:
            r._pipeline_running = True
            r._last_pipeline_loop_ts = now - 100000.0
            r._last_supervisor_ts = now - 100000.0
        self.assertFalse(r.get_health()["vision_hung"])

    def test_supervisor_heartbeat_is_stamped_by_run_pipeline(self):
        # _run_pipeline stamps the supervisor heartbeat before each session
        # attempt and after it ends, so a device-absent loop keeps it fresh.
        r = _make_reader()
        calls = {"n": 0, "ts_at_entry": []}

        def fake_once(dai, np_):
            calls["n"] += 1
            with r._lock:
                calls["ts_at_entry"].append(r._last_supervisor_ts)
            if calls["n"] >= 2:
                r._stop_event.set()
            return False

        r._run_pipeline_once = fake_once  # type: ignore[assignment]
        r._RECONNECT_BACKOFF_S = (0.01,)  # type: ignore[attr-defined]
        before = time.monotonic()
        r._run_pipeline()
        with r._lock:
            ts = r._last_supervisor_ts
        self.assertGreaterEqual(ts, before)
        self.assertEqual(calls["n"], 2)
        # The stamp is fresh when each session attempt STARTS, so a long
        # build/open inside _run_pipeline_once ages from a recent heartbeat.
        for entry_ts in calls["ts_at_entry"]:
            self.assertGreaterEqual(entry_ts, before)

    def test_exactly_at_threshold_is_not_hung(self):
        r = _make_reader(hang_s=30.0)
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = True
        with patch.object(oak_depth_mod.time, "monotonic", return_value=1000.0):
            with r._lock:
                r._last_pipeline_loop_ts = 970.0   # age exactly 30.0
                r._last_supervisor_ts = 0.0
            self.assertFalse(r.get_health()["vision_hung"])
            with r._lock:
                r._last_pipeline_loop_ts = 969.999
            self.assertTrue(r.get_health()["vision_hung"])

    def test_alive_thread_with_no_stamp_yet_is_not_hung(self):
        # Between Thread.start() and the first heartbeat (a few microseconds).
        r = _make_reader()
        r._thread = _FakeThread(alive=True)
        with r._lock:
            r._pipeline_running = False
            r._last_pipeline_loop_ts = 0.0
            r._last_supervisor_ts = 0.0
        h = r.get_health()
        self.assertFalse(h["vision_hung"])
        self.assertIsNone(h["vision_worker_age_s"])

    def test_start_publishes_a_running_thread_and_stamps_before_import(self):
        # The handle appears only once the thread runs, so no health sample
        # sees a started-but-not-running thread as dead; and the heartbeat
        # is stamped before `import depthai` (a wedged import is visible).
        r = _make_reader()
        r._run_pipeline_once = lambda dai, np_: (r._stop_event.set() or False)  # type: ignore[assignment]
        r._RECONNECT_BACKOFF_S = (0.01,)  # type: ignore[attr-defined]
        before = time.monotonic()
        r.start()
        with r._lock:
            thread = r._thread
        self.assertIsNotNone(thread)
        deadline = time.monotonic() + 5.0
        while thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(thread.is_alive())
        with r._lock:
            self.assertGreaterEqual(r._last_supervisor_ts, before)
        r.stop()
        with r._lock:
            self.assertIsNone(r._thread)
        self.assertFalse(r.get_health()["vision_hung"])

    def test_heartbeat_is_stamped_before_the_depthai_import(self):
        # A wedged `import depthai` must be visible as a hang: the stamp
        # happens before the import, so even an ImportError exit leaves it.
        r = _make_reader()
        saved = sys.modules.get("depthai")
        sys.modules["depthai"] = None  # type: ignore[assignment]  # makes `import depthai` raise ImportError
        try:
            before = time.monotonic()
            r._run_pipeline()   # returns at the ImportError, before the loop
        finally:
            if saved is None:
                sys.modules.pop("depthai", None)
            else:
                sys.modules["depthai"] = saved
        with r._lock:
            self.assertGreaterEqual(r._last_supervisor_ts, before)


if __name__ == "__main__":
    unittest.main()
