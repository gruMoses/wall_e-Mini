"""Colour-stream stall watchdog (2026-09-25 CAM_A).

A stalled NN queue must end the OAK session so the supervisor rebuilds it.
Depth keeps running, and there is no exception to catch. No OAK hardware:
depthai is a fake, same idea as test_oak_reconnect.py.
"""

import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from config import FollowMeConfig, OakDetectionConfig, ObstacleAvoidanceConfig
from pi_app.hardware.oak_depth import OakDepthReader, _color_stall_check
from pi_app.hardware import oak_depth as oak_depth_mod


# Binary-exact so N periods land on warmup + restart with no float drift.
# period = 1/16 s. Six sleeps reach 0.375 s.
_WARMUP_S = 0.25
_RESTART_S = 0.125
_RATE_HZ = 16.0
_PERIOD_S = 1.0 / _RATE_HZ


def _make_reader(det=None, update_rate_hz=50.0) -> OakDepthReader:
    obs = ObstacleAvoidanceConfig(
        roi_width_pct=0.8,
        roi_height_pct=0.5,
        robot_width_m=0.0,
        min_depth_mm=350,
        min_valid_pct=8.0,
        update_rate_hz=update_rate_hz,
        camera_hfov_deg=73.0,
    )
    return OakDepthReader(
        obstacle_config=obs,
        follow_me_config=FollowMeConfig(),
        detection_config=det,
    )


class TestColorStallCheck(unittest.TestCase):
    def test_config_defaults(self):
        cfg = OakDetectionConfig()
        self.assertEqual(cfg.color_stall_restart_s, 5.0)
        self.assertEqual(cfg.color_stall_warmup_s, 10.0)
        self.assertEqual(cfg.color_stall_max_restarts, 3)
        self.assertEqual(cfg.color_stall_window_s, 900.0)

    def test_disabled_at_zero_and_below(self):
        start = 1000.0
        for restart_s in (0.0, -1.0):
            cfg = OakDetectionConfig(color_stall_restart_s=restart_s)
            restart_now, _stalled = _color_stall_check(start + 100.0, start, None, cfg)
            self.assertFalse(restart_now)

    def test_no_restart_inside_warmup(self):
        cfg = OakDetectionConfig()
        start = 1000.0
        # Warm-up ends at start+10. The restart wait is on top of that.
        self.assertFalse(_color_stall_check(start + 9.9, start, None, cfg)[0])
        self.assertFalse(_color_stall_check(start + 10.0, start, None, cfg)[0])
        self.assertFalse(_color_stall_check(start + 14.9, start, None, cfg)[0])

    def test_no_packet_restarts_at_warmup_plus_restart(self):
        cfg = OakDetectionConfig()
        start = 1000.0
        deadline = start + 10.0 + 5.0
        self.assertFalse(_color_stall_check(deadline - 0.001, start, None, cfg)[0])
        restart_now, stalled = _color_stall_check(deadline, start, None, cfg)
        self.assertTrue(restart_now)
        self.assertAlmostEqual(stalled, 15.0)

    def test_packet_during_warmup_does_not_shorten_the_wait(self):
        cfg = OakDetectionConfig()
        start = 1000.0
        fresh = start + 1.0
        # 5 s after the packet, still inside the warm-up.
        self.assertFalse(_color_stall_check(start + 6.0, start, fresh, cfg)[0])
        self.assertFalse(_color_stall_check(start + 14.9, start, fresh, cfg)[0])
        self.assertTrue(_color_stall_check(start + 15.0, start, fresh, cfg)[0])

    def test_later_stall_uses_the_last_fresh_packet(self):
        cfg = OakDetectionConfig()
        # Session is long past warm-up. The packet is the reference.
        self.assertFalse(_color_stall_check(1004.9, 0.0, 1000.0, cfg)[0])
        restart_now, stalled = _color_stall_check(1005.0, 0.0, 1000.0, cfg)
        self.assertTrue(restart_now)
        self.assertAlmostEqual(stalled, 5.0)

    def test_health_defaults(self):
        health = _make_reader().get_health()
        self.assertEqual(health["color_stall_restarts"], 0)
        self.assertIs(health["color_stall_fault"], False)
        self.assertIn("connected", health)
        self.assertIn("det_fresh_age_s", health)
        self.assertIn("vision_stale", health)


class TestColorStallCap(unittest.TestCase):
    def _reader(self):
        return _make_reader(OakDetectionConfig())

    def test_disabled_reader_does_not_restart(self):
        reader = _make_reader(OakDetectionConfig(color_stall_restart_s=0.0))
        self.assertFalse(reader._color_stall_after_poll(10_000.0, 0.0, True))
        self.assertEqual(reader.get_health()["color_stall_restarts"], 0)

    def test_no_detection_queue_never_restarts(self):
        reader = self._reader()
        self.assertFalse(reader._color_stall_after_poll(10_000.0, 0.0, False))
        self.assertEqual(reader._color_stall_restart_count, 0)
        self.assertFalse(reader._color_stall_fault)

    def test_stop_does_not_count_as_a_restart(self):
        reader = self._reader()
        reader._stop_event.set()
        self.assertFalse(reader._color_stall_after_poll(10_000.0, 0.0, True))
        self.assertEqual(reader._color_stall_restart_count, 0)

    def test_warning_names_the_stall_and_the_depth_age(self):
        reader = self._reader()
        reader._last_depth_recv_ts = 1014.5
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            ended = reader._color_stall_after_poll(1015.0, 1000.0, True)
        self.assertTrue(ended)
        self.assertEqual(reader.get_health()["color_stall_restarts"], 1)
        self.assertIs(reader.get_health()["color_stall_fault"], False)
        self.assertEqual(reader.get_health()["last_pipeline_error"], "color_stall")
        text = "\n".join(caught.output)
        self.assertIn(
            "no fresh NN packet for 15.0 s (depth age 0.5 s); "
            "restarting the OAK session (1 in the last 15 min)",
            text,
        )

    def test_fourth_stall_inside_the_window_latches(self):
        reader = self._reader()
        for t in (100.0, 101.0, 102.0):
            self.assertTrue(reader._color_stall_after_poll(t, 0.0, True))
        self.assertEqual(reader._color_stall_restart_count, 3)
        self.assertFalse(reader._color_stall_fault)
        with self.assertLogs(oak_depth_mod.logger, level="ERROR") as caught:
            self.assertFalse(reader._color_stall_after_poll(110.0, 0.0, True))
            # Still stalled on the next poll: log once, do not end the session.
            self.assertFalse(reader._color_stall_after_poll(111.0, 0.0, True))
        self.assertTrue(reader._color_stall_fault)
        self.assertEqual(reader.get_health()["color_stall_restarts"], 3)
        self.assertIs(reader.get_health()["color_stall_fault"], True)
        errors = [line for line in caught.output if "auto-restart disabled" in line]
        self.assertEqual(len(errors), 1)
        self.assertIn("after 3 restarts in 15 min", errors[0])
        self.assertIn("restart the service to retry", errors[0])

    def test_restarts_older_than_the_window_are_pruned(self):
        reader = self._reader()
        for t in (100.0, 101.0, 102.0):
            self.assertTrue(reader._color_stall_after_poll(t, 0.0, True))
        # 20 minutes after the last restart. All three are outside 900 s.
        self.assertTrue(reader._color_stall_after_poll(102.0 + 1200.0, 0.0, True))
        self.assertFalse(reader._color_stall_fault)
        self.assertEqual(reader.get_health()["color_stall_restarts"], 4)
        self.assertEqual(reader._color_stall_restart_times, [1302.0])

    def test_window_boundary_keeps_a_restart_that_is_not_older(self):
        reader = self._reader()
        reader._color_stall_restart_times = [0.0, 1.0, 2.0]
        reader._color_stall_restart_count = 3
        # Age of t=0 is exactly the window: still inside, so this stalls latches.
        self.assertFalse(reader._color_stall_after_poll(900.0, 0.0, True))
        self.assertTrue(reader._color_stall_fault)

        reader = self._reader()
        reader._color_stall_restart_times = [0.0, 1.0, 2.0]
        reader._color_stall_restart_count = 3
        # t=0 is now older than the window. Two remain, so this one restarts.
        self.assertTrue(reader._color_stall_after_poll(901.0, 0.0, True))
        self.assertFalse(reader._color_stall_fault)
        self.assertEqual(reader.get_health()["color_stall_restarts"], 4)

    def test_fresh_packet_clears_the_latch_once(self):
        reader = self._reader()
        for t in (100.0, 101.0, 102.0):
            self.assertTrue(reader._color_stall_after_poll(t, 0.0, True))
        self.assertFalse(reader._color_stall_after_poll(110.0, 0.0, True))
        self.assertTrue(reader._color_stall_fault)

        # Exactly vision_stale_s is not below the threshold.
        reader._det_fresh_ts = 200.0
        self.assertFalse(reader._color_stall_after_poll(201.5, 0.0, True))
        self.assertTrue(reader._color_stall_fault)

        reader._det_fresh_ts = 300.0
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertFalse(reader._color_stall_after_poll(300.1, 0.0, True))
            self.assertFalse(reader._color_stall_after_poll(300.2, 0.0, True))
        self.assertFalse(reader._color_stall_fault)
        self.assertIs(reader.get_health()["color_stall_fault"], False)
        recoveries = [line for line in caught.output if "recovered" in line]
        self.assertEqual(len(recoveries), 1)
        # The cap is not reset by one good packet. The next stall inside
        # the window latches again and does not restart.
        self.assertFalse(reader._color_stall_after_poll(305.0, 0.0, True))
        self.assertTrue(reader._color_stall_fault)
        self.assertEqual(reader.get_health()["color_stall_restarts"], 3)


# ---------------------------------------------------------------------------
# Fake depthai. The real _run_pipeline_once builds a pipeline and polls.
# Only the YOLO path is constructed (model_path points at a file that
# exists; the blob bytes are never read).
# ---------------------------------------------------------------------------

_PORT_NAMES = (
    "out", "depth", "input", "left", "right", "inputDepth",
    "passthrough", "passthroughDepth", "bitstream",
)


class _Flex:
    def __init__(self, *args, **kwargs):
        object.__setattr__(self, "_a", {})

    def __getattr__(self, name):
        attrs = object.__getattribute__(self, "_a")
        if name not in attrs:
            attrs[name] = _Flex()
        return attrs[name]

    def __setattr__(self, name, value):
        object.__getattribute__(self, "_a")[name] = value

    def __call__(self, *args, **kwargs):
        return _Flex()


class _NodeNS:
    class Camera:
        _kind = "camera"

    class StereoDepth:
        _kind = "stereo"

        class PresetMode:
            DENSITY = "DENSITY"

    class SpatialLocationCalculator:
        _kind = "spatial"

    class NeuralNetwork:
        _kind = "nn"

    class IMU:
        _kind = "imu"


class _Algo:
    MIN = "MIN"
    MEDIAN = "MEDIAN"
    AVERAGE = "AVERAGE"


class _ImuSensor:
    ACCELEROMETER_RAW = "ACCELEROMETER_RAW"
    GYROSCOPE_RAW = "GYROSCOPE_RAW"


class _Device:
    @staticmethod
    def getAllAvailableDevices():
        return [object()]


class _NnMsg:
    detections = []

    def __init__(self, ts, seq):
        self._ts = ts
        self._seq = seq

    def getTimestamp(self):
        return self._ts

    def getSequenceNum(self):
        return self._seq


class _FrameMsg:
    def __init__(self, frame):
        self._frame = frame

    def getFrame(self):
        return self._frame


class _EmptyQueue:
    def tryGet(self):
        return None


class _PollQueue:
    """One item per poll, then None so the reader's drain loop stops."""

    def __init__(self, factory):
        self._factory = factory
        self._holding = False

    def tryGet(self):
        if self._holding:
            self._holding = False
            return None
        item = self._factory()
        if item is None:
            return None
        self._holding = True
        return item


class _FakePort:
    def __init__(self, state, kind, port):
        self._state = state
        self._kind = kind
        self._port = port

    def link(self, *args, **kwargs):
        return None

    def setBlocking(self, *args, **kwargs):
        return None

    def setMaxSize(self, *args, **kwargs):
        return None

    def getMaxSize(self):
        return 1

    def createOutputQueue(self, *args, **kwargs):
        return self._state.queue_for(self._kind, self._port)


class _FakeNode:
    def __init__(self, state, kind):
        self._state = state
        self._kind = kind
        self._cache = {}

    def build(self, *args, **kwargs):
        return self

    def requestOutput(self, *args, **kwargs):
        return _FakePort(self._state, "camera", "requestOutput")

    def __getattr__(self, name):
        cache = object.__getattribute__(self, "_cache")
        if name in cache:
            return cache[name]
        if name == "initialConfig":
            obj = _Flex()
        elif name in _PORT_NAMES:
            obj = _FakePort(self._state, self._kind, name)
        else:
            def _noop(*args, **kwargs):
                return None
            return _noop
        cache[name] = obj
        return obj


class _DaiState:
    def __init__(self):
        self.reader = None
        self.clock = None
        self.starts = 0
        self.start_ts = []
        self.stop_on_start = None
        self.nn_queue_is_none = False
        self.nn_served = 0
        self.depth_polls = 0
        self.depth_bail_after = None
        self.force_pipeline_down = False
        self.nn_queue = _PollQueue(self._nn_item)
        self.depth_queue = _PollQueue(self._depth_item)
        self.empty_queue = _EmptyQueue()
        self._frame = np.zeros((40, 40), dtype=np.uint16)
        self._started = None

    def queue_for(self, kind, port):
        if kind == "nn" and port == "out":
            if self.nn_queue_is_none:
                return None
            return self.nn_queue
        if kind == "stereo" and port == "depth":
            return self.depth_queue
        return self.empty_queue

    def _nn_item(self):
        # One packet at the start of the session, then silence.
        if self.nn_served >= 1:
            return None
        self.nn_served += 1
        return _NnMsg(self.clock.t, self.nn_served)

    def _depth_item(self):
        self.depth_polls += 1
        if self._started is not None and not self._started.is_set():
            self._started.set()
        if (
            self.depth_bail_after is not None
            and self.depth_polls > self.depth_bail_after
            and self.reader is not None
        ):
            self.force_pipeline_down = True
            self.reader._stop_event.set()
        return _FrameMsg(self._frame.copy())


def _install_dai(state: _DaiState):
    mod = types.ModuleType("depthai")
    mod.node = _NodeNS
    mod.CameraBoardSocket = _Flex()
    mod.ImgFrame = _Flex()
    mod.Rect = _Flex
    mod.Point2f = _Flex
    mod.SpatialLocationCalculatorConfigData = _Flex
    mod.SpatialLocationCalculatorAlgorithm = _Algo
    mod.IMUSensor = _ImuSensor
    mod.Device = _Device

    class Pipeline:
        def __init__(self):
            self._running = False

        def create(self, node_type):
            kind = getattr(node_type, "_kind", "unknown")
            return _FakeNode(state, kind)

        def start(self):
            state.starts += 1
            state.start_ts.append(state.clock.t)
            if (
                state.stop_on_start is not None
                and state.starts >= state.stop_on_start
            ):
                state.reader._stop_event.set()
            self._running = True

        def isRunning(self):
            if state.force_pipeline_down:
                return False
            return self._running

        def stop(self):
            self._running = False

        def getDefaultDevice(self):
            raise RuntimeError("no device in test")

    mod.Pipeline = Pipeline
    sys.modules["depthai"] = mod
    return mod


class _Clock:
    def __init__(self, t=10000.0):
        self.t = float(t)
        self.sleeps = 0
        self.stop_after = None
        self.reader = None

    def monotonic(self):
        return self.t

    def sleep(self, seconds):
        self.t += float(seconds)
        self.sleeps += 1
        if (
            self.reader is not None
            and self.stop_after is not None
            and self.t >= self.stop_after
        ):
            self.reader._stop_event.set()
        if self.reader is not None and self.sleeps > 10000:
            self.reader._stop_event.set()


def _run_supervisor(reader, max_calls=4):
    real = reader._run_pipeline_once
    calls = {"n": 0}

    def wrapped(dai, npmod):
        calls["n"] += 1
        if calls["n"] > max_calls:
            reader._stop_event.set()
            return False
        return real(dai, npmod)

    reader._run_pipeline_once = wrapped
    reader._RECONNECT_BACKOFF_S = (0.0, 0.0, 0.0)
    reader._run_pipeline()
    return calls["n"]


class TestColorStallSession(unittest.TestCase):
    def setUp(self):
        self._prev_dai = sys.modules.get("depthai")

    def tearDown(self):
        if self._prev_dai is None:
            sys.modules.pop("depthai", None)
        else:
            sys.modules["depthai"] = self._prev_dai

    def _bind(self, state, reader, clock):
        state.reader = reader
        state.clock = clock
        clock.reader = reader
        _install_dai(state)

    def test_stalled_nn_queue_rebuilds_after_warmup_plus_restart(self):
        """NN packets stop. Depth frames keep arriving. The session ends
        at warmup + restart, and the supervisor builds another session."""
        blob = Path(__file__).resolve().parents[2] / "config.py"
        self.assertTrue(blob.is_file(), "YOLO path needs an existing model_path")
        state = _DaiState()
        clock = _Clock()
        reader = _make_reader(
            OakDetectionConfig(
                model_path="config.py",
                color_stall_warmup_s=_WARMUP_S,
                color_stall_restart_s=_RESTART_S,
                color_stall_max_restarts=3,
                color_stall_window_s=900.0,
            ),
            update_rate_hz=_RATE_HZ,
        )
        state.stop_on_start = 2
        self._bind(state, reader, clock)
        with patch("pi_app.hardware.oak_depth.time.monotonic", clock.monotonic), \
                patch("pi_app.hardware.oak_depth.time.sleep", clock.sleep), \
                self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            calls = _run_supervisor(reader)
        self.assertEqual(state.starts, 2, "\n".join(caught.output))
        self.assertLessEqual(calls, 4)
        self.assertEqual(state.nn_served, 1)
        self.assertGreater(reader._depth_recv_count, state.nn_served)
        self.assertEqual(len(state.start_ts), 2)
        self.assertAlmostEqual(
            reader._color_stall_restart_times[0] - state.start_ts[0],
            _WARMUP_S + _RESTART_S,
            places=9,
        )
        health = reader.get_health()
        self.assertEqual(health["color_stall_restarts"], 1)
        self.assertIs(health["color_stall_fault"], False)
        self.assertEqual(health["reconnect_count"], 1)
        text = "\n".join(caught.output)
        self.assertIn("restarting the OAK session (1 in the last 15 min)", text)
        self.assertIn("depth age 0.0 s", text)
        self.assertEqual(text.count("restarting the OAK session"), 1)

    def test_no_detection_queue_does_not_end_the_session(self):
        state = _DaiState()
        state.nn_queue_is_none = True
        clock = _Clock()
        clock.stop_after = 10000.0 + 1.0
        reader = _make_reader(
            OakDetectionConfig(
                model_path="config.py",
                color_stall_warmup_s=_WARMUP_S,
                color_stall_restart_s=_RESTART_S,
            ),
            update_rate_hz=_RATE_HZ,
        )
        state.stop_on_start = 2
        self._bind(state, reader, clock)
        with patch("pi_app.hardware.oak_depth.time.monotonic", clock.monotonic), \
                patch("pi_app.hardware.oak_depth.time.sleep", clock.sleep), \
                self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            _run_supervisor(reader)
        self.assertGreaterEqual(clock.t, state.start_ts[0] + _WARMUP_S + _RESTART_S)
        self.assertEqual(state.starts, 1, "\n".join(caught.output))
        self.assertGreater(reader._depth_recv_count, 0)
        health = reader.get_health()
        self.assertEqual(health["color_stall_restarts"], 0)
        self.assertIs(health["color_stall_fault"], False)
        self.assertNotIn("restarting the OAK session", "\n".join(caught.output))

    def test_stop_during_a_stall_does_not_restart(self):
        state = _DaiState()
        clock = _Clock(t=time.monotonic())
        reader = _make_reader(
            OakDetectionConfig(
                model_path="config.py",
                color_stall_warmup_s=30.0,
                color_stall_restart_s=30.0,
            ),
            update_rate_hz=100.0,
        )
        state.stop_on_start = 2
        state.depth_bail_after = 250
        started = threading.Event()
        state._started = started
        self._bind(state, reader, clock)
        reader._RECONNECT_BACKOFF_S = (0.0, 0.0, 0.0)
        reader.start()
        try:
            self.assertTrue(started.wait(timeout=2.0))
            worker = reader._thread
            t0 = time.monotonic()
            reader.stop()
            self.assertLess(time.monotonic() - t0, 1.0)
            self.assertFalse(worker.is_alive())
        finally:
            reader._stop_event.set()
            if reader._thread is not None:
                reader._thread.join(timeout=1.0)
        self.assertEqual(state.starts, 1)
        self.assertLess(state.depth_polls, 50)
        health = reader.get_health()
        self.assertEqual(health["color_stall_restarts"], 0)
        self.assertIs(health["color_stall_fault"], False)


if __name__ == "__main__":
    unittest.main()
