"""IMU stall watchdog (2026-11-06 19:34:37).

The camera's IMU stream stopped reaching the Pi in the middle of a healthy
session, with no error. Depth and detections kept running, so nothing ended
the session, and the heading stayed frozen for 38.8 hours. The watchdog ends
the session when no IMU message has arrived for imu_stall_restart_s while
depth is fresh, and the supervisor rebuilds it. No OAK hardware: depthai is
the fake of test_oak_color_stall.py, with live depth and NN queues and an
IMU queue the test can silence.
"""

import datetime
import math
import sys
import unittest
from unittest.mock import patch

from config import OakDetectionConfig
from pi_app.hardware import oak_depth as oak_depth_mod
from pi_app.hardware.oak_depth import _imu_stall_check
from pi_app.hardware.oak_imu import OakImuReader
from pi_app.tests import test_oak_color_stall as cs

# Binary-exact, as in the colour-stall tests. period = 1/16 s.
_WARMUP_S = 0.25
_RESTART_S = 0.5
_RATE_HZ = 16.0
_TURN_DPS = 30.0


def _cfg(**kw) -> OakDetectionConfig:
    base = dict(
        model_path="config.py",
        # The colour watchdog stays quiet: NN packets are live, and its
        # warm-up is longer than these runs need.
        color_stall_warmup_s=60.0,
        color_stall_restart_s=5.0,
        imu_stall_warmup_s=_WARMUP_S,
        imu_stall_restart_s=_RESTART_S,
    )
    base.update(kw)
    return OakDetectionConfig(**base)


class _Vec:
    def __init__(self, x, y, z, dev_ts=None):
        self.x = x
        self.y = y
        self.z = z
        self._dev_ts = dev_ts

    def getTimestampDevice(self):
        return datetime.timedelta(seconds=self._dev_ts)


class _ImuPkt:
    def __init__(self, dev_ts, gy_rads):
        self.acceleroMeter = _Vec(0.0, -9.7, 0.4, dev_ts)
        self.gyroscope = _Vec(0.0, gy_rads, 0.0, dev_ts)


class _ImuMsg:
    def __init__(self, packets):
        self.packets = packets


class _ImuState(cs._DaiState):
    """Live depth and NN queues; the IMU queue obeys ``imu_live(session, t)``."""

    def __init__(self, imu_live):
        super().__init__()
        self.imu_live = imu_live
        self.imu_served = {}
        self.dev_ts = 1.0
        self.nn_queue = cs._PollQueue(self._fresh_nn_item)
        self.imu_queue = cs._PollQueue(self._imu_item)
        self.stop_after_session2_msgs = None

    def queue_for(self, kind, port):
        if kind == "imu" and port == "out":
            return self.imu_queue
        return super().queue_for(kind, port)

    def _fresh_nn_item(self):
        self.nn_served += 1
        return cs._NnMsg(self.clock.t, self.nn_served)

    def _imu_item(self):
        session = self.starts
        if not self.imu_live(session, self.clock.t):
            return None
        self.imu_served[session] = self.imu_served.get(session, 0) + 1
        if (
            session == 2
            and self.stop_after_session2_msgs is not None
            and self.imu_served[session] >= self.stop_after_session2_msgs
        ):
            self.reader._stop_event.set()
        self.dev_ts += 0.01
        return _ImuMsg([_ImuPkt(self.dev_ts, math.radians(_TURN_DPS))])


class TestImuStallCheck(unittest.TestCase):
    def test_config_defaults(self):
        cfg = OakDetectionConfig()
        self.assertEqual(cfg.imu_stall_restart_s, 10.0)
        self.assertEqual(cfg.imu_stall_warmup_s, 10.0)
        self.assertEqual(cfg.imu_stall_depth_fresh_s, 1.5)

    def test_disabled_at_zero(self):
        cfg = OakDetectionConfig(imu_stall_restart_s=0.0)
        self.assertFalse(_imu_stall_check(1e6, 0.0, 1.0, cfg)[0])

    def test_rule(self):
        cfg = OakDetectionConfig()
        start = 1000.0
        # No message this session: warm-up + restart from the session start.
        self.assertFalse(_imu_stall_check(start + 19.9, start, None, cfg)[0])
        self.assertTrue(_imu_stall_check(start + 20.0, start, None, cfg)[0])
        # Later stall: restart_s after the last message.
        self.assertFalse(_imu_stall_check(1109.9, start, 1100.0, cfg)[0])
        restart, stalled = _imu_stall_check(1110.0, start, 1100.0, cfg)
        self.assertTrue(restart)
        self.assertAlmostEqual(stalled, 10.0)

    def test_health_defaults(self):
        health = cs._make_reader(OakDetectionConfig()).get_health()
        self.assertEqual(health["imu_stall_restarts"], 0)
        self.assertIs(health["imu_stall_fault"], False)


class TestImuStallGate(unittest.TestCase):
    """_imu_stall_after_poll on a reader without a session (fake times)."""

    def _reader(self, **kw):
        reader = cs._make_reader(OakDetectionConfig(**kw))
        reader.set_imu_watchdog_enabled(True)
        reader._imu_msg_ts = 100.0
        reader._imu_session_msg_ts = 100.0
        return reader

    @staticmethod
    def _poll(reader, now, start=0.0):
        reader._last_depth_recv_ts = now - 0.05
        return reader._imu_stall_after_poll(now, start)

    def test_off_until_a_heading_consumer_enables_it(self):
        reader = cs._make_reader(OakDetectionConfig())
        reader._imu_msg_ts = 100.0
        reader._imu_session_msg_ts = 100.0
        self.assertFalse(self._poll(reader, 200.0))
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)
        OakImuReader(reader)
        self.assertTrue(reader._imu_watchdog_enabled)
        self.assertTrue(self._poll(reader, 200.0))

    def test_stall_restarts_after_restart_s(self):
        reader = self._reader()
        self.assertFalse(self._poll(reader, 109.9))
        reader._last_depth_recv_ts = 109.5
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertTrue(reader._imu_stall_after_poll(110.0, 0.0))
        health = reader.get_health()
        self.assertEqual(health["imu_stall_restarts"], 1)
        self.assertEqual(health["last_pipeline_error"], "imu_stall")
        self.assertIn(
            "OAK IMU stream stalled: no IMU message for 10.0 s (depth age 0.5 s); "
            "restarting the OAK session (1 in the last 15 min)",
            "\n".join(caught.output),
        )

    def test_stale_depth_does_not_restart(self):
        reader = self._reader()
        reader._last_depth_recv_ts = 108.4  # 1.6 s old at 110
        self.assertFalse(reader._imu_stall_after_poll(110.0, 0.0))
        reader._last_depth_recv_ts = 0.0  # never any depth
        self.assertFalse(reader._imu_stall_after_poll(110.0, 0.0))
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)

    def test_stop_does_not_count(self):
        reader = self._reader()
        reader._stop_event.set()
        self.assertFalse(self._poll(reader, 500.0))
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)

    def test_imu_that_never_delivered_is_not_restarted(self):
        reader = self._reader()
        reader._imu_msg_ts = None
        reader._imu_session_msg_ts = None
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            for t in (25.0, 60.0, 600.0):
                self.assertFalse(self._poll(reader, t, start=0.0))
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)
        self.assertIs(reader.get_health()["imu_stall_fault"], False)
        lines = [l for l in caught.output if "delivered no message since start" in l]
        self.assertEqual(len(lines), 1)

    def test_new_session_without_imu_after_it_worked_restarts(self):
        reader = self._reader()
        reader._imu_session_msg_ts = None  # the IMU worked in an earlier session
        start = 1000.0
        self.assertFalse(self._poll(reader, start + 19.9, start=start))
        self.assertTrue(self._poll(reader, start + 20.0, start=start))

    def test_budget_latches_and_fresh_message_clears(self):
        reader = self._reader()
        for t in (110.0, 111.0, 112.0):
            self.assertTrue(self._poll(reader, t))
        with self.assertLogs(oak_depth_mod.logger, level="ERROR") as caught:
            self.assertFalse(self._poll(reader, 113.0))
            self.assertFalse(self._poll(reader, 114.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 3)
        self.assertEqual(
            len([l for l in caught.output if "IMU auto-restart disabled" in l]), 1
        )
        # A message older than the latch does not clear it.
        reader._imu_msg_ts = 112.5
        self.assertFalse(self._poll(reader, 115.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])
        # A new message does, once; the restart history stays.
        reader._imu_msg_ts = 116.0
        reader._imu_session_msg_ts = 116.0
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertFalse(self._poll(reader, 116.1))
        self.assertIs(reader.get_health()["imu_stall_fault"], False)
        self.assertIn("recovered after a stall fault", "\n".join(caught.output))
        # Still inside the window: the next stall latches again.
        self.assertFalse(self._poll(reader, 126.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 3)

    def test_budget_is_shared_with_the_colour_watchdog(self):
        # Colour restarts fill the shared budget: the IMU watchdog latches.
        reader = self._reader()
        reader._color_stall_restart_times = [100.0, 101.0, 102.0]
        self.assertFalse(self._poll(reader, 110.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)

        # IMU restarts fill it: the colour watchdog latches.
        reader = self._reader()
        for t in (110.0, 111.0, 112.0):
            self.assertTrue(self._poll(reader, t))
        reader._last_depth_recv_ts = 120.0
        self.assertFalse(reader._color_stall_after_poll(120.0, 0.0, True))
        self.assertTrue(reader.get_health()["color_stall_fault"])
        self.assertEqual(reader.get_health()["color_stall_restarts"], 0)

        # Mixed: one colour + two IMU restarts are the three in the window.
        reader = self._reader()
        reader._color_stall_restart_times = [105.0]
        self.assertTrue(self._poll(reader, 110.0))
        self.assertTrue(self._poll(reader, 111.0))
        self.assertFalse(self._poll(reader, 112.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])

    def test_window_pruning_frees_the_budget(self):
        reader = self._reader()
        for t in (110.0, 111.0, 112.0):
            self.assertTrue(self._poll(reader, t))
        # 20 minutes later all three are outside the 900 s window.
        self.assertTrue(self._poll(reader, 112.0 + 1200.0))
        self.assertEqual(reader._imu_stall_restart_times, [1312.0])
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 4)


class TestImuStallSession(unittest.TestCase):
    def setUp(self):
        self._prev_dai = sys.modules.get("depthai")

    def tearDown(self):
        if self._prev_dai is None:
            sys.modules.pop("depthai", None)
        else:
            sys.modules["depthai"] = self._prev_dai

    def _run(self, state, clock, cfg, enable=True):
        reader = cs._make_reader(cfg, update_rate_hz=_RATE_HZ)
        if enable:
            reader.set_imu_watchdog_enabled(True)
        state.reader = reader
        state.clock = clock
        clock.reader = reader
        cs._install_dai(state)
        with patch("pi_app.hardware.oak_depth.time.monotonic", clock.monotonic), \
                patch("pi_app.hardware.oak_depth.time.sleep", clock.sleep), \
                self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            cs._run_supervisor(reader)
        return reader, "\n".join(caught.output)

    def test_silent_imu_queue_rebuilds_the_session_and_heading_resumes(self):
        t0 = 10000.0
        dead_from = t0 + 1.0
        # Session 1: IMU live for 1 s, then silent with depth and NN live.
        # Session 2: IMU live again.
        state = _ImuState(lambda session, t: session >= 2 or t < dead_from)
        state.stop_after_session2_msgs = 40
        clock = cs._Clock(t=t0)
        reader, text = self._run(state, clock, _cfg())
        self.assertEqual(state.starts, 2, text)
        health = reader.get_health()
        self.assertEqual(health["imu_stall_restarts"], 1)
        self.assertIs(health["imu_stall_fault"], False)
        self.assertEqual(health["color_stall_restarts"], 0)
        self.assertEqual(health["reconnect_count"], 1)
        # The session ended restart_s after the last IMU message (within one
        # loop period), not later.
        restart_ts = reader._imu_stall_restart_times[0]
        last_msg_s1 = restart_ts - _RESTART_S
        self.assertLess(last_msg_s1, dead_from)
        self.assertGreaterEqual(last_msg_s1, dead_from - 1.0 / _RATE_HZ - 1e-9)
        self.assertIn("OAK IMU stream stalled", text)
        self.assertEqual(text.count("restarting the OAK session"), 1)
        # Depth and detections never stopped.
        self.assertGreater(reader._depth_recv_count, 0)
        # Heading resumes in session 2: the producer integrates the new
        # session's packets and the cumulative yaw advances.
        self.assertGreaterEqual(state.imu_served.get(2, 0), 40)
        self.assertIsNotNone(reader._imu_session_msg_ts)
        self.assertGreater(reader._imu_session_msg_ts, restart_ts)
        metrics = reader.get_imu_metrics()
        self.assertGreater(metrics["producer_cum_yaw_y_deg"], 0.0)
        integrated_total = metrics["packets_integrated"]
        self.assertGreater(integrated_total, state.imu_served[1])

    def test_short_imu_drops_do_not_rebuild(self):
        t0 = 10000.0
        # Three 0.375 s drops (shorter than the 0.5 s restart), depth live.
        gaps = [(t0 + 1.0, t0 + 1.375), (t0 + 2.0, t0 + 2.375), (t0 + 3.0, t0 + 3.375)]

        def live(session, t):
            return not any(a <= t < b for a, b in gaps)

        state = _ImuState(live)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 5.0
        reader, text = self._run(state, clock, _cfg())
        self.assertEqual(state.starts, 1, text)
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)
        self.assertNotIn("restarting the OAK session", text)

    def test_disabled_watchdog_keeps_the_session(self):
        t0 = 10000.0
        state = _ImuState(lambda session, t: t < t0 + 1.0)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 4.0
        reader, text = self._run(state, clock, _cfg(), enable=False)
        self.assertEqual(state.starts, 1, text)
        self.assertEqual(reader.get_health()["imu_stall_restarts"], 0)

    def test_dead_imu_after_budget_latches_and_session_stays_up(self):
        t0 = 10000.0
        # The IMU never comes back after the first session's first second.
        state = _ImuState(lambda session, t: session == 1 and t < t0 + 1.0)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 12.0
        reader, text = self._run(state, clock, _cfg())
        health = reader.get_health()
        # 1 + 3 sessions: three restarts, then the latch; the fourth
        # session keeps running with depth live until stop().
        self.assertEqual(health["imu_stall_restarts"], 3, text)
        self.assertIs(health["imu_stall_fault"], True)
        self.assertEqual(state.starts, 4, text)
        self.assertEqual(text.count("IMU auto-restart disabled"), 1)


if __name__ == "__main__":
    unittest.main()
