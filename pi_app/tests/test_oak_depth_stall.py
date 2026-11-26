"""Depth stall watchdog (2026-11-22 08:43 and 2026-11-25 20:51).

depthai's "no request for this response: XLINK_READ_REL_RESP" error killed
the depth stream alone in a live session, with detections, RGB and the IMU
still running. Nothing ended the session, so depth stayed dead for 25.6 h
and 11.8 h until a service restart (MANUAL on the stale floor, FOLLOW_ME and
WAYPOINT_NAV refused). The watchdog ends the session when no depth frame has
arrived in this session for depth_stall_restart_s while detections are
fresh, and the supervisor rebuilds it. No OAK hardware: depthai is the fake
of test_oak_color_stall.py, with a live NN queue and a depth queue the test
can silence.
"""

import sys
import unittest
from unittest.mock import patch

import numpy as np

from config import OakDetectionConfig
from pi_app.hardware import oak_depth as oak_depth_mod
from pi_app.hardware.oak_depth import _depth_stall_check
from pi_app.tests import test_oak_color_stall as cs

# Binary-exact, as in the colour-stall tests. period = 1/16 s.
_WARMUP_S = 0.25
_RESTART_S = 0.5
_RATE_HZ = 16.0


def _cfg(**kw) -> OakDetectionConfig:
    base = dict(
        model_path="config.py",
        # The colour watchdog stays quiet: its warm-up is longer than
        # these runs need.
        color_stall_warmup_s=60.0,
        color_stall_restart_s=5.0,
        depth_stall_warmup_s=_WARMUP_S,
        depth_stall_restart_s=_RESTART_S,
    )
    base.update(kw)
    return OakDetectionConfig(**base)


class _DepthState(cs._DaiState):
    """Depth queue obeys ``depth_live(session, t)``; NN obeys ``nn_live``."""

    def __init__(self, depth_live, nn_live=None):
        super().__init__()
        self.depth_live = depth_live
        self.nn_live = nn_live or (lambda session, t: True)
        self.depth_served = {}
        self.stop_after_session2_frames = None
        self.nn_queue = cs._PollQueue(self._live_nn_item)
        self.depth_queue = cs._PollQueue(self._live_depth_item)

    def _live_nn_item(self):
        if not self.nn_live(self.starts, self.clock.t):
            return None
        self.nn_served += 1
        return cs._NnMsg(self.clock.t, self.nn_served)

    def _live_depth_item(self):
        session = self.starts
        if not self.depth_live(session, self.clock.t):
            return None
        self.depth_served[session] = self.depth_served.get(session, 0) + 1
        if (
            session == 2
            and self.stop_after_session2_frames is not None
            and self.depth_served[session] >= self.stop_after_session2_frames
        ):
            self.reader._stop_event.set()
        return cs._FrameMsg(self._frame.copy())


class TestDepthStallCheck(unittest.TestCase):
    def test_config_defaults(self):
        cfg = OakDetectionConfig()
        self.assertEqual(cfg.depth_stall_restart_s, 5.0)
        self.assertEqual(cfg.depth_stall_warmup_s, 10.0)
        self.assertEqual(cfg.depth_stall_det_fresh_s, 1.5)
        self.assertEqual(cfg.depth_stall_worked_s, 60.0)

    def test_disabled_at_zero(self):
        cfg = OakDetectionConfig(depth_stall_restart_s=0.0)
        self.assertFalse(_depth_stall_check(1e6, 0.0, 1.0, cfg)[0])

    def test_rule(self):
        cfg = OakDetectionConfig()
        start = 1000.0
        # No frame this session: warm-up + restart from the session start.
        self.assertFalse(_depth_stall_check(start + 14.9, start, None, cfg)[0])
        self.assertTrue(_depth_stall_check(start + 15.0, start, None, cfg)[0])
        # Later stall: restart_s after the last frame.
        self.assertFalse(_depth_stall_check(1104.9, start, 1100.0, cfg)[0])
        restart, stalled = _depth_stall_check(1105.0, start, 1100.0, cfg)
        self.assertTrue(restart)
        self.assertAlmostEqual(stalled, 5.0)

    def test_health_defaults(self):
        health = cs._make_reader(OakDetectionConfig()).get_health()
        self.assertEqual(health["depth_stall_restarts"], 0)
        self.assertIs(health["depth_stall_fault"], False)
        self.assertIs(health["depth_stall_given_up"], False)


class TestDepthStallGate(unittest.TestCase):
    """_depth_stall_after_poll on a reader without a session (fake times)."""

    def _reader(self, **kw):
        reader = cs._make_reader(OakDetectionConfig(**kw))
        reader._last_depth_recv_ts = 100.0
        reader._depth_session_recv_ts = 100.0
        return reader

    @staticmethod
    def _poll(reader, now, start=0.0, det_q=True):
        reader._det_fresh_ts = now - 0.05
        return reader._depth_stall_after_poll(now, start, det_q)

    def test_stall_restarts_after_restart_s(self):
        reader = self._reader()
        self.assertFalse(self._poll(reader, 104.9))
        reader._det_fresh_ts = 104.6
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertTrue(reader._depth_stall_after_poll(105.0, 0.0, True))
        health = reader.get_health()
        self.assertEqual(health["depth_stall_restarts"], 1)
        self.assertEqual(health["last_pipeline_error"], "depth_stall")
        self.assertIn(
            "OAK depth stream stalled: no depth frame for 5.0 s (detections age 0.4 s); "
            "restarting the OAK session (1 in the last 15 min)",
            "\n".join(caught.output),
        )

    def test_stale_detections_do_not_restart(self):
        reader = self._reader()
        reader._det_fresh_ts = 108.4  # 1.6 s old at 110
        self.assertFalse(reader._depth_stall_after_poll(110.0, 0.0, True))
        reader._det_fresh_ts = None  # never any NN packet
        self.assertFalse(reader._depth_stall_after_poll(110.0, 0.0, True))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    def test_no_detection_queue_and_stop_do_not_count(self):
        reader = self._reader()
        self.assertFalse(self._poll(reader, 500.0, det_q=False))
        reader._stop_event.set()
        self.assertFalse(self._poll(reader, 500.0))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    def test_disabled_never_restarts(self):
        reader = self._reader(depth_stall_restart_s=0.0)
        self.assertFalse(self._poll(reader, 1e5))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    def test_new_session_without_depth_restarts_after_warmup(self):
        reader = self._reader()
        reader._depth_session_recv_ts = None  # depth worked in an earlier session
        start = 1000.0
        self.assertFalse(self._poll(reader, start + 0.74 + 14.0, start=start))
        self.assertTrue(self._poll(reader, start + 15.0, start=start))

    def test_old_detections_from_before_the_rebuild_do_not_count(self):
        # New session, no depth yet; the last NN packet is the old
        # session's, 9 s before this session started.
        reader = self._reader()
        reader._depth_session_recv_ts = None
        start = 1000.0
        reader._det_fresh_ts = start - 9.0
        self.assertFalse(reader._depth_stall_after_poll(start + 15.0, start, True))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    @staticmethod
    def _session_delivers(reader, first, last):
        # The session after a rebuild delivered depth from first to last
        # without a gap.
        reader._depth_run_start_ts = first
        reader._depth_session_recv_ts = last
        reader._last_depth_recv_ts = last

    def _restarts_that_worked(self, reader, times):
        # Each rebuild brought depth back for 61 s, then the stream died
        # again: a flapping stream, not a dead one. Times >= 67 s apart.
        for t in times:
            self.assertTrue(self._poll(reader, t))
            self._session_delivers(reader, t + 0.5, t + 61.5)

    def test_full_budget_waits_and_fresh_frame_clears(self):
        reader = self._reader()
        self._restarts_that_worked(reader, (105.0, 200.0, 300.0))
        with self.assertLogs(oak_depth_mod.logger, level="ERROR") as caught:
            self.assertFalse(self._poll(reader, 380.0))
            self.assertFalse(self._poll(reader, 381.0))
        health = reader.get_health()
        self.assertTrue(health["depth_stall_fault"])
        self.assertIs(health["depth_stall_given_up"], False)
        self.assertEqual(health["depth_stall_restarts"], 3)
        self.assertEqual(
            len([l for l in caught.output if "when the window has room" in l]), 1
        )
        # A frame older than the fault does not clear it.
        self.assertEqual(reader._last_depth_recv_ts, 361.5)
        self.assertFalse(self._poll(reader, 382.0))
        self.assertTrue(reader.get_health()["depth_stall_fault"])
        # A new frame does, once; the restart history stays.
        reader._last_depth_recv_ts = 383.0
        reader._depth_session_recv_ts = 383.0
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertFalse(self._poll(reader, 383.1))
        self.assertIs(reader.get_health()["depth_stall_fault"], False)
        self.assertIn("recovered after a stall fault", "\n".join(caught.output))
        # Still inside the window: the next stall waits again.
        self.assertFalse(self._poll(reader, 390.0))
        self.assertTrue(reader.get_health()["depth_stall_fault"])
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 3)

    def test_full_budget_is_not_a_permanent_latch(self):
        # Grok review 2026-11-26: a dead depth stream never delivers the
        # frame that would clear a latch. A full budget only delays the
        # next restart until the window has room.
        reader = self._reader()
        self._restarts_that_worked(reader, (105.0, 200.0, 300.0))
        self.assertFalse(self._poll(reader, 380.0))
        self.assertTrue(reader.get_health()["depth_stall_fault"])
        self.assertFalse(self._poll(reader, 105.0 + 899.0))
        with self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            self.assertTrue(self._poll(reader, 105.0 + 901.0))
        self.assertIs(reader.get_health()["depth_stall_fault"], False)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 4)
        self.assertIn("restarting the OAK session (3 in the last 15 min)",
                      "\n".join(caught.output))

    def test_gives_up_after_rebuilds_that_bring_no_depth(self):
        # Second Grok review 2026-11-26: retrying a stream that rebuilds do
        # not fix is a periodic ~8 s detection blackout. Three rebuilds in a
        # row with no depth after them: give up.
        reader = self._reader()
        for t in (105.0, 106.0, 107.0):
            self.assertTrue(self._poll(reader, t))
            reader._depth_run_start_ts = None
            reader._depth_session_recv_ts = None
        with self.assertLogs(oak_depth_mod.logger, level="ERROR") as caught:
            self.assertFalse(self._poll(reader, 120.0))
            self.assertFalse(self._poll(reader, 121.0))
        self.assertEqual(
            len([l for l in caught.output if "depth auto-restart disabled" in l]), 1
        )
        health = reader.get_health()
        self.assertIs(health["depth_stall_given_up"], True)
        self.assertIs(health["depth_stall_fault"], True)
        # Room in the window does not lift the give-up.
        self.assertFalse(self._poll(reader, 105.0 + 2000.0))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 3)

    def test_one_frame_per_rebuild_still_gives_up(self):
        # Third Grok review 2026-11-26: a sick stream that sends one frame
        # after each rebuild and dies again is not a recovery.
        reader = self._reader()
        for t in (105.0, 115.0, 125.0):
            self.assertTrue(self._poll(reader, t))
            self._session_delivers(reader, t + 0.5, t + 0.5)
        with self.assertLogs(oak_depth_mod.logger, level="ERROR") as caught:
            self.assertFalse(self._poll(reader, 135.0))
        self.assertIn("depth auto-restart disabled", "\n".join(caught.output))
        self.assertIs(reader.get_health()["depth_stall_given_up"], True)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 3)

    def test_a_blip_clears_the_fault_but_not_the_count(self):
        reader = self._reader()
        for t in (105.0, 106.0, 107.0):
            self.assertTrue(self._poll(reader, t))
            reader._depth_run_start_ts = None
            reader._depth_session_recv_ts = None
        self.assertFalse(self._poll(reader, 120.0))
        self.assertIs(reader.get_health()["depth_stall_given_up"], True)
        # One stray frame: the fault clears, the give-up count does not.
        self._session_delivers(reader, 2000.0, 2000.0)
        self.assertFalse(self._poll(reader, 2000.1))
        self.assertIs(reader.get_health()["depth_stall_given_up"], False)
        with self.assertLogs(oak_depth_mod.logger, level="ERROR"):
            self.assertFalse(self._poll(reader, 2010.0))
        self.assertIs(reader.get_health()["depth_stall_given_up"], True)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 3)
        # Depth that then runs for 61 s is a recovery: the next stall
        # restarts again.
        self._session_delivers(reader, 3000.0, 3061.0)
        self.assertFalse(self._poll(reader, 3061.1))  # clears the fault
        self.assertTrue(self._poll(reader, 3070.0))
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 4)

    def test_budget_is_shared_with_the_other_watchdogs(self):
        # Colour + IMU restarts fill the shared budget: depth waits, with
        # no restart of its own, then restarts once they leave the window.
        reader = self._reader()
        reader._color_stall_restart_times = [100.0, 101.0]
        reader._imu_stall_restart_times = [102.0]
        self.assertFalse(self._poll(reader, 110.0))
        self.assertTrue(reader.get_health()["depth_stall_fault"])
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)
        self.assertTrue(self._poll(reader, 102.0 + 901.0))
        self.assertIs(reader.get_health()["depth_stall_fault"], False)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 1)

        # Depth restarts fill it: the colour watchdog latches.
        reader = self._reader()
        for t in (105.0, 106.0, 107.0):
            self.assertTrue(self._poll(reader, t))
        reader._last_depth_recv_ts = 120.0
        self.assertFalse(reader._color_stall_after_poll(120.0, 0.0, True))
        self.assertTrue(reader.get_health()["color_stall_fault"])
        self.assertEqual(reader.get_health()["color_stall_restarts"], 0)

        # And the IMU watchdog latches too.
        reader = self._reader()
        for t in (105.0, 106.0, 107.0):
            self.assertTrue(self._poll(reader, t))
        reader.set_imu_watchdog_enabled(True)
        reader._imu_msg_ts = 100.0
        reader._imu_session_msg_ts = 100.0
        reader._last_depth_recv_ts = 119.95
        self.assertFalse(reader._imu_stall_after_poll(120.0, 0.0))
        self.assertTrue(reader.get_health()["imu_stall_fault"])

    def test_window_pruning_frees_the_budget(self):
        reader = self._reader()
        self._restarts_that_worked(reader, (105.0, 200.0, 300.0))
        self.assertTrue(self._poll(reader, 107.0 + 1200.0))
        self.assertEqual(reader._depth_stall_restart_times, [1307.0])
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 4)


class TestDepthRun(unittest.TestCase):
    """The continuous-run stamp that _poll_depth keeps for the give-up count."""

    def test_a_gap_longer_than_restart_s_starts_a_new_run(self):
        reader = cs._make_reader(_cfg())  # depth_stall_restart_s 0.5
        frame = np.zeros((40, 40), dtype=np.uint16)
        depth_q = cs._PollQueue(lambda: cs._FrameMsg(frame.copy()))
        clock = [100.0]

        def poll_at(t):
            clock[0] = t
            reader._poll_depth(depth_q, cs._EmptyQueue(), np)

        with patch("pi_app.hardware.oak_depth.time.monotonic", lambda: clock[0]):
            for t in (100.0, 100.25, 100.5, 100.75):
                poll_at(t)
            self.assertEqual(reader._depth_run_start_ts, 100.0)
            self.assertEqual(reader._depth_session_recv_ts, 100.75)
            # A 0.5 s gap is not a break (the rule is "longer than").
            poll_at(101.25)
            self.assertEqual(reader._depth_run_start_ts, 100.0)
            # A frame after a longer gap starts a new run: a frame before a
            # gap and a stray one after it never add up to a recovery.
            poll_at(161.0)
            self.assertEqual(reader._depth_run_start_ts, 161.0)


class TestDepthStallSession(unittest.TestCase):
    def setUp(self):
        self._prev_dai = sys.modules.get("depthai")

    def tearDown(self):
        if self._prev_dai is None:
            sys.modules.pop("depthai", None)
        else:
            sys.modules["depthai"] = self._prev_dai

    def _run(self, state, clock, cfg):
        reader = cs._make_reader(cfg, update_rate_hz=_RATE_HZ)
        state.reader = reader
        state.clock = clock
        clock.reader = reader
        cs._install_dai(state)
        with patch("pi_app.hardware.oak_depth.time.monotonic", clock.monotonic), \
                patch("pi_app.hardware.oak_depth.time.sleep", clock.sleep), \
                self.assertLogs(oak_depth_mod.logger, level="WARNING") as caught:
            cs._run_supervisor(reader)
        return reader, "\n".join(caught.output)

    def test_silent_depth_queue_rebuilds_the_session_and_depth_resumes(self):
        t0 = 10000.0
        dead_from = t0 + 1.0
        # Session 1: depth live for 1 s, then silent with NN live (the
        # 11-25 20:51 state). Session 2: depth live again.
        state = _DepthState(lambda session, t: session >= 2 or t < dead_from)
        state.stop_after_session2_frames = 40
        clock = cs._Clock(t=t0)
        reader, text = self._run(state, clock, _cfg())
        self.assertEqual(state.starts, 2, text)
        health = reader.get_health()
        self.assertEqual(health["depth_stall_restarts"], 1)
        self.assertIs(health["depth_stall_fault"], False)
        self.assertEqual(health["color_stall_restarts"], 0)
        self.assertEqual(health["reconnect_count"], 1)
        # The session ended restart_s after the last depth frame (within
        # one loop period), not later.
        restart_ts = reader._depth_stall_restart_times[0]
        last_frame_s1 = restart_ts - _RESTART_S
        self.assertLess(last_frame_s1, dead_from)
        self.assertGreaterEqual(last_frame_s1, dead_from - 1.0 / _RATE_HZ - 1e-9)
        self.assertIn("OAK depth stream stalled", text)
        self.assertEqual(text.count("restarting the OAK session"), 1)
        # Depth resumes in session 2.
        self.assertGreaterEqual(state.depth_served.get(2, 0), 40)
        self.assertGreater(reader._depth_session_recv_ts, restart_ts)

    def test_short_depth_drops_do_not_rebuild(self):
        t0 = 10000.0
        # Three 0.375 s drops (shorter than the 0.5 s restart), NN live.
        gaps = [(t0 + 1.0, t0 + 1.375), (t0 + 2.0, t0 + 2.375), (t0 + 3.0, t0 + 3.375)]

        def live(session, t):
            return not any(a <= t < b for a, b in gaps)

        state = _DepthState(live)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 5.0
        reader, text = self._run(state, clock, _cfg())
        self.assertEqual(state.starts, 1, text)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)
        self.assertNotIn("restarting the OAK session", text)
        # The fake frames are all zeros, so every one is a quality reject:
        # rejected frames still count as received.
        self.assertGreater(reader._depth_quality_reject_count, 0)

    def test_whole_device_stall_is_not_a_depth_stall(self):
        t0 = 10000.0
        # Depth AND detections stop: the colour watchdog's job (its warm-up
        # is 60 s here, so nothing restarts in this run). The fresh gate is
        # scaled with the restart time, as in production (1.5 s < 5 s).
        state = _DepthState(
            lambda session, t: t < t0 + 1.0,
            nn_live=lambda session, t: t < t0 + 1.0,
        )
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 4.0
        reader, text = self._run(state, clock, _cfg(depth_stall_det_fresh_s=0.15))
        self.assertEqual(state.starts, 1, text)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    def test_disabled_watchdog_keeps_the_session(self):
        t0 = 10000.0
        state = _DepthState(lambda session, t: t < t0 + 1.0)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 4.0
        reader, text = self._run(state, clock, _cfg(depth_stall_restart_s=0.0))
        self.assertEqual(state.starts, 1, text)
        self.assertEqual(reader.get_health()["depth_stall_restarts"], 0)

    def test_long_run_then_dead_counts_each_rebuild_and_gives_up(self):
        # Fifth Grok review 2026-11-26 asked whether a finished 60 s run
        # keeps resetting the count while depth is dead. It cannot: the run
        # stamps are cleared at each session start, so the credit belongs
        # to the session that earned it. Session 1 runs depth for 62 s and
        # dies; sessions 2-4 deliver nothing.
        t0 = 10000.0
        state = _DepthState(lambda session, t: session == 1 and t < t0 + 62.0)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 70.0
        reader, text = self._run(state, clock, _cfg())
        health = reader.get_health()
        self.assertEqual(health["depth_stall_restarts"], 3, text)
        self.assertEqual(reader._depth_stall_failed_rebuilds, 3)
        self.assertIs(health["depth_stall_given_up"], True)
        self.assertEqual(state.starts, 4, text)
        self.assertEqual(text.count("depth auto-restart disabled"), 1)
        self.assertGreater(state.depth_served[1], 900)  # ~16 fps for 62 s

    def test_dead_depth_gives_up_and_session_stays_up(self):
        t0 = 10000.0
        # Depth never comes back after the first session's first second.
        state = _DepthState(lambda session, t: session == 1 and t < t0 + 1.0)
        clock = cs._Clock(t=t0)
        clock.stop_after = t0 + 12.0
        reader, text = self._run(state, clock, _cfg())
        health = reader.get_health()
        # 1 + 3 sessions: three rebuilds that bring no depth, then the
        # give-up; the fourth session keeps running with detections live
        # until stop().
        self.assertEqual(health["depth_stall_restarts"], 3, text)
        self.assertIs(health["depth_stall_fault"], True)
        self.assertIs(health["depth_stall_given_up"], True)
        self.assertEqual(state.starts, 4, text)
        self.assertEqual(text.count("depth auto-restart disabled"), 1)


if __name__ == "__main__":
    unittest.main()
