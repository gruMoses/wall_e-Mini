"""IMU stall alarm (2026-11-06 19:34:37).

The IMU stream was dead for 38.8 h and the only sign was ONE journal
warning: get_health() had no IMU field, the 5 s status line kept printing a
normal-looking "IMU 0°→0°", and the JSON log had no event. This covers the
health fields, the per-minute journal reminder, the status line text and the
JSON events.
"""

import logging
import math
import unittest
from unittest.mock import patch

from config import OakDetectionConfig
from pi_app.app.imu_stall import ImuStallMonitor, imu_status_text
from pi_app.hardware import oak_imu as oak_imu_mod
from pi_app.hardware.oak_imu import OakImuReader
from pi_app.tests import test_oak_color_stall as cs
from pi_app.tests.test_imu_steering_stale_heading import _FakeOak


class TestHealthFields(unittest.TestCase):
    def test_no_sample_yet(self):
        health = cs._make_reader(OakDetectionConfig()).get_health()
        self.assertIsNone(health["imu_age_s"])
        self.assertIs(health["imu_stale"], True)
        self.assertIs(health["imu_stall_fault"], False)
        self.assertEqual(health["imu_stall_restarts"], 0)
        self.assertIs(health["imu_watchdog"], False)

    def test_age_and_stale_follow_the_heading_sample(self):
        reader = cs._make_reader(OakDetectionConfig())
        reader.set_imu_watchdog_enabled(True)
        with patch("pi_app.hardware.oak_depth.time.monotonic", lambda: 1000.0):
            reader._imu_state.timestamp = 999.8
            health = reader.get_health()
            self.assertAlmostEqual(health["imu_age_s"], 0.2)
            self.assertIs(health["imu_stale"], False)
            self.assertIs(health["imu_watchdog"], True)
            # Same threshold as OakImuReader's stale age.
            reader._imu_state.timestamp = 1000.0 - oak_imu_mod._STALE_AGE_S - 0.01
            self.assertIs(reader.get_health()["imu_stale"], True)
            reader._imu_stall_fault = True
            self.assertIs(reader.get_health()["imu_stall_fault"], True)


class TestJournalReminder(unittest.TestCase):
    def _reader(self):
        oak = _FakeOak()
        oak.feed(0.0)
        reader = OakImuReader(oak)
        reader.read()
        return oak, reader

    def test_warning_repeats_about_once_a_minute_while_stale(self):
        oak, reader = self._reader()
        clock = {"t": 5000.0}
        with patch("pi_app.hardware.oak_imu.time.monotonic", lambda: clock["t"]), \
                self.assertLogs(oak_imu_mod.logger, level="WARNING") as caught:
            oak.now += 0.6  # sample now older than 0.5 s: stale
            for _ in range(int(130 / 0.1)):  # 130 s of reads at 10 Hz
                clock["t"] += 0.1
                oak.now += 0.1
                reader.read()
        first = [l for l in caught.output if "OAK IMU samples stale (age=" in l]
        again = [l for l in caught.output if "OAK IMU samples still stale after" in l]
        self.assertEqual(len(first), 1)
        self.assertEqual(len(again), 2)  # at ~60 s and ~120 s
        self.assertIn("still stale after 60 s", again[0])

    def test_resume_after_a_long_episode_is_logged_once(self):
        oak, reader = self._reader()
        clock = {"t": 5000.0}
        with patch("pi_app.hardware.oak_imu.time.monotonic", lambda: clock["t"]), \
                self.assertLogs(oak_imu_mod.logger, level="WARNING") as caught:
            oak.now += 0.6
            reader.read()
            clock["t"] += 3.0
            oak.now += 3.0
            reader.read()
            oak.feed(0.0)  # fresh sample
            clock["t"] += 0.1
            reader.read()
            reader.read()
        resumed = [l for l in caught.output if "OAK IMU samples resumed after" in l]
        self.assertEqual(len(resumed), 1)
        self.assertIn("resumed after 3.1 s", resumed[0])

    def test_short_episode_has_no_resume_line(self):
        oak, reader = self._reader()
        clock = {"t": 5000.0}
        logs = []
        handler = logging.Handler()
        handler.emit = lambda record: logs.append(record.getMessage())
        oak_imu_mod.logger.addHandler(handler)
        try:
            with patch("pi_app.hardware.oak_imu.time.monotonic", lambda: clock["t"]):
                oak.now += 0.6
                reader.read()
                clock["t"] += 0.5
                oak.feed(0.0)
                reader.read()
        finally:
            oak_imu_mod.logger.removeHandler(handler)
        self.assertEqual(len([m for m in logs if "samples stale (age=" in m]), 1)
        self.assertFalse([m for m in logs if "resumed" in m])


class TestStatusLine(unittest.TestCase):
    def test_fresh(self):
        status = {"is_available": True, "heading_deg": 12.0, "target_heading_deg": 10.0,
                  "heading_stale": False}
        self.assertEqual(imu_status_text(status, {"imu_age_s": 0.01}), "IMU 12°→10°")

    def test_stale_is_not_a_normal_heading(self):
        status = {"is_available": True, "heading_deg": 0.0, "target_heading_deg": 0.0,
                  "heading_stale": True}
        text = imu_status_text(status, {"imu_age_s": 3725.4, "imu_stall_fault": False})
        self.assertEqual(text, "IMU STALE 3725s (frozen 0°)")
        self.assertNotIn("→", text)
        text = imu_status_text(status, {"imu_age_s": 3725.4, "imu_stall_fault": True})
        self.assertEqual(text, "IMU STALE 3725s (frozen 0°) FAULT")

    def test_stale_from_health_alone(self):
        # An IMU source that does not report freshness, with a dead OAK IMU.
        status = {"is_available": True, "heading_deg": 5.0, "target_heading_deg": 5.0}
        self.assertEqual(
            imu_status_text(status, {"imu_age_s": None, "imu_stale": True, "imu_watchdog": True}),
            "IMU STALE ?s (frozen 5°)",
        )
        # Not the heading source (watchdog off): the OAK IMU age is not shown.
        self.assertEqual(
            imu_status_text(status, {"imu_age_s": None, "imu_stale": True, "imu_watchdog": False}),
            "IMU 5°→5°",
        )

    def test_offline_and_disabled(self):
        self.assertEqual(imu_status_text({"is_available": False}, None), "IMU OFFLINE")
        self.assertEqual(imu_status_text(None, None), "IMU DISABLED")


class TestStallEvents(unittest.TestCase):
    @staticmethod
    def _h(stale, age=None, restarts=0, fault=False):
        return {"imu_stale": stale, "imu_age_s": age,
                "imu_stall_restarts": restarts, "imu_stall_fault": fault}

    def test_dead_stream_restart_and_recovery(self):
        mon = ImuStallMonitor(event_s=5.0)
        self.assertEqual(mon.update(0.0, self._h(False, 0.01)), [])
        self.assertEqual(mon.update(1.0, self._h(True, 0.6)), [])
        self.assertEqual(mon.update(5.9, self._h(True, 5.5)), [])
        ev = mon.update(6.0, self._h(True, 5.6))
        self.assertEqual([e["event"] for e in ev], ["oak_imu_stall"])
        self.assertEqual(ev[0]["stale_for_s"], 5.0)
        self.assertEqual(mon.update(7.0, self._h(True, 6.6)), [])  # once
        ev = mon.update(11.0, self._h(True, 10.6, restarts=1))
        self.assertEqual([e["event"] for e in ev], ["oak_imu_stall_restart"])
        self.assertEqual(ev[0]["imu_stall_restarts"], 1)
        ev = mon.update(19.0, self._h(False, 0.01, restarts=1))
        self.assertEqual([e["event"] for e in ev], ["oak_imu_recovered"])
        self.assertEqual(ev[0]["stalled_for_s"], 18.0)
        self.assertEqual(mon.update(20.0, self._h(False, 0.01, restarts=1)), [])

    def test_short_drops_are_not_events(self):
        mon = ImuStallMonitor(event_s=5.0)
        t = 0.0
        for _ in range(50):  # 2026-10-22: a ~1 s drop every ~4.4 s
            for stale, dur in ((False, 3.4), (True, 1.0)):
                t += dur
                self.assertEqual(mon.update(t, self._h(stale, 0.6 if stale else 0.01)), [])

    def test_fault_latch_is_an_event(self):
        mon = ImuStallMonitor(event_s=5.0)
        mon.update(0.0, self._h(True, 1.0, restarts=3))
        ev = mon.update(1.0, self._h(True, 2.0, restarts=3, fault=True))
        self.assertEqual([e["event"] for e in ev], ["oak_imu_stall_fault"])
        self.assertEqual(mon.update(2.0, self._h(True, 3.0, restarts=3, fault=True)), [])

    def test_no_imu_fields_no_events(self):
        mon = ImuStallMonitor()
        self.assertEqual(mon.update(0.0, None), [])
        self.assertEqual(mon.update(10.0, {"is_stale": True}), [])


if __name__ == "__main__":
    unittest.main()
