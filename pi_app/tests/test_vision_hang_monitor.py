"""Tests for pi_app/app/vision_hang.py (the OAK worker-hang monitor).

Incident 2026-10-04 -> 10-08: the OAK worker thread hung inside depthai's
teardown for four days with pipeline_running True; nothing noticed, MANUAL
ran at the 10 percent stale-depth floor. The monitor must (1) shout at
onset and every minute, (2) never restart while the gate is closed (armed,
switch up, RC dropout), (3) restart exactly once after hang_restart_s with
the gate open for settle_s, (4) stay log-only when the restart knob is 0,
(5) re-arm after a recovery, and (6) respect a persisted restart budget.
"""

import json
import tempfile
import unittest
from pathlib import Path

from pi_app.app.vision_hang import RestartBudget, VisionHangMonitor, restart_permitted


class TestRestartPermitted(unittest.TestCase):
    LOW = 1200

    def test_armed_never(self):
        ok, why = restart_permitted(True, 0.0, 1000, self.LOW)
        self.assertFalse(ok)
        self.assertEqual(why, "armed")
        ok, _ = restart_permitted(True, None, None, self.LOW)
        self.assertFalse(ok)

    def test_fresh_rc_switch_low_allows(self):
        ok, why = restart_permitted(False, 0.05, 1000, self.LOW)
        self.assertTrue(ok)
        self.assertIn("switch low", why)
        ok, _ = restart_permitted(False, 1.0, self.LOW, self.LOW)  # at the threshold
        self.assertTrue(ok)

    def test_fresh_rc_switch_up_blocks(self):
        # RC-stale forced disarm is NOT this case; this is a live link with
        # ch3 high (e.g. flipped up in the last 0.3 s, before the debounce).
        ok, why = restart_permitted(False, 0.05, 1500, self.LOW)
        self.assertFalse(ok)
        self.assertEqual(why, "switch up")
        ok, _ = restart_permitted(False, 0.05, None, self.LOW)
        self.assertFalse(ok)

    def test_short_dropout_blocks(self):
        # 2026-10-04 shape: the hub glitch dropped the Arduino serial; the
        # controller forces is_armed False after 1 s while the switch is up.
        for age in (1.01, 2.0, 15.0, 29.9):
            ok, why = restart_permitted(False, age, 1500, self.LOW, rc_fresh_s=1.0, rc_off_s=30.0)
            self.assertFalse(ok, age)
            self.assertEqual(why, "RC dropout")

    def test_long_silence_allows(self):
        for age in (30.0, 120.0, 86400.0):
            ok, why = restart_permitted(False, age, 1500, self.LOW, rc_fresh_s=1.0, rc_off_s=30.0)
            self.assertTrue(ok, age)
            self.assertIn("no RC for", why)

    def test_no_rc_ever_allows(self):
        ok, why = restart_permitted(False, None, None, self.LOW)
        self.assertTrue(ok)
        self.assertEqual(why, "no RC ever")

    def test_rc_off_zero_means_dropout_never_allows(self):
        ok, _ = restart_permitted(False, 9999.0, 1500, self.LOW, rc_off_s=0.0)
        self.assertFalse(ok)


class TestVisionHangMonitor(unittest.TestCase):
    def _mon(self, restart=60.0, settle=10.0, log=60.0):
        return VisionHangMonitor(hang_restart_s=restart, log_interval_s=log, settle_s=settle)

    def test_not_hung_is_quiet(self):
        m = self._mon()
        for t in (0.0, 10.0, 1000.0):
            a = m.update(t, hung=False, may_restart=True)
            self.assertFalse(a.hung)
            self.assertFalse(a.onset or a.log_now or a.restart or a.recovered)
        self.assertIsNone(m.hung_since)

    def test_onset_logs_once_then_every_interval(self):
        m = self._mon(restart=0.0)
        a = m.update(100.0, hung=True, may_restart=False)
        self.assertTrue(a.hung and a.onset and a.log_now)
        self.assertFalse(a.restart)
        a = m.update(130.0, hung=True, may_restart=False)
        self.assertTrue(a.hung)
        self.assertFalse(a.onset or a.log_now or a.restart)
        self.assertAlmostEqual(a.hung_for_s, 30.0)
        a = m.update(160.0, hung=True, may_restart=False)
        self.assertTrue(a.log_now)
        a = m.update(190.0, hung=True, may_restart=False)
        self.assertFalse(a.log_now)
        a = m.update(220.0, hung=True, may_restart=False)
        self.assertTrue(a.log_now)

    def test_gate_closed_never_restarts_then_one_restart_after_settle(self):
        m = self._mon(restart=60.0, settle=10.0)
        m.update(0.0, hung=True, may_restart=False)
        for t in (30.0, 60.0, 120.0, 600.0):
            a = m.update(t, hung=True, may_restart=False)
            self.assertFalse(a.restart, f"restart fired with the gate closed at t={t}")
            self.assertIsNone(a.restart_in_s)
        self.assertFalse(m.restart_requested)
        # Gate opens (disarm, switch low): the settle window must elapse.
        a = m.update(601.0, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        self.assertAlmostEqual(a.restart_in_s, 10.0)
        a = m.update(610.9, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        a = m.update(611.0, hung=True, may_restart=True)
        self.assertTrue(a.restart)
        self.assertTrue(m.restart_requested)
        a = m.update(612.0, hung=True, may_restart=True)
        self.assertFalse(a.restart)

    def test_gate_reopening_resets_the_settle_clock(self):
        # Switch flipped up for one tick during the countdown: start over.
        m = self._mon(restart=60.0, settle=10.0)
        m.update(0.0, hung=True, may_restart=True)
        a = m.update(59.0, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        self.assertAlmostEqual(a.restart_in_s, 1.0)   # settle met long ago; hang 1 s short
        a = m.update(65.0, hung=True, may_restart=True)
        self.assertTrue(a.restart)  # settle (10 s) and hang (60 s) both met
        self.assertIsNone(a.restart_in_s)  # requested: no further countdown
        m2 = self._mon(restart=60.0, settle=10.0)
        m2.update(0.0, hung=True, may_restart=True)
        m2.update(55.0, hung=True, may_restart=False)   # blip: switch up / RC dropout
        a = m2.update(56.0, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        self.assertAlmostEqual(a.restart_in_s, 10.0)   # settle restarted at 56
        a = m2.update(65.9, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        a = m2.update(66.0, hung=True, may_restart=True)
        self.assertTrue(a.restart)

    def test_settle_counts_time_before_the_hang(self):
        # Robot parked and disarmed for an hour, then the worker hangs:
        # the restart fires at hang_restart_s, not hang_restart_s + settle.
        m = self._mon(restart=60.0, settle=10.0)
        m.update(0.0, hung=False, may_restart=True)
        m.update(3600.0, hung=True, may_restart=True)
        a = m.update(3659.9, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        a = m.update(3660.0, hung=True, may_restart=True)
        self.assertTrue(a.restart)

    def test_restart_disabled_at_zero(self):
        m = self._mon(restart=0.0)
        m.update(0.0, hung=True, may_restart=True)
        for t in (60.0, 3600.0, 86400.0):
            a = m.update(t, hung=True, may_restart=True)
            self.assertTrue(a.hung)
            self.assertFalse(a.restart)
            self.assertIsNone(a.restart_in_s)

    def test_recovery_reports_once_and_rearms(self):
        m = self._mon(restart=60.0, settle=0.0)
        m.update(0.0, hung=True, may_restart=True)
        m.update(30.0, hung=True, may_restart=True)
        a = m.update(40.0, hung=False, may_restart=True)
        self.assertTrue(a.recovered)
        self.assertFalse(a.hung)
        self.assertAlmostEqual(a.hung_for_s, 40.0)
        a = m.update(41.0, hung=False, may_restart=True)
        self.assertFalse(a.recovered)
        a = m.update(100.0, hung=True, may_restart=True)
        self.assertTrue(a.onset)
        a = m.update(159.0, hung=True, may_restart=True)
        self.assertFalse(a.restart)
        a = m.update(160.0, hung=True, may_restart=True)
        self.assertTrue(a.restart)

    def test_short_hang_that_recovers_never_restarts(self):
        m = self._mon(restart=60.0, settle=0.0)
        m.update(0.0, hung=True, may_restart=True)
        a = m.update(45.0, hung=False, may_restart=True)
        self.assertTrue(a.recovered)
        self.assertFalse(m.restart_requested)


class TestRestartBudget(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "sub" / "vision_hang_restarts.json"

    def tearDown(self):
        self._tmp.cleanup()

    def test_three_per_hour_then_denied(self):
        b = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        t0 = 5000.0
        for i in range(3):
            ok, n = b.allow(t0 + 100.0 * i)
            self.assertTrue(ok, i)
            self.assertEqual(n, i + 1)
        ok, n = b.allow(t0 + 300.0)
        self.assertFalse(ok)
        self.assertEqual(n, 3)
        # The file survives for the next process of the same boot.
        data = json.loads(self.path.read_text())
        self.assertEqual(data["boot_id"], "boot-A")
        self.assertEqual(len(data["stamps"]), 3)
        b2 = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        ok, _ = b2.allow(t0 + 400.0)
        self.assertFalse(ok)

    def test_new_boot_starts_a_fresh_count_and_old_stamps_never_prune_by_clock_step(self):
        # Monotonic stamps: an NTP step of the epoch clock changes nothing.
        b = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        for i in range(3):
            self.assertTrue(b.allow(100.0 + i)[0])
        self.assertFalse(b.allow(110.0)[0])
        # Same boot, later process: still denied.
        b_same = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        self.assertFalse(b_same.allow(120.0)[0])
        # A reboot: the boot id differs. The new monotonic clock may read
        # above the old stamps (the previous boot was short), so only the
        # boot id, not the window, can tell the two boots apart.
        b_new = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-B")
        ok, n = b_new.allow(200.0)
        self.assertTrue(ok)
        self.assertEqual(n, 1)
        ok, n = b_new.allow(42.0)   # and a clock below the old stamps, same answer
        self.assertTrue(ok)

    def test_unwritable_stamp_denies(self):
        # A budget that cannot be kept must not become an unbounded loop.
        self.path.parent.mkdir(parents=True)
        self.path.mkdir()   # the "file" is a directory: replace() fails
        b = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        ok, n = b.allow(100.0)
        self.assertFalse(ok)
        self.assertEqual(n, 0)

    def test_window_prunes(self):
        b = RestartBudget(self.path, max_restarts=3, window_s=3600.0, boot_id="boot-A")
        t0 = 5000.0
        for i in range(3):
            self.assertTrue(b.allow(t0 + i)[0])
        self.assertFalse(b.allow(t0 + 10.0)[0])
        ok, n = b.allow(t0 + 3600.0)   # the first stamp has aged out
        self.assertTrue(ok)
        self.assertEqual(n, 3)

    def test_no_cap_at_zero(self):
        b = RestartBudget(self.path, max_restarts=0, window_s=3600.0, boot_id="boot-A")
        for i in range(10):
            self.assertTrue(b.allow(100.0 + i)[0])

    def test_garbage_file_counts_as_empty(self):
        self.path.parent.mkdir(parents=True)
        self.path.write_text("{not json")
        b = RestartBudget(self.path, max_restarts=1, window_s=3600.0, boot_id="boot-A")
        self.assertTrue(b.allow(100.0)[0])
        self.assertFalse(b.allow(101.0)[0])
        self.path.write_text('{"a": 1}')
        self.assertTrue(b.allow(102.0)[0])
        self.path.write_text('[1, 2, 3]')   # the pre-boot-id shape
        self.assertTrue(b.allow(103.0)[0])

    def test_real_boot_id_is_read_when_not_given(self):
        b = RestartBudget(self.path, max_restarts=1, window_s=3600.0)
        self.assertIsInstance(b.boot_id, str)
        self.assertTrue(b.boot_id)


if __name__ == "__main__":
    unittest.main()
