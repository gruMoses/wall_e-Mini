"""Pack-low latch contract of 2026-11-10 (docs/status_audit_2026-11-10.md §3).

VescCanDriver must:
  * use the MINIMUM of the FRESH (< 0.5 s) per-VESC bus readings; none fresh,
    or two that disagree by > 1.5 V, is untrusted and never latches;
  * latch at the trusted voltage < 44.0 V sustained for voltage_shutdown_delay_s;
  * ramp the tracks to zero over pack_low_ramp_s instead of a step;
  * release only at >= 46.0 V with zero throttle in MANUAL, or on a disarm;
  * latch on the BMS lowest cell <= 3.30 V on two FRESH polls, until a disarm;
  * never latch from a dark BMS link;
  * optionally limp home (MANUAL only, speed cap, 42.0 V / 3.15 V floors).

All of these fail on the pre-2026-11-10 driver (max(), no freshness, 39.0 V,
step to RPM 0, release on voltage alone, no BMS input, no drive context).
A fake monotonic clock drives every timer.
"""

from __future__ import annotations

import logging
import struct
import unittest
from dataclasses import replace
from unittest.mock import patch

# No python-can stub: every test replaces _send_rpm, so the driver never
# imports `can` (and a stub here would leak into test_vesc_telemetry.py).
from config import VescConfig  # noqa: E402
from pi_app.control.mapping import CENTER_OUTPUT_VALUE  # noqa: E402
from pi_app.hardware import vesc as vesc_mod  # noqa: E402
from pi_app.hardware.vesc import VescCanDriver  # noqa: E402

LOGGER = "pi_app.hardware.vesc"
MAX_ERPM = 20000


def _status5(v: float) -> bytes:
    return struct.pack(">i", 0) + struct.pack(">h", int(round(v * 10)))


class _Clock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


class _Base(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        p = patch.object(vesc_mod.time, "monotonic", self.clock)
        p.start()
        self.addCleanup(p.stop)
        self.sent: list = []

    def make(self, **overrides) -> VescCanDriver:
        # Most tests start straight in the low band to time the countdown, so
        # the start seed is off unless a test asks for it (TestStartSeed).
        overrides.setdefault("pack_low_seed_on_start", False)
        cfg = replace(VescConfig(), **overrides)
        d = VescCanDriver(left_id=2, right_id=1, max_rpm=MAX_ERPM, vesc_cfg=cfg)
        d._send_rpm = lambda can_id, rpm: self.sent.append((can_id, int(rpm)))  # type: ignore[method-assign]
        # getattr: so each test fails on the old driver for its OWN behaviour,
        # not only because this method did not exist.
        ctx = getattr(d, "set_drive_context", None)
        if ctx is not None:
            ctx(armed=True, manual=True)
        return d

    def volts(self, d: VescCanDriver, left=None, right=None, check: bool = True) -> None:
        if left is not None:
            d._parse_status5("left", _status5(left), self.clock.t)
        if right is not None:
            d._parse_status5("right", _status5(right), self.clock.t)
        if check:
            d._check_voltage_shutdown()

    def hold(self, d: VescCanDriver, seconds: float, left=None, right=None, step: float = 0.1) -> None:
        """Advance the clock, feeding fresh readings and running the check."""
        n = int(round(seconds / step))
        for _ in range(n):
            self.clock.t += step
            self.volts(d, left, right)

    def last_pair(self):
        return self.sent[-2][1], self.sent[-1][1]

    def latch_by_voltage(self, d: VescCanDriver, v: float = 43.5) -> None:
        self.volts(d, v, v)
        self.hold(d, float(d._cfg.voltage_shutdown_delay_s) + 0.2, v, v)
        self.assertTrue(d.get_pack_low_latched())


class TestPackVoltageSelection(_Base):
    def test_min_of_two_fresh_readings(self):
        d = self.make()
        self.volts(d, 44.5, 45.4)
        self.assertAlmostEqual(d.get_voltage(), 44.5, places=2)
        st = d.get_pack_guard_status()
        self.assertEqual(st["voltage_state"], "trusted_both")
        self.assertTrue(st["trusted"])

    def test_min_reading_drives_the_latch(self):
        # Left 43.8 V, right 44.7 V (the right VESC reads ~0.9 V high). The old
        # max() saw 44.7 V and could never start the countdown.
        d = self.make()
        self.volts(d, 43.8, 44.7)
        self.hold(d, 30.2, 43.8, 44.7)
        self.assertTrue(d.get_pack_low_latched())

    def test_stale_reading_is_ignored(self):
        d = self.make()
        self.volts(d, 50.0, 50.0)           # both fresh, healthy
        self.clock.t += 1.0                 # both now 1 s old
        self.volts(d, left=43.0)            # only the left updates
        self.assertAlmostEqual(d.get_voltage(), 43.0, places=2)
        self.assertEqual(d.get_pack_guard_status()["voltage_state"], "trusted_left")
        self.hold(d, 30.2, left=43.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_no_fresh_reading_is_untrusted_and_never_latches(self):
        d = self.make()
        self.volts(d, 43.0, 43.0)           # timer starts
        self.assertIsNotNone(d._low_voltage_since)
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            for _ in range(400):            # 40 s with no new frame
                self.clock.t += 0.1
                d._check_voltage_shutdown()
        self.assertFalse(d.get_pack_low_latched())
        st = d.get_pack_guard_status()
        self.assertEqual(st["voltage_state"], "stale")
        self.assertIsNone(st["voltage_v"])
        self.assertIsNone(d.get_voltage())
        self.assertEqual(sum("UNTRUSTED" in m for m in cm.output), 1)   # edge only

    def test_disagreement_is_untrusted_no_latch_logged_once(self):
        d = self.make()
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            self.volts(d, 43.0, 44.6)       # 1.6 V apart
            self.hold(d, 40.0, 43.0, 44.6)
        self.assertFalse(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["voltage_state"], "disagree")
        self.assertEqual(sum("UNTRUSTED" in m for m in cm.output), 1)
        # 1.5 V apart is still trusted (limit is "more than").
        d2 = self.make()
        self.volts(d2, 43.0, 44.5)
        self.assertEqual(d2.get_pack_guard_status()["voltage_state"], "trusted_both")

    def test_sustained_disagreement_does_not_clear_a_running_countdown(self):
        # Grok round 2: 2 s of disagreement used to restart the 30 s forever.
        d = self.make()
        self.volts(d, 43.0, 43.0)
        self.hold(d, 20.0, 43.0, 43.0)
        self.hold(d, 10.0, 43.0, 44.6)      # straddles 44 V: untrusted, 10 s
        self.assertFalse(d.get_pack_low_latched())
        self.assertIsNotNone(d._low_voltage_since)   # countdown kept
        self.hold(d, 0.1, 43.0, 43.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_disagreement_with_both_readings_low_still_latches(self):
        d = self.make()
        self.volts(d, 41.0, 43.0)           # 2 V apart, but both below 44 V
        self.hold(d, 30.2, 41.0, 43.0)
        self.assertTrue(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["voltage_state"], "disagree")
        # ... and disagreeing data never releases.
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 3.0, 46.5, 49.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_right_reading_alone_is_corrected_for_its_offset(self):
        d = self.make()
        self.volts(d, 50.0, 50.0)
        self.clock.t += 1.0                  # left goes stale
        self.volts(d, right=44.5)            # BMS would see about 43.5 V
        self.assertAlmostEqual(d.get_voltage(), 43.5, places=2)
        self.assertEqual(d.get_pack_guard_status()["voltage_state"], "trusted_right")
        self.hold(d, 30.2, right=44.5)
        self.assertTrue(d.get_pack_low_latched())

    def test_voltage_release_needs_a_drive_context(self):
        d = VescCanDriver(left_id=2, right_id=1, max_rpm=MAX_ERPM, vesc_cfg=VescConfig())
        d._send_rpm = lambda can_id, rpm: None  # type: ignore[method-assign]
        d._trigger_low_voltage_shutdown(43.0)
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 1.0, 47.0, 47.0)
        self.assertTrue(d.get_pack_low_latched())   # no context: no release

    def test_untrusted_while_latched_does_not_release(self):
        d = self.make()
        self.latch_by_voltage(d)
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.volts(d, 47.0, 49.0)           # disagree, both above release
        self.assertTrue(d.get_pack_low_latched())


class TestThresholdAndDelay(_Base):
    def test_default_threshold_44_and_delay_30(self):
        cfg = VescConfig()
        self.assertEqual(cfg.voltage_shutdown_threshold_v, 44.0)
        self.assertEqual(cfg.voltage_shutdown_delay_s, 30.0)
        d = self.make()
        self.volts(d, 43.9, 43.9)
        self.hold(d, 29.5, 43.9, 43.9)
        self.assertFalse(d.get_pack_low_latched())
        self.hold(d, 0.7, 43.9, 43.9)
        self.assertTrue(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["reason"], "pack_voltage")

    def test_exact_threshold_does_not_count(self):
        d = self.make()
        self.volts(d, 44.0, 44.0)
        self.hold(d, 40.0, 44.0, 44.0)
        self.assertFalse(d.get_pack_low_latched())

    def test_one_bad_frame_pauses_but_does_not_restart_the_countdown(self):
        # Grok round 1: one disagreeing / stale / above-threshold frame at
        # 29 s used to restart the full 30 s.
        for bad in ((43.0, 44.6), (44.5, 44.5)):
            d = self.make()
            self.volts(d, 43.0, 43.0)
            self.hold(d, 29.0, 43.0, 43.0)
            self.hold(d, 0.1, *bad)
            self.assertFalse(d.get_pack_low_latched())   # never latches on it
            self.hold(d, 1.2, 43.0, 43.0)
            self.assertTrue(d.get_pack_low_latched(), bad)

    def test_alternating_untrusted_frames_still_latch(self):
        d = self.make()
        self.volts(d, 43.0, 43.0)
        for i in range(320):                 # 32 s, every other frame disagrees
            self.clock.t += 0.1
            if i % 2:
                self.volts(d, 43.0, 44.8)
            else:
                self.volts(d, 43.0, 43.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_a_real_recovery_resets_the_countdown(self):
        d = self.make()
        self.volts(d, 43.0, 43.0)
        self.hold(d, 20.0, 43.0, 43.0)
        self.hold(d, 2.5, 44.5, 44.5)       # back above the threshold > grace
        self.hold(d, 20.0, 43.0, 43.0)
        self.assertFalse(d.get_pack_low_latched())


class TestRampInsteadOfStep(_Base):
    def test_trip_ramps_to_zero_over_ramp_s(self):
        # The control loop is alive (set_tracks every tick), so the trip must
        # not send RPM 0 itself: set_tracks() ramps from full speed to zero.
        d = self.make()
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        self.volts(d, 43.5, 43.5)
        d.set_tracks(254, 254)
        for _ in range(400):                 # ticks at 0.1 s until the trip
            if d.get_pack_low_latched():
                break
            self.clock.t += 0.1
            self.volts(d, 43.5, 43.5)
            if not d.get_pack_low_latched():
                d.set_tracks(254, 254)
        self.assertNotIn((2, 0), self.sent)  # no step to RPM 0 at the trip
        outs = []
        for _ in range(12):                  # 1.2 s of latched ticks
            d.set_tracks(254, 254)
            outs.append(self.last_pair()[0])
            self.clock.t += 0.1
        self.assertEqual(outs[0], full)      # first latched tick: ramp starts at full
        self.assertTrue(all(b <= a for a, b in zip(outs, outs[1:])))  # monotone
        self.assertAlmostEqual(outs[4], full * 0.5, delta=0.11 * full)  # ~0.5 s in
        self.assertEqual(outs[-1], 0)        # zero after the 1.0 s ramp

    def test_requested_stop_is_never_delayed_and_no_reversal(self):
        d = self.make()
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        d.set_tracks(254, 254)
        d._trigger_low_voltage_shutdown(43.0)   # control loop alive: no step
        self.assertNotEqual(self.last_pair(), (0, 0))
        self.clock.t += 0.1
        d.set_tracks(CENTER_OUTPUT_VALUE, 0)     # left: stop, right: full reverse
        self.assertEqual(self.last_pair(), (0, 0))
        self.clock.t += 0.1
        d.set_tracks(254, 254)                   # cannot speed up again
        self.assertEqual(self.last_pair(), (0, 0))
        self.assertLess(0, full)

    def test_trip_with_silent_control_loop_sends_zero(self):
        d = self.make()
        self.volts(d, 43.5, 43.5)
        self.hold(d, 30.2, 43.5, 43.5)           # no set_tracks at all
        self.assertTrue(d.get_pack_low_latched())
        self.assertIn((2, 0), self.sent)
        self.assertIn((1, 0), self.sent)


class TestReleaseRules(_Base):
    def test_voltage_release_needs_zero_throttle(self):
        d = self.make()
        self.latch_by_voltage(d)
        d.set_tracks(200, 200)
        self.hold(d, 1.0, 46.5, 46.5)
        self.assertTrue(d.get_pack_low_latched())       # throttle held
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 0.1, 46.5, 46.5)
        self.assertFalse(d.get_pack_low_latched())

    def test_voltage_release_needs_46_v(self):
        d = self.make()
        self.latch_by_voltage(d)
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 5.0, 45.9, 45.9)
        self.assertTrue(d.get_pack_low_latched())
        self.hold(d, 0.1, 46.0, 46.0)
        self.assertFalse(d.get_pack_low_latched())

    def test_autonomous_mode_releases_only_by_disarm(self):
        d = self.make()
        self.latch_by_voltage(d)
        d.set_drive_context(armed=True, manual=False)
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 2.0, 47.0, 47.0)
        self.assertTrue(d.get_pack_low_latched())
        d.set_drive_context(armed=False, manual=True)
        self.assertFalse(d.get_pack_low_latched())

    def test_disarm_releases_and_drive_resumes(self):
        d = self.make()
        self.latch_by_voltage(d, 43.5)
        self.hold(d, 1.0, 44.5, 44.5)                    # rests between 44 and 46
        d.set_drive_context(armed=False, manual=True)
        self.assertFalse(d.get_pack_low_latched())
        d.set_drive_context(armed=True, manual=True)
        self.hold(d, 1.0, 44.5, 44.5)
        self.assertFalse(d.get_pack_low_latched())
        d.set_tracks(254, 254)
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        self.assertEqual(self.last_pair(), (full, full))  # full drive (limp off)

    def test_rearm_between_44_and_46_retrips_after_full_delay(self):
        d = self.make()
        self.latch_by_voltage(d, 43.5)
        self.hold(d, 1.0, 45.0, 45.0)
        d.set_drive_context(armed=False, manual=True)
        d.set_drive_context(armed=True, manual=True)
        # Driving sags it under 44 again: a NEW full 30 s countdown is needed.
        self.hold(d, 29.0, 43.8, 43.8)
        self.assertFalse(d.get_pack_low_latched())
        self.hold(d, 1.5, 43.8, 43.8)
        self.assertTrue(d.get_pack_low_latched())

    def test_rearm_below_44_gets_a_fresh_window_manual_only(self):
        # A latch must never need a restart (Grok round 1): the disarm gives a
        # new countdown, but only MANUAL may drive until the pack recovers.
        d = self.make()
        self.latch_by_voltage(d, 43.5)
        d.set_drive_context(armed=False, manual=True)
        self.assertFalse(d.get_pack_low_latched())
        d.set_drive_context(armed=True, manual=True)
        self.hold(d, 1.0, 43.5, 43.5)
        self.assertFalse(d.get_pack_low_latched())
        self.assertTrue(d.get_pack_guard_status()["post_latch"])
        d.set_tracks(200, 200)
        self.assertGreater(self.last_pair()[0], 0)        # MANUAL drives
        d.set_drive_context(armed=True, manual=False)    # FOLLOW_ME entered
        for _ in range(12):
            self.clock.t += 0.1
            d.set_tracks(200, 200)
        self.assertEqual(self.last_pair(), (0, 0))       # autonomous ramps to 0
        d.set_drive_context(armed=True, manual=True)
        self.hold(d, 29.5, 43.5, 43.5)
        self.assertTrue(d.get_pack_low_latched())        # new 30 s ran out

    def test_disarm_restarts_the_cell_count(self):
        d = self.make()
        d.set_bms_cell_reading(3.25, poll_id=1, age_s=1.0)
        d.set_bms_cell_reading(3.25, poll_id=2, age_s=1.0)
        self.assertTrue(d.get_pack_low_latched())
        d.set_drive_context(armed=False, manual=True)
        d.set_drive_context(armed=True, manual=True)
        d.set_bms_cell_reading(3.25, poll_id=3, age_s=1.0)
        self.assertFalse(d.get_pack_low_latched())       # needs two polls again
        d.set_bms_cell_reading(3.25, poll_id=4, age_s=1.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_stop_does_not_count_as_zero_throttle(self):
        # Charger inhibit and disarm call stop(); that is a forced zero, not
        # the operator centring the sticks.
        d = self.make()
        self.latch_by_voltage(d)
        d.set_tracks(200, 200)
        d.stop()
        self.hold(d, 1.0, 47.0, 47.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_set_tracks_after_stop_cannot_climb_during_ramp(self):
        d = self.make()
        d.set_tracks(254, 254)
        d._trigger_low_voltage_shutdown(43.0)
        self.clock.t += 0.1
        d.set_tracks(254, 254)
        self.assertGreater(self.last_pair()[0], 0)        # ramp running
        d.stop()
        self.clock.t += 0.05
        d.set_tracks(254, 254)
        self.assertEqual(self.last_pair(), (0, 0))

    def test_bms_cell_latch_not_released_by_voltage(self):
        d = self.make()
        self.volts(d, 47.0, 47.0)
        d.set_bms_cell_reading(3.29, poll_id=1, age_s=1.0)
        d.set_bms_cell_reading(3.28, poll_id=2, age_s=1.0)
        self.assertTrue(d.get_pack_low_latched())
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 2.0, 47.0, 47.0)
        self.assertTrue(d.get_pack_low_latched())
        d.set_drive_context(armed=False, manual=True)
        self.assertFalse(d.get_pack_low_latched())


class TestBmsCellTrigger(_Base):
    def test_two_fresh_low_polls_latch(self):
        d = self.make()
        self.volts(d, 47.0, 47.0)
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            d.set_bms_cell_reading(3.30, poll_id=10.0, age_s=2.0)
            self.assertFalse(d.get_pack_low_latched())
            d.set_bms_cell_reading(3.30, poll_id=10.0, age_s=2.5)   # same poll: no count
            self.assertFalse(d.get_pack_low_latched())
            d.set_bms_cell_reading(3.29, poll_id=18.0, age_s=1.0)
        self.assertTrue(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["reason"], "bms_cell")
        self.assertTrue(any("bms_cell" in m for m in cm.output))

    def test_a_stale_gap_breaks_consecutive(self):
        d = self.make()
        d.set_bms_cell_reading(3.25, poll_id=1, age_s=1.0)
        d.set_bms_cell_reading(3.25, poll_id=1, age_s=30.0)      # link went dark
        d.set_bms_cell_reading(3.25, poll_id=2, age_s=1.0)
        self.assertFalse(d.get_pack_low_latched())

    def test_a_good_poll_resets_the_count(self):
        d = self.make()
        d.set_bms_cell_reading(3.25, poll_id=1, age_s=1.0)
        d.set_bms_cell_reading(3.40, poll_id=2, age_s=1.0)
        d.set_bms_cell_reading(3.25, poll_id=3, age_s=1.0)
        self.assertFalse(d.get_pack_low_latched())

    def test_stale_bms_data_never_latches(self):
        d = self.make()
        for i in range(10):
            d.set_bms_cell_reading(3.10, poll_id=i, age_s=25.0)
        self.assertFalse(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["bms_cell_state"], "stale")

    def test_dark_bms_link_never_latches(self):
        d = self.make()
        for _ in range(10):
            d.set_bms_cell_reading(None, poll_id=None, age_s=None)
            self.clock.t += 10.0
        d.set_bms_cell_reading(3.10, poll_id=5, age_s=None)    # never polled
        self.assertFalse(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["bms_cell_state"], "no_data")

    def test_implausible_cell_is_ignored(self):
        d = self.make()
        for i in range(5):
            d.set_bms_cell_reading(0.0, poll_id=i, age_s=1.0)
        self.assertFalse(d.get_pack_low_latched())


class TestLimpHome(_Base):
    def _limp(self) -> VescCanDriver:
        d = self.make(pack_low_limp_enabled=True)
        self.latch_by_voltage(d, 43.5)
        d.set_drive_context(armed=False, manual=True)
        d.set_drive_context(armed=True, manual=True)
        return d

    def test_off_by_default(self):
        self.assertFalse(VescConfig().pack_low_limp_enabled)

    def test_limp_caps_speed_in_manual(self):
        d = self._limp()
        st = d.get_pack_guard_status()
        self.assertTrue(st["limp_active"])
        self.assertFalse(st["latched"])
        self.hold(d, 1.0, 43.5, 43.5)                     # 44 V rule no longer applies
        self.assertFalse(d.get_pack_low_latched())
        d.set_tracks(254, 0)
        cap = VescCanDriver._byte_to_rpm(CENTER_OUTPUT_VALUE + 32, MAX_ERPM)
        self.assertEqual(self.last_pair(), (cap, -cap))

    def test_limp_never_drives_autonomous(self):
        d = self._limp()
        d.set_tracks(150, 150)
        moving = self.last_pair()
        self.assertGreater(moving[0], 0)
        d.set_drive_context(armed=True, manual=False)
        outs = []
        for _ in range(15):
            self.clock.t += 0.1
            d.set_tracks(150, 150)
            outs.append(self.last_pair()[0])
        self.assertEqual(outs[-1], 0)
        self.assertTrue(all(b <= a for a, b in zip(outs, outs[1:])))

    def test_limp_floor_trips_after_limp_delay(self):
        d = self._limp()
        self.volts(d, 41.9, 41.9)
        self.hold(d, 4.5, 41.9, 41.9)
        self.assertFalse(d.get_pack_low_latched())
        self.hold(d, 0.7, 41.9, 41.9)
        self.assertTrue(d.get_pack_low_latched())
        self.assertEqual(d.get_pack_guard_status()["reason"], "limp_floor")

    def test_limp_uses_the_lower_cell_floor(self):
        d = self._limp()
        d.set_bms_cell_reading(3.25, poll_id=1, age_s=1.0)
        d.set_bms_cell_reading(3.25, poll_id=2, age_s=1.0)
        self.assertFalse(d.get_pack_low_latched())
        d.set_bms_cell_reading(3.14, poll_id=3, age_s=1.0)
        d.set_bms_cell_reading(3.14, poll_id=4, age_s=1.0)
        self.assertTrue(d.get_pack_low_latched())

    def test_limp_ends_at_46_v_with_zero_throttle(self):
        d = self._limp()
        d.set_tracks(CENTER_OUTPUT_VALUE, CENTER_OUTPUT_VALUE)
        self.hold(d, 0.2, 46.1, 46.1)
        self.assertFalse(d.get_pack_guard_status()["limp_active"])

    def test_without_limp_no_cap_after_disarm(self):
        d = self.make()
        self.latch_by_voltage(d, 43.5)
        self.hold(d, 0.5, 44.5, 44.5)
        d.set_drive_context(armed=False, manual=True)
        d.set_drive_context(armed=True, manual=True)
        st = d.get_pack_guard_status()
        self.assertFalse(st["limp_active"])
        self.assertTrue(st["post_latch"])                # MANUAL-only session
        d.set_tracks(254, 254)
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        self.assertEqual(self.last_pair(), (full, full))  # no speed cap


class TestTelemetrySurface(_Base):
    def test_get_telemetry_reports_state(self):
        d = self.make()
        d._parse_status("left", struct.pack(">ihh", 100, 0, 0), self.clock.t)
        self.volts(d, 45.0, 45.9)
        t = d.get_telemetry()
        self.assertAlmostEqual(t.voltage_v, 45.0, places=2)
        self.assertEqual(t.pack_voltage_state, "trusted_both")
        self.assertTrue(t.pack_voltage_trusted)
        self.assertIsNone(t.pack_low_reason)
        self.clock.t += 1.0
        d._check_voltage_shutdown()
        t = d.get_telemetry()
        self.assertIsNone(t.voltage_v)
        self.assertFalse(t.pack_voltage_trusted)

    def test_latch_edges_logged_at_warning_once(self):
        d = self.make()
        logging.getLogger(LOGGER).setLevel(logging.DEBUG)
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            self.latch_by_voltage(d, 43.5)
            self.hold(d, 5.0, 43.5, 43.5)
        self.assertEqual(sum("ENGAGED" in m for m in cm.output), 1)


class TestRestrictedState(_Base):
    """Voltage not cross-checked for pack_voltage_untrusted_limit_s (10 s):
    MANUAL capped at the limp speed, autonomous ramps to zero. Not the latch."""

    CAP_B = 32

    def cap_rpm(self) -> int:
        return VescCanDriver._byte_to_rpm(CENTER_OUTPUT_VALUE + self.CAP_B, MAX_ERPM)

    def assert_restricted_effects(self, d):
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        st = d.get_pack_guard_status()
        self.assertTrue(st["restricted"])
        self.assertFalse(st["latched"])              # no low-voltage claim
        self.assertIsNone(st["reason"])
        d.set_tracks(254, 254)
        self.assertEqual(self.last_pair(), (self.cap_rpm(), self.cap_rpm()))
        self.assertLess(self.cap_rpm(), full)
        d.set_drive_context(armed=True, manual=False)   # FOLLOW_ME
        for _ in range(12):
            self.clock.t += 0.1
            d.set_tracks(254, 254)
        self.assertEqual(self.last_pair(), (0, 0))
        d.set_drive_context(armed=True, manual=True)

    def test_disagreement_straddling_44_v_past_the_limit_restricts(self):
        d = self.make()
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            self.volts(d, 43.0, 44.8)
            self.hold(d, 9.5, 43.0, 44.8)
            self.assertFalse(d.get_pack_guard_status()["restricted"])
            self.hold(d, 0.7, 43.0, 44.8)
        self.assert_restricted_effects(d)
        self.assertEqual(sum("RESTRICTED" in m for m in cm.output), 1)

    def test_lone_high_reader_restricts(self):
        # Left VESC silent, right one stuck reading high: it cannot be
        # cross-checked, so autonomy must not depend on it.
        d = self.make()
        self.volts(d, 47.0, 48.0)
        self.clock.t += 1.0
        self.hold(d, 10.5, right=48.0)
        self.assertEqual(d.get_pack_guard_status()["voltage_state"], "trusted_right")
        self.assert_restricted_effects(d)

    def test_short_glitch_does_not_restrict(self):
        d = self.make()
        self.volts(d, 47.0, 47.0)
        for _ in range(3):
            self.hold(d, 3.0, 47.0, 49.0)           # 3 s disagreement
            self.hold(d, 2.5, 47.0, 47.0)           # cross-checked again > 2 s
        self.clock.t += 0.3                          # a 0.3 s CAN gap
        d._check_voltage_shutdown()
        self.hold(d, 0.5, 47.0, 47.0)
        self.assertFalse(d.get_pack_guard_status()["restricted"])
        d.set_tracks(254, 254)
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        self.assertEqual(self.last_pair(), (full, full))

    def test_bad_time_accumulates_across_short_good_gaps(self):
        # Grok 01a1c86b: 9.9 s bad / 0.1 s good used to reset the 10 s clock.
        d = self.make()
        self.volts(d, 47.0, 47.0)
        for _ in range(3):
            self.hold(d, 9.9, 47.0, 49.0)
            self.hold(d, 0.1, 47.0, 47.0)
        self.assertTrue(d.get_pack_guard_status()["restricted"])

    def test_below_floor_frame_does_not_pulse_zero_while_driving(self):
        d = self.make()
        full = VescCanDriver._byte_to_rpm(254, MAX_ERPM)
        self.volts(d, 47.0, 47.0)
        d.set_tracks(254, 254)
        self.sent.clear()
        self.clock.t += 0.03
        self.volts(d, 6.0, 6.0)                     # one garbage frame pair
        self.assertNotIn((2, 0), self.sent)         # no brake pulse
        self.clock.t += 0.03
        self.volts(d, 47.0, 47.0)
        d.set_tracks(254, 254)
        self.assertEqual(self.last_pair(), (full, full))

    def test_recovery_needs_confirm_and_zero_throttle(self):
        d = self.make()
        self.volts(d, 43.0, 44.8)
        self.hold(d, 10.5, 43.0, 44.8)
        self.assertTrue(d.get_pack_guard_status()["restricted"])
        d.set_drive_context(armed=True, manual=True, request_neutral=False)
        self.hold(d, 1.0, 46.0, 46.0)
        self.assertTrue(d.get_pack_guard_status()["restricted"])   # confirm not met
        self.hold(d, 2.0, 46.0, 46.0)
        self.assertTrue(d.get_pack_guard_status()["restricted"])   # throttle held
        d.set_drive_context(armed=True, manual=True, request_neutral=True)
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            self.hold(d, 0.1, 46.0, 46.0)
        self.assertFalse(d.get_pack_guard_status()["restricted"])
        self.assertTrue(any("RESTRICTED state ended" in m for m in cm.output))

    def test_disarm_ends_it_only_with_cross_checked_data(self):
        d = self.make()
        self.volts(d, 43.0, 44.8)
        self.hold(d, 10.5, 43.0, 44.8)
        d.set_drive_context(armed=False, manual=True)
        d.set_drive_context(armed=True, manual=True)
        self.assertTrue(d.get_pack_guard_status()["restricted"])   # still disagreeing
        self.hold(d, 0.2, 46.0, 46.0)
        d.set_drive_context(armed=False, manual=True)
        self.assertFalse(d.get_pack_guard_status()["restricted"])

    def test_frozen_reader_is_excluded_then_restricts(self):
        d = self.make()
        self.volts(d, 47.0, 47.0)
        for i in range(215):                          # 21.5 s: left frozen, right moves
            self.clock.t += 0.1
            self.volts(d, 47.0, 47.0 if i % 2 else 47.1)
        st = d.get_pack_guard_status()
        self.assertEqual(st["frozen_reader"], "left")
        self.assertEqual(st["voltage_state"], "trusted_right")
        for i in range(105):
            self.clock.t += 0.1
            self.volts(d, 47.0, 47.0 if i % 2 else 47.1)
        self.assertTrue(d.get_pack_guard_status()["restricted"])

    def test_both_still_at_rest_is_not_frozen(self):
        d = self.make()
        self.volts(d, 47.0, 47.9)
        self.hold(d, 60.0, 47.0, 47.9)
        st = d.get_pack_guard_status()
        self.assertIsNone(st["frozen_reader"])
        self.assertFalse(st["restricted"])

    def test_below_floor_with_frames_restricts_and_never_latches(self):
        d = self.make()
        self.volts(d, 6.8, 6.8)
        self.hold(d, 40.0, 6.8, 6.8)
        st = d.get_pack_guard_status()
        self.assertEqual(st["voltage_state"], "below_floor")
        self.assertTrue(st["restricted"])
        self.assertFalse(st["latched"])


class TestStartSeed(_Base):
    """pack_low_seed_on_start (default True): a service restart on a low pack
    must not hand out a fresh 30 s of full drive."""

    def test_restart_at_43_5_v_latches_after_2_s_not_30(self):
        self.assertTrue(VescConfig().pack_low_seed_on_start)
        d = self.make(pack_low_seed_on_start=True)
        with self.assertLogs(LOGGER, level="WARNING") as cm:
            self.volts(d, 43.5, 44.4)                 # first trusted reading
            self.hold(d, 1.8, 43.5, 44.4)
            self.assertFalse(d.get_pack_low_latched())
            self.hold(d, 0.4, 43.5, 44.4)
        self.assertTrue(d.get_pack_low_latched())
        self.assertTrue(any("seeded" in m for m in cm.output))

    def test_restart_on_a_healthy_pack_is_not_seeded(self):
        d = self.make(pack_low_seed_on_start=True)
        self.volts(d, 45.0, 45.9)
        self.hold(d, 1.0, 45.0, 45.9)
        self.hold(d, 25.0, 43.5, 44.4)
        self.assertFalse(d.get_pack_low_latched())    # normal 30 s countdown
        self.hold(d, 5.5, 43.5, 44.4)
        self.assertTrue(d.get_pack_low_latched())

    def test_a_lone_bad_first_reader_cannot_seed(self):
        d = self.make(pack_low_seed_on_start=True)
        self.volts(d, left=43.0)                     # only one reader, low
        self.clock.t += 0.1
        self.volts(d, 46.0, 46.5)                    # both healthy
        self.hold(d, 5.0, 46.0, 46.5)
        self.assertFalse(d.get_pack_low_latched())

    def test_seed_off_gives_the_full_delay(self):
        d = self.make(pack_low_seed_on_start=False)
        self.volts(d, 43.5, 44.4)
        self.hold(d, 25.0, 43.5, 44.4)
        self.assertFalse(d.get_pack_low_latched())


if __name__ == "__main__":
    unittest.main()
