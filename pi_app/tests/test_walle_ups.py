"""Tests for bin/walle_ups.py, the Waveshare UPS HAT (B) watcher.

Everything with a side effect (INA219, RTC alarm, systemctl) is a fake, so
these run anywhere. They pin the safety contract from the module doc
(rules of 2026-11-04):
  * present resets; the middle band, I2C errors and implausible readings hold;
  * a discharging sample is absent below S - 0.30 V, above_supply above
    S + 0.15 V, and holds in the band between; the optional edge rule
    (--edge-s, off by default) counts a band sample within 10 s of the last
    present sample;
  * 5 s of counted absent samples halt, in the order kill -> alarm -> sync ->
    poweroff; a 10 s gap without a counted sample resets the count;
  * low battery halts: 7.0 V for 10 s, 6.4 V on one sample; every halt
    arms the 90 s first alarm (the wake check picks a slower poll); a unit
    still stopping after the kill
    stretches the alarm to 300 s; a poweroff that did not happen is retried,
    then forced; every power-off writes the alarm seconds for the shutdown
    hook, which re-arms in the last second;
  * the boot check fails open on doubt; it powers off, uncapped, when the
    pack reads at or below the floor (60 s, or 600 s at or below 6.4 V); it counts band
    samples inside any chain except a simulated one; after 24 power-offs
    the boot continues and run mode grants a grace once.
"""

import importlib.util
import json
import os
import sys
import subprocess
import tempfile
import unittest
from unittest import mock

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MODULE_PATH = os.path.join(_REPO_ROOT, "bin", "walle_ups.py")


def _load():
    spec = importlib.util.spec_from_file_location("walle_ups", _MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclasses + postponed annotations need the module registered
    spec.loader.exec_module(mod)
    return mod


ups = _load()

# The bench brick: S = 8.4 V, so absent < 8.10 V, band 8.10-8.55 V, above > 8.55 V.
TH = ups.Thresholds(supply_v=8.4, present_ma=-40.0, absent_ma=-250.0, loss_s=5.0)
TH_EDGE = ups.Thresholds(supply_v=8.4, present_ma=-40.0, absent_ma=-250.0, loss_s=5.0,
                         edge_s=10.0)
POLL = 0.5
ABSENT_SAMPLES = int(round(TH.loss_s / POLL))  # 10

PRESENT = (8.35, 12.0)
HARD = (7.90, -450.0)     # well below the setpoint: the supply is gone
BAND = (8.30, -450.0)     # discharging near the setpoint: ambiguous
MIDDLE = (8.30, -120.0)   # light discharge: hold
ABOVE = (8.70, -450.0)    # the pack is fuller than the float


class FakeSensor:
    dev_path = None  # boot_check skips the device wait
    addr = 0x42
    bus_num = 1

    def __init__(self, readings):
        self.readings = list(readings)
        self.opened = 0
        self.configured = 0
        self.reopened = 0

    def open(self):
        self.opened += 1

    def configure(self):
        self.configured += 1

    def reopen(self):
        self.reopened += 1

    def read(self, now=None):
        if not self.readings:
            raise AssertionError("fake sensor exhausted")
        item = self.readings.pop(0)
        if isinstance(item, Exception):
            raise item
        v, ma = item
        return ups.Sample(ts=0.0 if now is None else now, batt_v=v, batt_ma=ma)


class FakeRtc:
    def __init__(self, now=1000, fail=False):
        self._now = now
        self.fail = fail
        self.armed = []
        self.cleared = 0

    def now(self):
        return self._now

    def clear(self):
        self.cleared += 1

    def arm(self, seconds):
        if self.fail:
            raise RuntimeError("no rtc")
        self.armed.append(int(seconds))
        return self._now + int(seconds)


class FakeActions:
    def __init__(self, stuck=()):
        self.calls = []
        self.stuck = set(stuck)

    def kill_service(self, name):
        self.calls.append(("kill", name))

    def sync(self):
        self.calls.append(("sync",))

    def poweroff(self, force=False):
        self.calls.append(("poweroff-force",) if force else ("poweroff",))
        return 0

    def is_active(self, name):
        return name in self.stuck


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


HALT_CALLS = [("kill", "wall-e.service"), ("sync",), ("poweroff",)]


class ClassifyTests(unittest.TestCase):
    def test_current_bands(self):
        self.assertEqual(ups.classify(12.0, 8.35, TH), ups.PRESENT)
        self.assertEqual(ups.classify(-40.0, 8.35, TH), ups.PRESENT)
        self.assertEqual(ups.classify(-41.0, 8.35, TH), ups.UNKNOWN)
        self.assertEqual(ups.classify(-249.0, 8.35, TH), ups.UNKNOWN)

    def test_discharging_is_judged_against_the_setpoint(self):
        self.assertEqual(ups.classify(-250.0, 8.09, TH), ups.ABSENT)
        self.assertEqual(ups.classify(-600.0, 7.50, TH), ups.ABSENT)
        self.assertEqual(ups.classify(-450.0, 8.10, TH), ups.DISCHARGE)
        self.assertEqual(ups.classify(-450.0, 8.30, TH), ups.DISCHARGE)
        self.assertEqual(ups.classify(-450.0, 8.55, TH), ups.DISCHARGE)
        self.assertEqual(ups.classify(-450.0, 8.56, TH), ups.ABOVE_SUPPLY)
        # The voltage rules only apply to a discharge.
        self.assertEqual(ups.classify(12.0, 7.0, TH), ups.PRESENT)

    def test_plausible(self):
        self.assertTrue(ups.plausible(ups.Sample(0, 7.5, -500.0), TH))
        for v, ma, ovf in ((0.0, -500.0, False), (3.9, -500.0, False), (9.6, 0.0, False),
                           (7.5, -6000.0, False), (7.5, 0.0, True)):
            self.assertFalse(ups.plausible(ups.Sample(0, v, ma, ovf), TH), (v, ma, ovf))


class TimerTests(unittest.TestCase):
    def test_counts_absent_samples_only(self):
        t = ups.LossTimer(5.0, 0.5, gap_s=10.0)
        now = 0.0
        for _ in range(9):
            now += 0.5
            self.assertFalse(t.update(ups.ABSENT, now))
        for state in (ups.UNKNOWN, ups.ABOVE_SUPPLY, ups.DISCHARGE):
            now += 0.5
            self.assertFalse(t.update(state, now))  # holds
        self.assertAlmostEqual(t.absent_s, 4.5)
        now += 0.5
        self.assertTrue(t.update(ups.ABSENT, now))

    def test_present_resets(self):
        t = ups.LossTimer(5.0, 0.5)
        for i in range(9):
            t.update(ups.ABSENT, i * 0.5)
        self.assertFalse(t.update(ups.PRESENT, 5.0))
        self.assertEqual(t.absent_s, 0.0)

    def test_a_long_gap_resets(self):
        t = ups.LossTimer(5.0, 0.5, gap_s=10.0)
        t.update(ups.ABSENT, 0.0)
        t.update(ups.UNKNOWN, 10.0)
        self.assertAlmostEqual(t.absent_s, 0.5)   # exactly 10 s: still holding
        t.update(ups.UNKNOWN, 10.5)
        self.assertEqual(t.absent_s, 0.0)          # over 10 s: reset

    def test_absent_after_a_long_gap_starts_a_new_count(self):
        # Grok review: 4.5 s of absent, 10 s of hold, then one absent sample
        # must not complete the stale count.
        t = ups.LossTimer(5.0, 0.5, gap_s=10.0)
        for i in range(9):
            t.update(ups.ABSENT, i * 0.5)          # last counted at 4.0 s
        t.update(ups.UNKNOWN, 14.0)                 # exactly 10 s: holds
        self.assertAlmostEqual(t.absent_s, 4.5)
        self.assertFalse(t.update(ups.ABSENT, 14.5))
        self.assertAlmostEqual(t.absent_s, 0.5)

    def test_level_timer(self):
        lt = ups.LevelTimer(7.0, 10.0, 0.5)
        for _ in range(19):
            self.assertFalse(lt.update(ups.UNKNOWN, 6.95))
        self.assertFalse(lt.update(ups.UNKNOWN, None))   # an error holds
        self.assertFalse(lt.update(ups.UNKNOWN, 7.05))   # inside the hysteresis: holds
        self.assertTrue(lt.update(ups.ABSENT, 6.9))
        lt.update(ups.UNKNOWN, 7.2)                       # clearly above: resets
        self.assertEqual(lt.below_s, 0.0)
        lt.update(ups.UNKNOWN, 6.9)
        lt.update(ups.PRESENT, 6.9)                       # present resets
        self.assertEqual(lt.below_s, 0.0)


class WatcherHarness:
    def __init__(self, readings, th=TH, detect_only=False, rtc=None, nohalt=False,
                 chain=None, grace_s=0.0, grace_file=None, stuck=()):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = ups.StateDir(os.path.join(self.tmp.name, "state"))
        if nohalt:
            self.state.ensure()
            open(os.path.join(self.state.path, ups.StateDir.NOHALT), "w").close()
        if chain:
            self.state.write_chain(chain)
        self.status_file = os.path.join(self.tmp.name, "ups_status.json")
        self.arm_file = os.path.join(self.tmp.name, "run", "arm")
        self.sensor = FakeSensor(readings)
        self.actions = FakeActions(stuck=stuck)
        self.rtc = rtc or FakeRtc()
        self.now = 1000.0
        self.w = ups.Watcher(self.sensor, th, self.actions, self.rtc, self.state,
                             status_file=self.status_file, poll_s=POLL,
                             first_alarm_s=90, kill_services=("wall-e.service",),
                             detect_only=detect_only, grace_s=grace_s, now=self.now,
                             grace_file=grace_file, slow_wake_s=600,
                             arm_file=self.arm_file)

    def step(self):
        self.now += POLL
        return self.w.step(self.now)

    def run(self, n):
        return [self.step() for _ in range(n)]

    def status(self):
        with open(self.status_file) as f:
            return json.load(f)

    def armed_file(self):
        try:
            with open(self.arm_file) as f:
                return int(f.read().split()[0])
        except OSError:
            return None

    def events(self):
        path = os.path.join(self.state.path, ups.StateDir.EVENTS)
        if not os.path.exists(path):
            return []
        with open(path) as f:
            return [json.loads(line) for line in f if line.strip()]

    def close(self):
        self.tmp.cleanup()


class WatcherTests(unittest.TestCase):
    def test_edge_rule_catches_a_pull_from_the_float_and_halts_in_order(self):
        # Healthy cells floating at the setpoint: a pull reads in the band.
        h = WatcherHarness([PRESENT] * 3 + [BAND] * ABSENT_SAMPLES, th=TH_EDGE)
        try:
            results = h.run(3 + ABSENT_SAMPLES)
            self.assertEqual(results[:3], ["monitoring"] * 3)
            self.assertEqual(results[3:-1], ["countdown"] * (ABSENT_SAMPLES - 1))
            self.assertEqual(results[-1], "shutdown")
            self.assertEqual(h.actions.calls, HALT_CALLS)
            self.assertEqual(h.rtc.armed, [90])
            loss = h.state.read_loss()
            self.assertEqual(loss["kind"], ups.KIND_EDGE)
            self.assertEqual(loss["reason"], "mains")
            self.assertEqual(h.state.read_chain()["kind"], ups.KIND_EDGE)
            self.assertEqual(h.state.read_chain()["wakes"], 1)   # counts power-offs
            self.assertEqual([e["event"] for e in h.events()], ["loss", "alarm"])
            self.assertEqual(h.status()["state"], "shutdown")
            # Once halting, further steps do nothing new.
            h.sensor.readings.append(PRESENT)
            self.assertEqual(h.step(), "shutdown")
            self.assertEqual(len(h.actions.calls), 3)
        finally:
            h.close()

    def test_edge_rule_is_off_by_default_so_supply_droop_holds(self):
        # Grok review 01a1a7a9: with healthy cells a 40 mV droop under a load
        # step pushes 0.5 A out of the pack right after a floating sample.
        self.assertEqual(ups.DEFAULT_EDGE_S, 0.0)
        h = WatcherHarness([PRESENT] * 3 + [BAND] * 40)
        try:
            self.assertNotIn("shutdown", h.run(43))
        finally:
            h.close()

    def test_hard_absent_halts_without_any_history(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES)
        try:
            self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "shutdown")
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_HARD)
            self.assertEqual(h.state.read_chain()["kind"], ups.KIND_HARD)
        finally:
            h.close()

    def test_band_without_a_recent_present_holds(self):
        # A cold boot onto a supply that sits below the pack.
        h = WatcherHarness([BAND] * 60)
        try:
            results = h.run(60)
            self.assertNotIn("shutdown", results)
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["input"], ups.DISCHARGE)
        finally:
            h.close()

    def test_band_more_than_ten_seconds_after_present_holds(self):
        h = WatcherHarness([PRESENT] + [MIDDLE] * 25 + [BAND] * 40, th=TH_EDGE)
        try:
            self.assertNotIn("shutdown", h.run(66))
        finally:
            h.close()

    def test_top_up_then_lower_supply_never_halts(self):
        # Grok 2026-11-04: charged at 8.4 V, then the supply is set back to
        # 7.5 V. The pack carries the Pi from above the setpoint, then rides
        # down through the band with the supply still connected.
        th = ups.Thresholds(supply_v=7.5, edge_s=10.0)   # even with the edge rule on
        h = WatcherHarness([(8.38, 5.0)] * 4 + [(8.20, -500.0)] * 40
                           + [(7.60, -500.0)] * 40, th=th)
        try:
            self.assertNotIn("shutdown", h.run(84))
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_field_2026_11_04_high_ir_cells_above_a_7_5_v_supply_hold(self):
        # Cells of about 1.1 ohm resting at 8.27 V carried the Pi above a
        # 7.5 V supply; the 2026-11-03 rules called this a loss.
        th = ups.Thresholds(supply_v=7.5)
        h = WatcherHarness([(7.704, -461.0), (7.592, -560.0), (7.672, -552.0)] * 30, th=th)
        try:
            self.assertNotIn("shutdown", h.run(90))
        finally:
            h.close()

    # Field 2026-11-04 13:33-13:40, 7.5 V supply pulled: the pack hovered at
    # the 7.20 V hard line and the count kept restarting for 6 minutes.
    DITHER = [(7.14, -1020.0)] + [(7.27, -570.0), (7.26, -630.0)] * 10
    S75 = ups.Thresholds(supply_v=7.5)

    def test_hysteresis_holds_a_count_through_dithering_at_the_hard_line(self):
        h = WatcherHarness(self.DITHER, th=self.S75)
        try:
            results = h.run(len(self.DITHER))
            self.assertIn("shutdown", results)
            self.assertEqual(results.index("shutdown"), ABSENT_SAMPLES - 1)  # 5 s
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_HARD)
        finally:
            h.close()

    def test_without_hysteresis_the_dither_restarts_the_count(self):
        th = ups.Thresholds(supply_v=7.5, hard_hyst_v=0.0)
        h = WatcherHarness(self.DITHER, th=th)
        try:
            self.assertNotIn("shutdown", h.run(len(self.DITHER)))
        finally:
            h.close()

    def test_hysteresis_stops_above_its_release_level(self):
        h = WatcherHarness([(7.14, -1020.0)] + [(7.32, -600.0)] * 30, th=self.S75)
        try:
            self.assertNotIn("shutdown", h.run(31))
        finally:
            h.close()

    def test_hysteresis_ends_when_the_count_resets(self):
        for reset in ([(7.25, -120.0)] * 22,       # middle band past the 10 s gap
                      [(7.60, 10.0)]):             # one present sample
            h = WatcherHarness([(7.14, -1020.0)] * 3 + reset + [(7.25, -600.0)] * 20,
                               th=self.S75)
            try:
                self.assertNotIn("shutdown", h.run(23 + len(reset)))
            finally:
                h.close()

    def test_hysteresis_never_starts_a_count_after_a_stall(self):
        # A stalled loop (no step() for over 10 s) must not let a band sample
        # continue a stale count: the count has to start below S - 0.30 V.
        h = WatcherHarness([(7.14, -1020.0)] + [(7.25, -600.0)] * 12, th=self.S75)
        try:
            h.step()
            h.now += 11.0                                  # the stall
            results = h.run(12)
            self.assertNotIn("shutdown", results)
            self.assertEqual(h.w.timer.absent_s, 0.0)
        finally:
            h.close()

    def test_one_present_sample_resets_the_countdown(self):
        h = WatcherHarness([HARD] * (ABSENT_SAMPLES - 1) + [PRESENT] + [HARD] * (ABSENT_SAMPLES - 1))
        try:
            results = h.run(2 * ABSENT_SAMPLES - 1)
            self.assertNotIn("shutdown", results)
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["state"], "countdown")
        finally:
            h.close()

    def test_i2c_errors_hold_the_countdown_and_stale_the_status(self):
        errs = [OSError(110, "timeout")] * 3
        h = WatcherHarness([HARD] * 5 + errs + [HARD] * 5)
        try:
            results = h.run(5)
            self.assertEqual(results[-1], "countdown")
            ts_before = h.status()["ts"]
            h.run(3)
            st = h.status()
            self.assertEqual(st["ts"], ts_before)          # the guard's witness stays old
            self.assertGreater(st["ts_loop"], ts_before)   # but the daemon is alive
            self.assertEqual(st["input"], ups.ABSENT)      # last known
            self.assertEqual(st["errors"], 3)
            self.assertAlmostEqual(st["absent_s"], 2.5)
            self.assertEqual(h.run(5)[-1], "shutdown")
        finally:
            h.close()

    def test_implausible_readings_hold_and_never_count(self):
        h = WatcherHarness([HARD] * 5 + [(0.0, -3000.0), (12.0, -450.0), (7.9, -9000.0)] * 2
                           + [HARD] * 5)
        try:
            results = h.run(16)
            self.assertNotIn("shutdown", results[:-1])
            self.assertEqual(results[-1], "shutdown")
            self.assertEqual(h.w.implausible_total, 6)
        finally:
            h.close()

    def test_middle_band_holds(self):
        h = WatcherHarness([HARD] * 5 + [MIDDLE] * 10 + [HARD] * 5)
        try:
            results = h.run(20)
            self.assertEqual(results[-1], "shutdown")
            self.assertNotIn("shutdown", results[:-1])
        finally:
            h.close()

    def test_scattered_absent_samples_do_not_add_up(self):
        h = WatcherHarness(([HARD] * 4 + [MIDDLE] * 25) * 4)
        try:
            self.assertNotIn("shutdown", h.run(116))
        finally:
            h.close()

    def test_pack_above_supply_never_halts(self):
        h = WatcherHarness([ABOVE] * 40)
        try:
            results = h.run(40)
            self.assertEqual(set(results), {"monitoring"})
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["input"], ups.ABOVE_SUPPLY)
        finally:
            h.close()

    def test_low_battery_floor_halts_from_a_hold_state(self):
        # A discharge that never looks like a loss still ends in a clean halt.
        h = WatcherHarness([(6.95, -120.0)] * 20)
        try:
            results = h.run(20)
            self.assertEqual(results[-1], "shutdown")
            self.assertNotIn("shutdown", results[:-1])
            loss = h.state.read_loss()
            self.assertEqual(loss["kind"], ups.KIND_FLOOR)
            self.assertEqual(loss["reason"], "low_battery")
            self.assertEqual(h.rtc.armed, [90])    # only a critical halt polls slowly
            self.assertEqual(h.armed_file(), 90)   # for the shutdown hook
        finally:
            h.close()

    def test_critical_battery_halts_on_one_sample(self):
        h = WatcherHarness([(6.30, -700.0)])
        try:
            self.assertEqual(h.run(1), ["shutdown"])
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_CRIT)
            # A reading under the run load says little about the charge left
            # (field 2026-11-04: 6.34 V at -1.65 A with the cells near 7.5 V).
            self.assertEqual(h.rtc.armed, [90])
            self.assertEqual(h.armed_file(), 90)
        finally:
            h.close()

    def test_sag_from_above_supply_to_critical_halts_at_once(self):
        # Grok review: a load step on 1 ohm cells can go from above_supply to
        # near the HY2120 cutoff between two samples.
        h = WatcherHarness([ABOVE] * 4 + [(6.2, -1500.0)])
        try:
            self.assertEqual(h.run(5)[-1], "shutdown")
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_CRIT)
        finally:
            h.close()

    def test_run_mode_halt_keeps_the_open_chain_count(self):
        # Grok review: fail-open boots followed by run-mode halts must still
        # reach the cap.
        h = WatcherHarness([HARD] * ABSENT_SAMPLES,
                           chain={"kind": ups.KIND_HARD, "wakes": 23, "opened": 5.0})
        try:
            self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "shutdown")
            self.assertEqual(h.state.read_chain(),
                             {"kind": ups.KIND_HARD, "wakes": 24, "opened": 5.0})
        finally:
            h.close()

    def test_low_battery_chain_kind_is_sticky(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES, chain={"kind": ups.KIND_FLOOR, "wakes": 3})
        try:
            h.run(ABSENT_SAMPLES)
            self.assertEqual(h.state.read_chain()["kind"], ups.KIND_FLOOR)
            self.assertEqual(h.state.read_chain()["wakes"], 4)
        finally:
            h.close()

    def test_a_unit_still_stopping_stretches_the_first_alarm(self):
        # Opus review: wall-e stuck in the kernel can outlast a 90 s alarm.
        h = WatcherHarness([HARD] * ABSENT_SAMPLES, stuck={"wall-e.service"})
        try:
            self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "shutdown")
            self.assertEqual(h.rtc.armed, [300])
            self.assertEqual(h.armed_file(), 300)
        finally:
            h.close()

    def test_a_poweroff_that_did_not_happen_is_retried_then_forced(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES)
        try:
            h.run(ABSENT_SAMPLES)
            self.assertEqual(h.actions.calls[-1], ("poweroff",))
            h.run(59)                                  # 29.5 s: nothing yet
            self.assertEqual(h.actions.calls.count(("poweroff",)), 1)
            h.run(1)                                   # 30 s: retry, re-armed
            self.assertEqual(h.actions.calls.count(("poweroff",)), 2)
            self.assertEqual(h.rtc.armed, [90, 90])
            h.run(60)                                  # 60 s: forced
            self.assertEqual(h.actions.calls[-1], ("poweroff-force",))
            events = [e["event"] for e in h.events()]
            self.assertEqual(events.count("poweroff_retry"), 2)
        finally:
            h.close()

    def test_low_voltage_while_present_is_not_low_battery(self):
        h = WatcherHarness([(6.9, 300.0)] * 40)
        try:
            self.assertNotIn("shutdown", h.run(40))
        finally:
            h.close()

    def test_grace_ignores_mains_loss_but_not_low_battery(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES + [(6.3, -600.0)], grace_s=600)
        try:
            results = h.run(ABSENT_SAMPLES)
            self.assertEqual(results[-1], "grace")
            self.assertEqual(h.actions.calls, [])
            self.assertGreater(h.status()["grace_s_left"], 0)
            self.assertEqual(h.run(1)[-1], "shutdown")
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_CRIT)
        finally:
            h.close()

    def test_a_chain_at_the_cap_starts_a_grace_that_survives_a_restart(self):
        with tempfile.TemporaryDirectory() as d:
            grace_file = os.path.join(d, "run", "grace")
            h = WatcherHarness([HARD] * ABSENT_SAMPLES,
                               chain={"kind": ups.KIND_HARD, "wakes": 24},
                               grace_file=grace_file)
            try:
                self.assertIsNone(h.state.read_chain())          # closed at start
                self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "grace")
                self.assertEqual(h.actions.calls, [])
                # A restarted watcher (bus guard, Restart=always) keeps the grace.
                left = ups.read_grace(grace_file, boot_now=h.now)
                self.assertGreater(left, 590.0)
                h2 = WatcherHarness([HARD] * ABSENT_SAMPLES, grace_s=left)
                try:
                    self.assertEqual(h2.run(ABSENT_SAMPLES)[-1], "grace")
                finally:
                    h2.close()
            finally:
                h.close()

    def test_cap_grace_applies_to_a_low_battery_chain_too(self):
        # The boot check only lets a boot through when the pack is above the
        # floor at boot; the low-battery halts stay on during the grace.
        with tempfile.TemporaryDirectory() as d:
            grace_file = os.path.join(d, "run", "grace")
            h = WatcherHarness([HARD] * ABSENT_SAMPLES + [(6.3, -600.0)],
                               chain={"kind": ups.KIND_CRIT, "wakes": 40},
                               grace_file=grace_file)
            try:
                self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "grace")
                self.assertEqual(h.run(1)[-1], "shutdown")   # critical still halts
            finally:
                h.close()

    def test_a_failed_grace_write_keeps_the_chain(self):
        with tempfile.TemporaryDirectory() as d:
            blocker = os.path.join(d, "run")
            open(blocker, "w").close()                    # a file where the dir should be
            h = WatcherHarness([HARD] * ABSENT_SAMPLES,
                               chain={"kind": ups.KIND_HARD, "wakes": 24},
                               grace_file=os.path.join(blocker, "grace"))
            try:
                self.assertEqual(h.state.read_chain()["wakes"], 24)   # kept for a restart
                self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "grace")  # in-process grace
            finally:
                h.close()

    def test_timing_ignores_a_wall_clock_step(self):
        # Grok review 01a1a7b9: NTP steps the wall clock forward after an RTC
        # reset. Timers and the grace run on the boot clock.
        mono = [100.0]

        def clock():
            mono[0] += POLL
            return mono[0]

        tmp = tempfile.TemporaryDirectory()
        try:
            state = ups.StateDir(os.path.join(tmp.name, "state"))
            w = ups.Watcher(FakeSensor([HARD] * 4 + [MIDDLE] + [HARD] * 6), TH, FakeActions(),
                            FakeRtc(), state, status_file=os.path.join(tmp.name, "s.json"),
                            poll_s=POLL, grace_s=600, clock=clock)
            def wall():  # the RTC-reset date, then NTP's step 1.4 years forward
                return (1748538245.0 if mono[0] < 102.6 else 1793812165.0) + mono[0]

            with mock.patch.object(ups.time, "time", side_effect=wall):
                results = [w.step() for _ in range(11)]
            self.assertEqual(results[-1], "grace")             # the grace held across the step
            self.assertAlmostEqual(w.timer.absent_s, 0.0)      # reset by the grace, not the step
        finally:
            tmp.cleanup()

    def test_critical_chain_stays_critical_after_a_floor_halt(self):
        h = WatcherHarness([(6.95, -120.0)] * 20, chain={"kind": ups.KIND_CRIT, "wakes": 5})
        try:
            self.assertEqual(h.run(20)[-1], "shutdown")
            self.assertEqual(h.state.read_loss()["kind"], ups.KIND_FLOOR)
            self.assertEqual(h.state.read_chain()["kind"], ups.KIND_CRIT)
        finally:
            h.close()

    def test_present_or_above_closes_the_chain(self):
        for reading in (PRESENT, ABOVE):
            h = WatcherHarness([reading], chain={"kind": ups.KIND_EDGE, "wakes": 3})
            try:
                self.assertTrue(h.w.chain_open)
                h.step()
                self.assertIsNone(h.state.read_chain())
                self.assertFalse(h.status()["chain_open"])
            finally:
                h.close()

    def test_setpoint_mismatch_is_reported(self):
        h = WatcherHarness([(7.52, 5.0)] * 130)  # floating at 7.5 V, configured 8.4 V
        try:
            h.run(130)
            self.assertTrue(h.status()["supply_mismatch"])
        finally:
            h.close()
        h = WatcherHarness([(8.38, 5.0)] * 130)
        try:
            h.run(130)
            self.assertFalse(h.status()["supply_mismatch"])
        finally:
            h.close()

    def test_a_mains_halt_writes_the_arm_file_and_halting_off_does_not(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES)
        try:
            h.run(ABSENT_SAMPLES)
            self.assertEqual(h.armed_file(), 90)
        finally:
            h.close()
        h = WatcherHarness([HARD] * ABSENT_SAMPLES, nohalt=True)
        try:
            h.run(ABSENT_SAMPLES)
            self.assertIsNone(h.armed_file())   # a later manual poweroff must not wake
        finally:
            h.close()

    def test_detect_only_logs_the_loss_but_never_halts(self):
        h = WatcherHarness([HARD] * (ABSENT_SAMPLES + 2), detect_only=True)
        try:
            results = h.run(ABSENT_SAMPLES)
            self.assertEqual(results[-1], "loss-ignored")
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.rtc.armed, [])
            self.assertFalse(h.state.read_loss()["halt_enabled"])
            self.assertIsNone(h.state.read_chain())
            self.assertEqual(h.status()["mode"], "detect-only")
            self.assertFalse(h.status()["halt_enabled"])
            # The timer restarted; the daemon keeps watching.
            self.assertEqual(h.step(), "countdown")
        finally:
            h.close()

    def test_nohalt_flag_file_disables_the_halt_and_logs_once_a_minute(self):
        h = WatcherHarness([(6.3, -700.0)] * 40, nohalt=True)
        try:
            results = h.run(40)
            self.assertNotIn("shutdown", results)
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["mode"], "nohalt-flag")
            self.assertEqual([e["event"] for e in h.events()], ["loss"])  # not one per second
        finally:
            h.close()

    def test_rtc_failure_still_powers_off(self):
        h = WatcherHarness([HARD] * ABSENT_SAMPLES, rtc=FakeRtc(fail=True))
        try:
            self.assertEqual(h.run(ABSENT_SAMPLES)[-1], "shutdown")
            self.assertEqual(h.actions.calls, HALT_CALLS)
            self.assertEqual([e["event"] for e in h.events()], ["loss", "alarm_failed"])
        finally:
            h.close()

    def test_bus_reopened_after_ten_consecutive_errors(self):
        h = WatcherHarness([OSError(121, "remote io")] * 12 + [PRESENT])
        try:
            h.run(12)
            self.assertEqual(h.sensor.reopened, 1)
            self.assertEqual(h.step(), "monitoring")
            self.assertEqual(h.w.errors_run, 0)
            self.assertEqual(h.w.errors_total, 12)
        finally:
            h.close()

    def test_status_schema(self):
        h = WatcherHarness([PRESENT])
        try:
            h.step()
            st = h.status()
            for key in ("schema", "ts", "ts_loop", "input", "batt_v", "batt_ma", "soc_pct",
                        "supply_v", "supply_mismatch", "absent_s", "loss_s", "floor_s",
                        "floor_v", "state", "mode", "halt_enabled", "grace_s_left",
                        "chain_open", "errors", "implausible", "last_error", "last_loss",
                        "first_alarm_s"):
                self.assertIn(key, st)
            self.assertEqual(st["schema"], ups.SCHEMA)
            self.assertEqual(st["input"], ups.PRESENT)
            self.assertEqual(st["mode"], "armed")
            self.assertIsNone(st["last_loss"])
        finally:
            h.close()


class BootCheckHarness:
    def __init__(self, readings, th=TH, chain=None, nohalt=False, sensor=None):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = ups.StateDir(os.path.join(self.tmp.name, "state"))
        self.grace_file = os.path.join(self.tmp.name, "run", "grace")
        self.clock = FakeClock(5000.0)
        if nohalt:
            self.state.ensure()
            open(os.path.join(self.state.path, ups.StateDir.NOHALT), "w").close()
        if chain is not None:
            self.state.write_chain(chain)
        self.sensor = sensor or FakeSensor(readings)
        self.actions = FakeActions()
        self.rtc = FakeRtc()
        self.th = th

    def run(self, **kw):
        kw.setdefault("wake_s", 60)
        kw.setdefault("slow_wake_s", 600)
        kw.setdefault("chain_max_wakes", 24)
        kw.setdefault("arm_file", os.path.join(self.tmp.name, "run", "arm"))
        return ups.boot_check(self.sensor, self.th, self.actions, self.rtc, self.state,
                              now_fn=self.clock.now, sleep=self.clock.sleep, **kw)

    def events(self):
        with open(os.path.join(self.state.path, ups.StateDir.EVENTS)) as f:
            return [json.loads(line) for line in f if line.strip()]

    def close(self):
        self.tmp.cleanup()


class BootCheckTests(unittest.TestCase):
    def test_present_continues_and_closes_the_chain(self):
        h = BootCheckHarness([HARD] * 3 + [PRESENT] + [HARD] * 20,
                             chain={"kind": ups.KIND_HARD, "wakes": 5})
        try:
            self.assertEqual(h.run(), "present")
            self.assertEqual(h.actions.calls, [])
            self.assertIsNone(h.state.read_chain())
        finally:
            h.close()

    def test_hard_absent_powers_off_and_opens_a_chain(self):
        h = BootCheckHarness([HARD] * 30)
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.rtc.armed, [60])
            with open(os.path.join(h.tmp.name, "run", "arm")) as f:
                self.assertEqual(f.read().split()[0], "60")    # for the shutdown hook
            self.assertEqual(h.actions.calls, [("sync",), ("poweroff",)])
            chain = h.state.read_chain()
            self.assertEqual((chain["kind"], chain["wakes"]), (ups.KIND_HARD, 1))
            self.assertIsNotNone(chain["opened"])
        finally:
            h.close()

    def test_band_without_a_chain_fails_open(self):
        h = BootCheckHarness([BAND] * 30)
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_band_inside_an_edge_chain_powers_off(self):
        h = BootCheckHarness([BAND] * 30, chain={"kind": ups.KIND_EDGE, "wakes": 2})
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.state.read_chain()["wakes"], 3)
            self.assertEqual(h.state.read_chain()["kind"], ups.KIND_EDGE)
        finally:
            h.close()

    def test_band_in_a_hard_or_floor_chain_powers_off(self):
        # Grok review 01a1a7b9: after a hard halt the pack rebounds into the
        # band under the light early-boot load; the outage is still going.
        for kind in (ups.KIND_HARD, ups.KIND_FLOOR):
            h = BootCheckHarness([BAND] * 30, chain={"kind": kind, "wakes": 2})
            try:
                self.assertEqual(h.run(), "poweroff", kind)
                self.assertEqual(h.state.read_chain()["wakes"], 3)
            finally:
                h.close()

    def test_band_in_a_critical_chain_powers_off(self):
        h = BootCheckHarness([BAND] * 30, chain={"kind": ups.KIND_CRIT, "wakes": 2})
        try:
            self.assertEqual(h.run(), "poweroff")
        finally:
            h.close()

    # Field 2026-11-04, 7.5 V supply. A real outage in a hard chain read
    # 7.30-7.35 V at -567..-600 mA; the robot buck back at 7.50 V, with the
    # cells resting above it, read 7.50 V at -270..-305 mA.
    S75 = ups.Thresholds(supply_v=7.5)

    def test_field_real_outage_wakes_in_a_hard_chain_power_off(self):
        for trace in ([(7.33, -580.0)] * 11,
                      [(7.18, -583.0)] * 3 + [(7.24, -578.0)] * 8):
            h = BootCheckHarness(trace, th=self.S75, chain={"kind": ups.KIND_HARD, "wakes": 3})
            try:
                self.assertEqual(h.run(), "poweroff", trace[0])
            finally:
                h.close()

    def test_field_supply_sharing_below_the_pack_boots(self):
        h = BootCheckHarness([(7.50, -290.0), (7.504, -305.0), (7.508, -270.0)] * 4,
                             th=self.S75, chain={"kind": ups.KIND_HARD, "wakes": 3})
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_chain_band_current_boundary(self):
        for ma, want in ((-400.0, "poweroff"), (-399.0, "inconclusive")):
            h = BootCheckHarness([(7.33, ma)] * 11, th=self.S75,
                                 chain={"kind": ups.KIND_HARD, "wakes": 3})
            try:
                self.assertEqual(h.run(), want, ma)
            finally:
                h.close()

    def test_absent_counts_whatever_the_current_above_the_band_cut(self):
        # V < S - 0.30 needs no current gate beyond the -250 mA discharge line.
        h = BootCheckHarness([(7.15, -260.0)] * 11, th=self.S75)
        try:
            self.assertEqual(h.run(), "poweroff")
        finally:
            h.close()

    def test_a_pack_pinned_at_the_setpoint_boots_even_at_a_high_current(self):
        # Grok review 01a1a857: a fuller pack on a 7.5 V supply can push over
        # 400 mA, but the supply pins the node at its setpoint. Only a pack
        # BELOW the pinned window counts (field 2026-11-06, next test).
        for v, want in ((7.50, "inconclusive"), (7.47, "inconclusive"), (7.46, "poweroff"),
                        (7.54, "inconclusive")):
            h = BootCheckHarness([(v, -450.0)] * 11, th=self.S75,
                                 chain={"kind": ups.KIND_HARD, "wakes": 3})
            try:
                self.assertEqual(h.run(), want, v)
            finally:
                h.close()

    def test_field_fresh_cells_above_the_supply_boot_in_a_hard_chain(self):
        # Field 2026-11-06: Kevin swapped in charged cells during a hard chain.
        # With main power back they carried the Pi at 7.568 V, -448 mA, and the
        # wake check powered off 12 times. A halt fires at or under S - 0.30 V,
        # so a pack above S is not the outage rebounding.
        for v in (7.568, 7.60, 7.64):
            h = BootCheckHarness([(v, -448.0)] * 11, th=self.S75,
                                 chain={"kind": ups.KIND_HARD, "wakes": 12})
            try:
                self.assertEqual(h.run(), "inconclusive", v)
                self.assertEqual(h.actions.calls, [])
            finally:
                h.close()

    def test_band_in_a_simulated_chain_fails_open(self):
        # simulate-loss on a pack above the supply must boot back.
        h = BootCheckHarness([BAND] * 30, chain={"kind": ups.KIND_SIM, "wakes": 2})
        try:
            self.assertEqual(h.run(), "inconclusive")
        finally:
            h.close()

    def test_a_pack_below_the_floor_at_boot_powers_off_whatever_the_current(self):
        # Opus review: 6.9 V at -200 mA (the middle band) used to fail open and
        # full-boot a low pack. It polls slowly only when critical: with 1 ohm
        # cells a half-full pack reads about 6.9 V at boot (field 2026-11-04).
        for v, wake in ((6.9, 60), (6.3, 600)):
            for chain in (None, {"kind": ups.KIND_HARD, "wakes": 40}):
                h = BootCheckHarness([(v, -200.0)] * 30, chain=chain)
                try:
                    self.assertEqual(h.run(), "poweroff", (v, chain))
                    self.assertEqual(h.rtc.armed, [wake])
                finally:
                    h.close()

    def test_band_in_a_capped_hard_chain_boots(self):
        h = BootCheckHarness([BAND] * 30, chain={"kind": ups.KIND_HARD, "wakes": 24})
        try:
            self.assertEqual(h.run(), "cap")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_chain_cap_continues_the_boot(self):
        # The boot check only lets the boot through; run mode sees the capped
        # chain, grants the grace and closes the chain.
        h = BootCheckHarness([HARD] * 30, chain={"kind": ups.KIND_HARD, "wakes": 24})
        try:
            self.assertEqual(h.run(), "cap")
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.state.read_chain()["wakes"], 24)
            self.assertEqual(h.events()[-1]["decision"], "cap")
        finally:
            h.close()

    def test_critical_pack_polls_slowly_and_is_never_capped(self):
        h = BootCheckHarness([(6.2, -600.0)] * 30, chain={"kind": ups.KIND_CRIT, "wakes": 40})
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.rtc.armed, [600])
            self.assertEqual(h.state.read_chain()["wakes"], 41)
        finally:
            h.close()

    def test_a_low_pack_is_never_capped(self):
        for v, wake in ((6.8, 60), (6.2, 600)):
            h = BootCheckHarness([(v, -600.0)] * 30, chain={"kind": ups.KIND_CRIT, "wakes": 40})
            try:
                self.assertEqual(h.run(), "poweroff")
                self.assertEqual(h.rtc.armed, [wake])
                self.assertEqual(h.state.read_chain()["wakes"], 41)
            finally:
                h.close()

    def test_a_floor_chain_above_the_floor_at_boot_is_capped(self):
        # 1 ohm cells can halt on the floor with the pack resting near 8 V.
        h = BootCheckHarness([BAND] * 30, chain={"kind": ups.KIND_FLOOR, "wakes": 24})
        try:
            self.assertEqual(h.run(), "cap")
        finally:
            h.close()

    def test_pack_above_supply_fails_open_and_closes_the_chain(self):
        h = BootCheckHarness([ABOVE] * 30, chain={"kind": ups.KIND_EDGE, "wakes": 1})
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
            self.assertIsNone(h.state.read_chain())
        finally:
            h.close()

    def test_field_2026_11_04_high_ir_cells_above_a_7_5_v_supply_boot(self):
        th = ups.Thresholds(supply_v=7.5)
        h = BootCheckHarness([(7.704, -461.0), (7.592, -560.0)] * 15, th=th)
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_mixed_readings_fail_open(self):
        h = BootCheckHarness([HARD, MIDDLE] * 15)
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_errors_only_fail_open(self):
        h = BootCheckHarness([OSError(121, "remote io")] * 30)
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_too_many_bad_samples_fail_open(self):
        h = BootCheckHarness([HARD] * 7 + [OSError(121, "remote io")] * 2 + [(0.0, 0.0)] * 2
                             + [HARD] * 20)
        try:
            self.assertEqual(h.run(), "inconclusive")   # 4 of 11 bad
        finally:
            h.close()

    def test_nohalt_flag_skips(self):
        h = BootCheckHarness([HARD] * 30, nohalt=True)
        try:
            self.assertEqual(h.run(), "nohalt")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_missing_device_fails_open(self):
        sensor = FakeSensor([HARD] * 30)
        sensor.dev_path = "/nonexistent/i2c-99"
        h = BootCheckHarness([], sensor=sensor)
        try:
            self.assertEqual(h.run(dev_wait_s=0.05), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_rtc_failure_still_powers_off(self):
        h = BootCheckHarness([HARD] * 30, chain={"kind": ups.KIND_HARD, "wakes": 0})
        h.rtc.fail = True
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.actions.calls, [("sync",), ("poweroff",)])
        finally:
            h.close()


class ShutdownHookTests(unittest.TestCase):
    HOOK = os.path.join(_REPO_ROOT, "bin", "walle-ups-shutdown-alarm")

    def _run(self, verb, arm_content, uptime="1000.42 900.1"):
        with tempfile.TemporaryDirectory() as d:
            rtc = os.path.join(d, "rtc")
            os.makedirs(rtc)
            with open(os.path.join(rtc, "wakealarm"), "w") as f:
                f.write("untouched\n")
            arm = os.path.join(d, "arm")
            if arm_content is not None:
                with open(arm, "w") as f:
                    f.write(arm_content)
            up = os.path.join(d, "uptime")
            with open(up, "w") as f:
                f.write(uptime + "\n")
            env = dict(os.environ, WALLE_UPS_ARM=arm, WALLE_UPS_RTC=rtc, WALLE_UPS_UPTIME=up)
            proc = subprocess.run(["sh", self.HOOK, verb], env=env, capture_output=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            with open(os.path.join(rtc, "wakealarm")) as f:
                return f.read().strip()

    def test_poweroff_with_a_fresh_arm_file_arms_the_alarm(self):
        self.assertEqual(self._run("poweroff", "90 990\n"), "+90")
        self.assertEqual(self._run("halt", "600 400"), "+600")         # exactly 600 s old

    def test_anything_else_leaves_the_alarm_alone(self):
        self.assertEqual(self._run("reboot", "90 990"), "untouched")
        self.assertEqual(self._run("poweroff", None), "untouched")     # manual poweroff, e-stop
        self.assertEqual(self._run("poweroff", "90 399"), "untouched") # stale: a failed halt
        for junk in ("", "abc", "-5 990", "0 990", "90", "90 x", "9 0 1"):
            self.assertEqual(self._run("poweroff", junk), "untouched", junk)

    def test_the_watcher_writes_what_the_hook_reads(self):
        with tempfile.TemporaryDirectory() as d:
            arm = os.path.join(d, "run", "arm")
            self.assertTrue(ups.write_arm(arm, 90))
            with open(arm) as f:
                seconds, stamp = f.read().split()
            self.assertEqual(seconds, "90")
            self.assertEqual(stamp, str(int(ups.boot_clock())))


class UnitFileTests(unittest.TestCase):
    def _args(self, unit):
        with open(os.path.join(_REPO_ROOT, "bin", unit)) as f:
            line = next(l for l in f if l.startswith("ExecStart="))
        argv = line.split("walle_ups.py", 1)[1].split()
        return ups.build_arg_parser().parse_args(argv)

    def test_both_unit_files_parse_and_agree_on_the_setpoint(self):
        run = self._args("walle-ups.service")
        boot = self._args("walle-ups-wakecheck.service")
        self.assertEqual((run.mode, boot.mode), ("run", "boot-check"))
        self.assertEqual(run.supply_v, boot.supply_v)
        self.assertEqual(run.supply_v, 7.5)
        self.assertEqual(ups.thresholds_from_args(run), ups.thresholds_from_args(boot))


class HelperTests(unittest.TestCase):
    def test_soc_estimate(self):
        self.assertEqual(ups.soc_estimate_pct(6.4), 0)
        self.assertEqual(ups.soc_estimate_pct(7.6), 50)
        self.assertEqual(ups.soc_estimate_pct(8.4), 100)
        last = -1
        for mv in range(6400, 8401, 50):
            soc = ups.soc_estimate_pct(mv / 1000.0)
            self.assertGreaterEqual(soc, last)
            last = soc

    def test_status_file_is_atomic_and_readable(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.json")
            self.assertTrue(ups.write_status_file(path, {"a": 1}))
            with open(path) as f:
                self.assertEqual(json.load(f), {"a": 1})
            self.assertEqual([n for n in os.listdir(d) if n.startswith("s.json.tmp")], [])
            self.assertFalse(ups.write_status_file(os.path.join(d, "no", "dir", "s.json"), {}))

    def test_chain_and_grace_files(self):
        with tempfile.TemporaryDirectory() as d:
            st = ups.StateDir(d)
            self.assertIsNone(st.read_chain())
            self.assertFalse(st.clear_chain())
            st.write_chain({"kind": ups.KIND_EDGE, "wakes": "3"})
            self.assertEqual(st.read_chain()["wakes"], 3)
            self.assertTrue(st.clear_chain())
            g = os.path.join(d, "run", "grace")
            self.assertEqual(ups.read_grace(g, 100.0), 0.0)
            self.assertTrue(ups.write_grace(g, 600, boot_now=100.0))
            self.assertEqual(ups.read_grace(g, 100.0), 600.0)
            self.assertEqual(ups.read_grace(g, 650.0), 50.0)   # read twice: still there
            self.assertEqual(ups.read_grace(g, 800.0), 0.0)

    def test_rtc_arm_rejects_a_readback_in_the_past(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "since_epoch"), "w") as f:
                f.write("1000\n")
            with open(os.path.join(d, "wakealarm"), "w") as f:
                f.write("")
            rtc = ups.Rtc(d)
            with self.assertRaises(ValueError):
                rtc.arm(0)
            # A plain file keeps the literal "+90"; the real sysfs node would
            # return now+90. The read-back check must refuse this.
            with self.assertRaises(RuntimeError):
                rtc.arm(90)
            rtc.clear()
            with open(os.path.join(d, "wakealarm")) as f:
                self.assertEqual(f.read().strip(), "0")

    def test_ina219_byte_swap_and_scaling(self):
        class Bus:
            def read_word_data(self, addr, reg):
                return {ups.REG_SHUNT: 0xEFFF, ups.REG_BUS: 0xDA40}[reg]  # LE words from the chip

        s = ups.Ina219(1, 0x42, 0.1)
        s._bus = Bus()
        sample = s.read(now=1.0)
        # 0xFFEF -> -17 LSB -> -170 uV -> -1.7 mA; 0x40DA -> 2075 * 4 mV = 8.300 V
        self.assertAlmostEqual(sample.batt_ma, -1.7)
        self.assertAlmostEqual(sample.batt_v, 8.300)
        self.assertFalse(sample.overflow)


class RealTraceTests(unittest.TestCase):
    """Raw INA219 register values logged on the bench 2026-11-03 (bus reg,
    shunt reg) with the 8.4 V brick, classified with the module defaults
    except the setpoint. The shunt is 10 mOhm, not the schematic's 0.1R:
    with 0.1 the unplug read as -52 mA and sat in the hold band."""

    BRICK = ups.Thresholds(supply_v=8.4)

    def _ma_v(self, bus_reg, shunt_reg):
        class Bus:
            def read_word_data(self_, addr, reg):
                raw = {ups.REG_SHUNT: shunt_reg & 0xFFFF, ups.REG_BUS: bus_reg}[reg]
                return ((raw & 0xFF) << 8) | (raw >> 8)  # little-endian as the chip sends it

        s = ups.Ina219()
        s._bus = Bus()
        sample = s.read(now=0.0)
        return sample.batt_ma, sample.batt_v

    def test_default_shunt_is_ten_milliohm(self):
        self.assertEqual(ups.DEFAULT_SHUNT_OHM, 0.01)

    def test_unplugged_bench_pi_is_absent(self):
        # 20:51-20:52 with the brick out: bus 7.6 V, shunt -5.2..-7.3 mV.
        for bus_mv, shunt_uv in ((7684, -5240), (7620, -5930), (7564, -7290), (7556, -6940)):
            ma, v = self._ma_v(bus_mv // 4 << 3, shunt_uv // 10)
            self.assertLess(ma, -500.0)
            self.assertEqual(ups.classify(ma, v, self.BRICK), ups.ABSENT, (bus_mv, shunt_uv, ma))

    def test_first_sample_of_the_2026_11_04_pulls_is_absent(self):
        # 08:53:02.97 and 10:47:05: the first absent samples after a pull.
        for v, ma in ((7.988, -519.0), (7.80, -510.0)):
            self.assertEqual(ups.classify(ma, v, self.BRICK), ups.ABSENT)

    def test_brick_connected_is_present(self):
        # Floating all afternoon (+0.2..+1.6 mV) and charging after the replug (+2.0 mV).
        for bus_mv, shunt_uv in ((8348, 230), (8364, 1570), (8300, -390), (8348, 2040)):
            ma, v = self._ma_v(bus_mv // 4 << 3, shunt_uv // 10)
            self.assertEqual(ups.classify(ma, v, self.BRICK), ups.PRESENT, (bus_mv, shunt_uv, ma))

    def test_boot_transient_holds_not_halts(self):
        # 20:00:31, first daemon sample after a reboot on the brick: -1.7 mV.
        ma, v = self._ma_v(8236 // 4 << 3, -170)
        self.assertEqual(ups.classify(ma, v, self.BRICK), ups.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
