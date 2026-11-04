"""Tests for bin/walle_ups.py, the Waveshare UPS HAT (B) watcher.

Everything with a side effect (INA219, RTC alarm, systemctl) is a fake, so
these run anywhere. They pin the safety contract from the module doc:
5 s of ABSENT samples halt; PRESENT resets; UNKNOWN (middle band, I2C
errors) and ABOVE_SUPPLY hold; the halt order is kill -> alarm -> sync ->
poweroff; the boot check fails open on anything but a clear "absent".
"""

import importlib.util
import json
import os
import sys
import tempfile
import unittest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MODULE_PATH = os.path.join(_REPO_ROOT, "bin", "walle_ups.py")


def _load():
    spec = importlib.util.spec_from_file_location("walle_ups", _MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclasses + postponed annotations need the module registered
    spec.loader.exec_module(mod)
    return mod


ups = _load()

TH = ups.Thresholds(supply_v=8.4, gate_margin_v=0.15, present_ma=-40.0,
                    absent_ma=-250.0, loss_s=5.0)
POLL = 0.5
ABSENT_SAMPLES = int(round(TH.loss_s / POLL))  # 10


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
    def __init__(self):
        self.calls = []

    def kill_service(self, name):
        self.calls.append(("kill", name))

    def sync(self):
        self.calls.append(("sync",))

    def poweroff(self):
        self.calls.append(("poweroff",))


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


PRESENT = (8.35, 12.0)
ABSENT = (8.30, -450.0)
MIDDLE = (8.30, -120.0)
ABOVE = (8.30, -450.0)  # with supply 7.6 the pack is above the charger


class ClassifyTests(unittest.TestCase):
    def test_bands(self):
        self.assertEqual(ups.classify(12.0, 8.35, TH), ups.PRESENT)
        self.assertEqual(ups.classify(-40.0, 8.35, TH), ups.PRESENT)
        self.assertEqual(ups.classify(-41.0, 8.35, TH), ups.UNKNOWN)
        self.assertEqual(ups.classify(-249.0, 8.35, TH), ups.UNKNOWN)
        self.assertEqual(ups.classify(-250.0, 8.35, TH), ups.ABSENT)
        self.assertEqual(ups.classify(-600.0, 8.30, TH), ups.ABSENT)

    def test_pack_above_supply_is_not_a_loss(self):
        th = ups.Thresholds(supply_v=7.6)
        self.assertEqual(ups.classify(-450.0, 7.75, th), ups.ABSENT)
        self.assertEqual(ups.classify(-450.0, 7.76, th), ups.ABOVE_SUPPLY)
        self.assertEqual(ups.classify(-450.0, 8.30, th), ups.ABOVE_SUPPLY)
        # The gate only applies to the absent band.
        self.assertEqual(ups.classify(12.0, 8.30, th), ups.PRESENT)


class LossTimerTests(unittest.TestCase):
    def test_counts_absent_samples_only(self):
        t = ups.LossTimer(5.0, 0.5)
        for _ in range(9):
            self.assertFalse(t.update(ups.ABSENT))
        self.assertFalse(t.update(ups.UNKNOWN))      # holds
        self.assertFalse(t.update(ups.ABOVE_SUPPLY)) # holds
        self.assertAlmostEqual(t.absent_s, 4.5)
        self.assertTrue(t.update(ups.ABSENT))

    def test_present_resets(self):
        t = ups.LossTimer(5.0, 0.5)
        for _ in range(9):
            t.update(ups.ABSENT)
        self.assertFalse(t.update(ups.PRESENT))
        self.assertEqual(t.absent_s, 0.0)
        for _ in range(9):
            self.assertFalse(t.update(ups.ABSENT))


class WatcherHarness:
    def __init__(self, readings, th=TH, detect_only=False, rtc=None, nohalt=False):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = ups.StateDir(os.path.join(self.tmp.name, "state"))
        if nohalt:
            self.state.ensure()
            open(os.path.join(self.state.path, ups.StateDir.NOHALT), "w").close()
        self.status_file = os.path.join(self.tmp.name, "ups_status.json")
        self.sensor = FakeSensor(readings)
        self.actions = FakeActions()
        self.rtc = rtc or FakeRtc()
        self.w = ups.Watcher(self.sensor, th, self.actions, self.rtc, self.state,
                             status_file=self.status_file, poll_s=POLL,
                             first_alarm_s=90, kill_services=("wall-e.service",),
                             detect_only=detect_only)
        self.now = 1000.0

    def step(self):
        self.now += POLL
        return self.w.step(self.now)

    def run(self, n):
        return [self.step() for _ in range(n)]

    def status(self):
        with open(self.status_file) as f:
            return json.load(f)

    def events(self):
        path = os.path.join(self.state.path, ups.StateDir.EVENTS)
        if not os.path.exists(path):
            return []
        with open(path) as f:
            return [json.loads(line) for line in f if line.strip()]

    def close(self):
        self.tmp.cleanup()


class WatcherTests(unittest.TestCase):
    def test_five_seconds_absent_halts_in_order(self):
        h = WatcherHarness([PRESENT] * 3 + [ABSENT] * ABSENT_SAMPLES)
        try:
            results = h.run(3 + ABSENT_SAMPLES)
            self.assertEqual(results[:3], ["monitoring"] * 3)
            self.assertEqual(results[3:-1], ["countdown"] * (ABSENT_SAMPLES - 1))
            self.assertEqual(results[-1], "shutdown")
            self.assertEqual(h.actions.calls,
                             [("kill", "wall-e.service"), ("sync",), ("poweroff",)])
            self.assertEqual(h.rtc.armed, [90])
            loss = h.state.read_loss()
            self.assertAlmostEqual(loss["batt_v"], 8.30)
            self.assertTrue(loss["halt_enabled"])
            kinds = [e["event"] for e in h.events()]
            self.assertEqual(kinds, ["loss", "alarm"])
            self.assertEqual(h.status()["state"], "shutdown")
            # Once halting, further steps do nothing new.
            h.sensor.readings.append(PRESENT)
            self.assertEqual(h.step(), "shutdown")
            self.assertEqual(len(h.actions.calls), 3)
        finally:
            h.close()

    def test_one_present_sample_resets_the_countdown(self):
        h = WatcherHarness([ABSENT] * (ABSENT_SAMPLES - 1) + [PRESENT] + [ABSENT] * (ABSENT_SAMPLES - 1))
        try:
            results = h.run(2 * ABSENT_SAMPLES - 1)
            self.assertNotIn("shutdown", results)
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["state"], "countdown")
        finally:
            h.close()

    def test_i2c_errors_hold_the_countdown_and_stale_the_status(self):
        errs = [OSError(110, "timeout")] * 3
        h = WatcherHarness([ABSENT] * 5 + errs + [ABSENT] * 5)
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
            results = h.run(5)
            self.assertEqual(results[-1], "shutdown")
        finally:
            h.close()

    def test_middle_band_holds(self):
        h = WatcherHarness([ABSENT] * 5 + [MIDDLE] * 20 + [ABSENT] * 5)
        try:
            results = h.run(30)
            self.assertEqual(results[-1], "shutdown")
            self.assertNotIn("shutdown", results[:-1])
        finally:
            h.close()

    def test_pack_above_supply_never_halts(self):
        th = ups.Thresholds(supply_v=7.6)
        h = WatcherHarness([ABOVE] * 40, th=th)
        try:
            results = h.run(40)
            self.assertEqual(set(results), {"monitoring"})
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["input"], ups.ABOVE_SUPPLY)
        finally:
            h.close()

    def test_detect_only_logs_the_loss_but_never_halts(self):
        h = WatcherHarness([ABSENT] * (ABSENT_SAMPLES + 2), detect_only=True)
        try:
            results = h.run(ABSENT_SAMPLES)
            self.assertEqual(results[-1], "loss-ignored")
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.rtc.armed, [])
            self.assertFalse(h.state.read_loss()["halt_enabled"])
            self.assertEqual(h.status()["mode"], "detect-only")
            self.assertFalse(h.status()["halt_enabled"])
            # The timer restarted; the daemon keeps watching.
            self.assertEqual(h.step(), "countdown")
        finally:
            h.close()

    def test_nohalt_flag_file_disables_the_halt(self):
        h = WatcherHarness([ABSENT] * ABSENT_SAMPLES, nohalt=True)
        try:
            results = h.run(ABSENT_SAMPLES)
            self.assertEqual(results[-1], "loss-ignored")
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.status()["mode"], "nohalt-flag")
        finally:
            h.close()

    def test_rtc_failure_still_powers_off(self):
        h = WatcherHarness([ABSENT] * ABSENT_SAMPLES, rtc=FakeRtc(fail=True))
        try:
            results = h.run(ABSENT_SAMPLES)
            self.assertEqual(results[-1], "shutdown")
            self.assertEqual(h.actions.calls,
                             [("kill", "wall-e.service"), ("sync",), ("poweroff",)])
            kinds = [e["event"] for e in h.events()]
            self.assertEqual(kinds, ["loss", "alarm_failed"])
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
                        "supply_v", "absent_s", "loss_s", "state", "mode", "halt_enabled",
                        "errors", "last_error", "last_loss", "first_alarm_s"):
                self.assertIn(key, st)
            self.assertEqual(st["schema"], ups.SCHEMA)
            self.assertEqual(st["input"], ups.PRESENT)
            self.assertEqual(st["mode"], "armed")
            self.assertIsNone(st["last_loss"])
        finally:
            h.close()


class BootCheckHarness:
    def __init__(self, readings, th=TH, loss_age_s=None, nohalt=False, sensor=None):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = ups.StateDir(os.path.join(self.tmp.name, "state"))
        self.clock = FakeClock(5000.0)
        if nohalt:
            self.state.ensure()
            open(os.path.join(self.state.path, ups.StateDir.NOHALT), "w").close()
        if loss_age_s is not None:
            self.state.write_loss({"ts": self.clock.t - loss_age_s, "batt_v": 8.2})
        self.sensor = sensor or FakeSensor(readings)
        self.actions = FakeActions()
        self.rtc = FakeRtc()
        self.th = th

    def run(self, **kw):
        kw.setdefault("wake_s", 300)
        kw.setdefault("wake_fast_s", 60)
        kw.setdefault("fast_window_s", 600)
        return ups.boot_check(self.sensor, self.th, self.actions, self.rtc, self.state,
                              now_fn=self.clock.now, sleep=self.clock.sleep, **kw)

    def close(self):
        self.tmp.cleanup()


class BootCheckTests(unittest.TestCase):
    def test_present_continues_boot(self):
        h = BootCheckHarness([ABSENT] * 3 + [PRESENT] + [ABSENT] * 20)
        try:
            self.assertEqual(h.run(), "present")
            self.assertEqual(h.actions.calls, [])
            self.assertEqual(h.rtc.armed, [])
        finally:
            h.close()

    def test_absent_recent_loss_uses_fast_alarm(self):
        h = BootCheckHarness([ABSENT] * 30, loss_age_s=100)
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.rtc.armed, [60])
            self.assertEqual(h.actions.calls, [("sync",), ("poweroff",)])
        finally:
            h.close()

    def test_absent_old_or_missing_loss_uses_slow_alarm(self):
        for age in (None, 601, 86400):
            h = BootCheckHarness([ABSENT] * 30, loss_age_s=age)
            try:
                self.assertEqual(h.run(), "poweroff")
                self.assertEqual(h.rtc.armed, [300])
            finally:
                h.close()

    def test_mixed_readings_fail_open(self):
        h = BootCheckHarness([ABSENT, MIDDLE] * 15)
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

    def test_pack_above_supply_fails_open(self):
        th = ups.Thresholds(supply_v=7.6)
        h = BootCheckHarness([ABOVE] * 30, th=th)
        try:
            self.assertEqual(h.run(), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_nohalt_flag_skips(self):
        h = BootCheckHarness([ABSENT] * 30, nohalt=True)
        try:
            self.assertEqual(h.run(), "nohalt")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_missing_device_fails_open(self):
        sensor = FakeSensor([ABSENT] * 30)
        sensor.dev_path = "/nonexistent/i2c-99"
        h = BootCheckHarness([], sensor=sensor)
        try:
            self.assertEqual(h.run(dev_wait_s=0.05), "inconclusive")
            self.assertEqual(h.actions.calls, [])
        finally:
            h.close()

    def test_rtc_failure_still_powers_off(self):
        h = BootCheckHarness([ABSENT] * 30, loss_age_s=10)
        h.rtc.fail = True
        try:
            self.assertEqual(h.run(), "poweroff")
            self.assertEqual(h.actions.calls, [("sync",), ("poweroff",)])
        finally:
            h.close()


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


if __name__ == "__main__":
    unittest.main()
