"""Tests for bin/i2c_bus_guard.py, the I2C bus 1 auto-unlock guard.

bin/ is not a package, so the script is loaded via importlib straight from
its file path (no sys.path changes needed -- the script is stdlib-only, so
unlike scripts/upsPlus_power_daemon.py there is nothing to stub before
import).

All hardware access in the guard goes through one small Hardware class
(read_sda / set_pin / unbind / bind). Every test below except the narrow
"real Hardware plumbing" group replaces that class entirely with
FakeHardware, which records every call in order and plays back a scripted
list of SDA readings -- so detection and recovery are exercised as pure
logic, with no subprocess or sysfs involved.
"""

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_MODULE_PATH = os.path.join(_REPO_ROOT, "bin", "i2c_bus_guard.py")


def _load_guard_module():
    spec = importlib.util.spec_from_file_location("i2c_bus_guard", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    # The module must be registered in sys.modules before exec_module runs:
    # the script's @dataclass (GuardConfig) resolves its annotations by
    # looking up sys.modules[cls.__module__], which is None otherwise.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


guard = _load_guard_module()


def _noop_sleep(_seconds: float) -> None:
    """Stands in for time.sleep in every test -- no real waiting."""


class FakeHardware:
    """Records every call in order; read_sda() plays back a scripted list
    of levels (repeating the last entry once exhausted, so a test does not
    have to over-provision it).

    `fail_at` (1-based) makes the Nth set_pin/unbind/bind call raise
    `fail_exc` -- read_sda never raises, matching the real Hardware, which
    catches its own subprocess errors and returns "unknown" instead.
    """

    def __init__(self, sda_sequence=None, fail_at=None, fail_exc=None):
        self.calls: list[tuple] = []
        self._sda_sequence = list(sda_sequence) if sda_sequence else ["hi"]
        self._sda_index = 0
        self._fail_at = fail_at
        self._fail_exc = fail_exc or RuntimeError("fake hardware failure")
        self._call_count = 0

    def _record(self, call: tuple) -> None:
        self._call_count += 1
        self.calls.append(call)
        if self._fail_at is not None and self._call_count == self._fail_at:
            raise self._fail_exc

    def read_sda(self) -> str:
        idx = min(self._sda_index, len(self._sda_sequence) - 1)
        level = self._sda_sequence[idx]
        self._sda_index += 1
        self.calls.append(("read_sda",))
        return level

    def set_pin(self, pin, *mode) -> None:
        self._record(("set_pin", pin, mode))

    def unbind(self) -> None:
        self._record(("unbind",))

    def bind(self) -> None:
        self._record(("bind",))


# ---------------------------------------------------------------------------
# pinctrl output parsing
# ---------------------------------------------------------------------------

class ParsePinctrlLevelTests(unittest.TestCase):
    def test_high_line(self):
        line = " 2: a3    pu | hi // GPIO2 = SDA1\n"
        self.assertEqual(guard.parse_pinctrl_level(line), "hi")

    def test_low_line(self):
        line = " 2: a3    pu | lo // GPIO2 = SDA1\n"
        self.assertEqual(guard.parse_pinctrl_level(line), "lo")

    def test_garbage_output_is_unknown(self):
        self.assertEqual(guard.parse_pinctrl_level("no such pin\n"), "unknown")

    def test_empty_output_is_unknown(self):
        self.assertEqual(guard.parse_pinctrl_level(""), "unknown")


# ---------------------------------------------------------------------------
# The real Hardware class: subprocess/sysfs plumbing only. Everything else
# in this file uses FakeHardware instead.
# ---------------------------------------------------------------------------

class HardwarePlumbingTests(unittest.TestCase):
    def test_read_sda_parses_subprocess_stdout(self):
        hw = guard.Hardware()
        fake_result = mock.Mock(stdout=" 2: a3    pu | hi // GPIO2 = SDA1\n")
        with mock.patch("subprocess.run", return_value=fake_result) as run:
            level = hw.read_sda()
        self.assertEqual(level, "hi")
        run.assert_called_once_with(
            ["pinctrl", "get", "2"], capture_output=True, text=True, timeout=5, check=True,
        )

    def test_read_sda_subprocess_failure_is_unknown_not_raise(self):
        hw = guard.Hardware()
        with mock.patch("subprocess.run", side_effect=FileNotFoundError("no pinctrl")):
            level = hw.read_sda()
        self.assertEqual(level, "unknown")

    def test_set_pin_calls_pinctrl_set(self):
        hw = guard.Hardware()
        with mock.patch("subprocess.run") as run:
            hw.set_pin(2, "ip", "pu")
        run.assert_called_once_with(["pinctrl", "set", "2", "ip", "pu"], timeout=5, check=True)

    def test_unbind_writes_device_name_to_sysfs(self):
        hw = guard.Hardware()
        m = mock.mock_open()
        with mock.patch("builtins.open", m):
            hw.unbind()
        m.assert_called_once_with(guard.UNBIND_PATH, "w")
        m().write.assert_called_once_with(guard.DEVICE_NAME)

    def test_bind_writes_device_name_to_sysfs(self):
        hw = guard.Hardware()
        m = mock.mock_open()
        with mock.patch("builtins.open", m):
            hw.bind()
        m.assert_called_once_with(guard.BIND_PATH, "w")

    def test_dry_run_set_pin_does_not_touch_subprocess(self):
        hw = guard.Hardware(dry_run=True)
        with mock.patch("subprocess.run") as run:
            hw.set_pin(3, "op", "dh")
        run.assert_not_called()

    def test_dry_run_unbind_and_bind_do_not_touch_sysfs(self):
        hw = guard.Hardware(dry_run=True)
        with mock.patch("builtins.open") as m_open:
            hw.unbind()
            hw.bind()
        m_open.assert_not_called()

    def test_dry_run_read_sda_is_still_real(self):
        # Reading is non-destructive, so dry-run must not fake it -- the
        # whole point of --dry-run is watching real detection happen.
        hw = guard.Hardware(dry_run=True)
        fake_result = mock.Mock(stdout=" 2: a3    pu | lo // GPIO2 = SDA1\n")
        with mock.patch("subprocess.run", return_value=fake_result) as run:
            level = hw.read_sda()
        self.assertEqual(level, "lo")
        run.assert_called_once()


# ---------------------------------------------------------------------------
# UPS status-file age
# ---------------------------------------------------------------------------

class ReadUpsAgeTests(unittest.TestCase):
    def test_missing_file_is_none(self):
        with tempfile.TemporaryDirectory() as d:
            age = guard.read_ups_age_s(os.path.join(d, "nope.json"), now=1000.0)
        self.assertIsNone(age)

    def test_malformed_json_is_none(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ups.json")
            with open(path, "w") as f:
                f.write("{not valid json")
            age = guard.read_ups_age_s(path, now=1000.0)
        self.assertIsNone(age)

    def test_missing_ts_key_is_none(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ups.json")
            with open(path, "w") as f:
                json.dump({"ac_present": True}, f)
            age = guard.read_ups_age_s(path, now=1000.0)
        self.assertIsNone(age)

    def test_valid_ts_returns_age(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ups.json")
            with open(path, "w") as f:
                json.dump({"ts": 940.0}, f)
            age = guard.read_ups_age_s(path, now=1000.0)
        self.assertEqual(age, 60.0)


# ---------------------------------------------------------------------------
# SDA sampling: N consecutive lows, a high in between resets.
# ---------------------------------------------------------------------------

class SdaSamplingTests(unittest.TestCase):
    def test_all_low_is_a_full_streak(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
        samples = guard.sample_sda_levels(hw, 3, sleep_fn=_noop_sleep)
        self.assertEqual(samples, ["lo", "lo", "lo"])
        self.assertTrue(guard.is_low_streak(samples, 3))

    def test_high_in_the_middle_breaks_the_streak(self):
        hw = FakeHardware(sda_sequence=["lo", "hi", "lo"])
        samples = guard.sample_sda_levels(hw, 3, sleep_fn=_noop_sleep)
        # Stops the moment the high is seen -- the third "lo" is never read.
        self.assertEqual(samples, ["lo", "hi"])
        self.assertFalse(guard.is_low_streak(samples, 3))

    def test_short_sample_list_is_not_a_streak(self):
        self.assertFalse(guard.is_low_streak(["lo", "lo"], 3))


# ---------------------------------------------------------------------------
# Stuck detection: both conditions required.
# ---------------------------------------------------------------------------

class DetectStuckTests(unittest.TestCase):
    def test_stale_only_is_not_stuck(self):
        hw = FakeHardware(sda_sequence=["hi"])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, low_samples=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_low_only_is_not_stuck(self):
        # UPS is fresh -- SDA sampling is skipped entirely (short-circuit),
        # so only one read happens regardless of what it would have shown.
        hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=5.0, stale_s=30.0, low_samples=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)
        self.assertEqual(len(hw.calls), 1)

    def test_missing_ups_status_is_not_stuck(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=None, stale_s=30.0, low_samples=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_stale_and_low_is_stuck(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, low_samples=3, sleep_fn=_noop_sleep
        )
        self.assertTrue(stuck)
        self.assertEqual(samples, ["lo", "lo", "lo"])

    def test_unknown_sda_reading_is_not_stuck(self):
        # A pinctrl parse failure must never be mistaken for a stuck-low bus.
        hw = FakeHardware(sda_sequence=["unknown", "unknown", "unknown"])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, low_samples=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)


# ---------------------------------------------------------------------------
# Recovery sequence.
# ---------------------------------------------------------------------------

class RecoverBusTests(unittest.TestCase):
    def test_unbind_is_the_first_call(self):
        hw = FakeHardware(sda_sequence=["hi"])
        guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertEqual(hw.calls[0], ("unbind",))

    def test_pins_restored_and_bound_even_when_a_pulse_step_raises(self):
        hw = FakeHardware(
            sda_sequence=["lo"] * 5, fail_at=4, fail_exc=RuntimeError("pulse failed"),
        )
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)
        self.assertEqual(error, "pulse failed")
        self.assertIn(("set_pin", guard.SCL_PIN, ("a3", "pu")), hw.calls)
        self.assertIn(("set_pin", guard.SDA_PIN, ("a3", "pu")), hw.calls)
        self.assertEqual(hw.calls[-1], ("bind",))

    def test_unbind_itself_raising_still_restores_and_binds(self):
        hw = FakeHardware(sda_sequence=["lo"], fail_at=1, fail_exc=OSError("no such device"))
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)
        self.assertEqual(error, "no such device")
        self.assertIn(("set_pin", guard.SCL_PIN, ("a3", "pu")), hw.calls)
        self.assertIn(("set_pin", guard.SDA_PIN, ("a3", "pu")), hw.calls)
        self.assertEqual(hw.calls[-1], ("bind",))

    def test_pulses_stop_early_once_sda_reads_high(self):
        # Two low checks, then high on the third pulse; the fourth reading
        # is the post-rebind confirmation.
        hw = FakeHardware(sda_sequence=["lo", "lo", "hi", "hi"])
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)
        self.assertIsNone(error)
        pulses = [c for c in hw.calls if c == ("set_pin", guard.SCL_PIN, ("op", "dl"))]
        self.assertEqual(len(pulses), 3)

    def test_sixteen_pulse_cap(self):
        hw = FakeHardware(sda_sequence=["lo"] * 20)  # SDA never frees
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)
        self.assertEqual(error, "SDA still lo after recovery")
        pulses = [c for c in hw.calls if c == ("set_pin", guard.SCL_PIN, ("op", "dl"))]
        self.assertEqual(len(pulses), guard.MAX_PULSES)
        self.assertEqual(len(pulses), 16)

    def test_stop_condition_emitted_after_the_pulse_loop(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "hi", "hi"])
        guard.recover_bus(hw, sleep_fn=_noop_sleep)
        idx = hw.calls.index(("set_pin", guard.SDA_PIN, ("op", "dl")))
        self.assertEqual(
            hw.calls[idx : idx + 3],
            [
                ("set_pin", guard.SDA_PIN, ("op", "dl")),
                ("set_pin", guard.SCL_PIN, ("op", "dh")),
                ("set_pin", guard.SDA_PIN, ("op", "dh")),
            ],
        )

    def test_success_requires_final_sda_high_after_rebind(self):
        # Loop never sees high (so it runs the full cap), but by pure luck
        # the post-rebind reading is high -- recovery is still reported ok.
        hw = FakeHardware(sda_sequence=["lo"] * 16 + ["hi"])
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)
        self.assertIsNone(error)


# ---------------------------------------------------------------------------
# Recovery rate limiting.
# ---------------------------------------------------------------------------

class RecoveryRateLimiterTests(unittest.TestCase):
    def test_allows_up_to_the_cap_then_blocks(self):
        limiter = guard.RecoveryRateLimiter(max_per_hour=3)
        now = 1000.0
        for _ in range(3):
            self.assertTrue(limiter.allow(now))
            limiter.record(now)
            now += 1.0
        self.assertFalse(limiter.allow(now))
        self.assertEqual(limiter.count(now), 3)

    def test_window_frees_after_it_elapses(self):
        limiter = guard.RecoveryRateLimiter(max_per_hour=1, window_s=3600.0)
        limiter.record(0.0)
        self.assertFalse(limiter.allow(100.0))
        self.assertTrue(limiter.allow(3601.0))
        self.assertEqual(limiter.count(3601.0), 0)


# ---------------------------------------------------------------------------
# Status file.
# ---------------------------------------------------------------------------

class WriteStatusFileTests(unittest.TestCase):
    def test_writes_expected_json_with_no_leftover_temp_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "i2c_guard_status.json")
            status = {
                "ts": 123.0, "sda_level": "hi", "ups_age_s": 1.5, "stuck": False,
                "recoveries_total": 0, "last_recovery_ts": None,
                "last_result": None, "last_error": None,
            }
            guard.write_status_file(path, status)
            with open(path) as f:
                data = json.load(f)
            self.assertEqual(data, status)
            leftovers = [n for n in os.listdir(d) if n != "i2c_guard_status.json"]
        self.assertEqual(leftovers, [])

    def test_unwritable_path_is_swallowed_not_raised(self):
        bad = "/nonexistent_dir_xyz_i2c_guard/status.json"
        self.assertFalse(os.path.isdir(os.path.dirname(bad)))
        guard.write_status_file(bad, {"ts": 1.0})  # must not raise


# ---------------------------------------------------------------------------
# I2cBusGuard: ties detection, recovery, rate limiting and status together.
# ---------------------------------------------------------------------------

class GuardCycleTests(unittest.TestCase):
    @staticmethod
    def _cfg(status_file, ups_status_file, **overrides):
        kwargs = dict(
            interval_s=5.0, stale_s=30.0, low_samples=3, max_per_hour=6,
            status_file=status_file, ups_status_file=ups_status_file, dry_run=False,
        )
        kwargs.update(overrides)
        return guard.GuardConfig(**kwargs)

    def test_not_stuck_cycle_never_calls_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "missing_ups.json")  # never created -> age unknown
            hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(hw, cfg, sleep_fn=_noop_sleep)
            result = g.run_cycle()
            with open(status_path) as f:
                on_disk = json.load(f)

        self.assertFalse(result["stuck"])
        self.assertEqual(g.recoveries_total, 0)
        self.assertNotIn("unbind", [c[0] for c in hw.calls])
        self.assertEqual(on_disk, result)

    def test_stuck_cycle_runs_one_recovery_and_publishes_status(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            # 3 detect samples (all low) + recovery: one loop check that
            # already reads high (breaks immediately) + the final post-
            # rebind confirmation, also high.
            hw = FakeHardware(sda_sequence=["lo", "lo", "lo", "hi", "hi"])
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            result = g.run_cycle()
            with open(status_path) as f:
                on_disk = json.load(f)

        self.assertTrue(result["stuck"])
        self.assertEqual(g.recoveries_total, 1)
        self.assertEqual(g.last_result, "ok")
        self.assertEqual(result["last_result"], "ok")
        self.assertEqual(result["recoveries_total"], 1)
        self.assertEqual(on_disk, result)

    def test_rate_limited_stuck_logs_error_once_and_stops_attempting(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            # Always low -- every recovery attempt runs the full pulse cap
            # and still fails, and every cycle's detect sampling sees low too.
            hw = FakeHardware(sda_sequence=["lo"] * 200)
            cfg = self._cfg(status_path, ups_path, max_per_hour=1)
            clock = {"m": 0.0}
            g = guard.I2cBusGuard(
                hw, cfg,
                now_fn=lambda: 1000.0 + clock["m"],
                monotonic_fn=lambda: clock["m"],
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="ERROR") as captured:
                g.run_cycle()  # stuck -> recovery attempted, uses up the cap
                clock["m"] += 5.0
                g.run_cycle()  # still stuck, rate-limited -> ERROR logged once
                clock["m"] += 5.0
                g.run_cycle()  # still rate-limited -> must not log again

        rate_limit_lines = [m for m in captured.output if "already ran in the last hour" in m]
        self.assertEqual(len(rate_limit_lines), 1)
        self.assertEqual(g.recoveries_total, 1)

    def test_rate_limit_warning_fires_again_after_window_frees(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            hw = FakeHardware(sda_sequence=["lo"] * 400)
            cfg = self._cfg(status_path, ups_path, max_per_hour=1)
            clock = {"m": 0.0}
            g = guard.I2cBusGuard(
                hw, cfg,
                now_fn=lambda: 100000.0 + clock["m"],
                monotonic_fn=lambda: clock["m"],
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="ERROR") as captured:
                g.run_cycle()                          # uses the cap
                clock["m"] += 5.0
                g.run_cycle()                           # rate-limited -> ERROR #1
                clock["m"] += guard.RATE_LIMIT_WINDOW_S  # window fully elapses
                g.run_cycle()                           # cap free again -> new attempt
                clock["m"] += 5.0
                g.run_cycle()                           # rate-limited again -> ERROR #2

        rate_limit_lines = [m for m in captured.output if "already ran in the last hour" in m]
        self.assertEqual(len(rate_limit_lines), 2)
        self.assertEqual(g.recoveries_total, 2)


# ---------------------------------------------------------------------------
# CLI wiring.
# ---------------------------------------------------------------------------

class ArgParserDefaultsTests(unittest.TestCase):
    def test_defaults_match_the_spec(self):
        args = guard.build_arg_parser().parse_args([])
        self.assertEqual(args.interval, 5.0)
        self.assertEqual(args.stale_s, 30.0)
        self.assertEqual(args.low_samples, 3)
        self.assertEqual(args.max_per_hour, 6)
        self.assertEqual(args.status_file, "/tmp/i2c_guard_status.json")
        self.assertFalse(args.dry_run)
        self.assertFalse(args.once)


class MainSmokeTests(unittest.TestCase):
    def test_once_dry_run_writes_a_status_file(self):
        # No pinctrl on a dev machine: Hardware.read_sda() swallows the
        # subprocess failure and reports "unknown", so this never looks
        # stuck -- a real end-to-end smoke test of the CLI wiring that
        # cannot touch real hardware.
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            missing_ups = os.path.join(d, "no_such_ups.json")
            rc = guard.main([
                "--once", "--dry-run",
                "--status-file", status_path,
                "--ups-status-file", missing_ups,
            ])
            with open(status_path) as f:
                data = json.load(f)

        self.assertEqual(rc, 0)
        self.assertIn(data["sda_level"], ("hi", "lo", "unknown"))
        self.assertFalse(data["stuck"])
        self.assertIsNone(data["ups_age_s"])


if __name__ == "__main__":
    unittest.main()
