"""Tests for bin/i2c_bus_guard.py, the I2C bus 1 auto-unlock guard.

bin/ is not a package, so the script is loaded via importlib straight from
its file path (no sys.path changes needed -- the script is stdlib-only, so
unlike scripts/upsPlus_power_daemon.py there is nothing to stub before
import).

All hardware access in the guard goes through one Hardware class
(read_sda / read_pins / is_bound / set_pin / unbind / bind /
restart_ups_service). Every test below except the narrow "real Hardware
plumbing" group replaces that class entirely with FakeHardware, which
records every call in order and plays back scripted SDA/joint SDA+SCL
readings -- so detection, recovery, and self-heal are exercised as pure
logic, with no subprocess, sysfs, or real signal ever involved.
"""

import importlib.util
import json
import os
import signal
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


def _count_pulses(calls: list[tuple]) -> int:
    """Count pulse-loop iterations from a FakeHardware call log.

    A loop pulse is the consecutive pair (SCL driven low, SCL released).
    The STOP sequence's own SCL-low step is NOT a pulse -- it is followed
    by driving SDA low, never by releasing SCL -- so this pair pattern
    counts only real pulses even though the STOP sequence starts with the
    exact same (SCL, "op", "dl") call a pulse does.
    """
    low = ("set_pin", guard.SCL_PIN, ("op", "dl"))
    release = ("set_pin", guard.SCL_PIN, ("ip", "pu"))
    return sum(
        1 for i in range(len(calls) - 1) if calls[i] == low and calls[i + 1] == release
    )


class FakeHardware:
    """Records every call in order.

    read_sda() plays back `sda_sequence` (single-pin level, used by
    recover_bus's pulse loop and check_recovery_success). read_pins()
    plays back `joint_sequence` (a list of (sda_level, scl_level) tuples,
    used by detect_stuck's joint sampling and by self-heal's health
    check) for LEVEL, and reports the current `pin_functions` for
    FUNCTION -- kept as real, mutable state that set_pin() updates, so a
    test can drive self_heal() and then observe read_pins() reflect what
    it did. Both sequences repeat their last entry once exhausted, so a
    test does not have to over-provision either one.

    `bound` starts True (a healthy bus) and is flipped by unbind()/bind()
    -- also real, mutable state -- so self_heal()'s "did this transition
    unbound -> bound" contract is exercised faithfully.

    `fail_at` (1-based) makes the Nth set_pin/unbind/bind call raise
    `fail_exc` -- read_sda/read_pins/is_bound never raise, matching the
    real Hardware, which catches its own subprocess errors and returns
    "unknown" instead.
    """

    def __init__(
        self,
        sda_sequence=None,
        joint_sequence=None,
        bound=True,
        pin_functions=None,
        fail_at=None,
        fail_exc=None,
    ):
        self.calls: list[tuple] = []
        self._sda_sequence = list(sda_sequence) if sda_sequence else ["hi"]
        self._sda_index = 0
        self._joint_sequence = list(joint_sequence) if joint_sequence else [("hi", "hi")]
        self._joint_index = 0
        self.bound = bound
        self.pin_functions = (
            dict(pin_functions) if pin_functions
            else {guard.SDA_PIN: "a3", guard.SCL_PIN: "a3"}
        )
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

    def read_pins(self):
        idx = min(self._joint_index, len(self._joint_sequence) - 1)
        sda_level, scl_level = self._joint_sequence[idx]
        self._joint_index += 1
        self.calls.append(("read_pins",))
        return {
            guard.SDA_PIN: guard.PinReading(self.pin_functions[guard.SDA_PIN], sda_level),
            guard.SCL_PIN: guard.PinReading(self.pin_functions[guard.SCL_PIN], scl_level),
        }

    def is_bound(self) -> bool:
        self.calls.append(("is_bound",))
        return self.bound

    def set_pin(self, pin, *mode) -> None:
        self._record(("set_pin", pin, mode))
        if mode:
            self.pin_functions[pin] = mode[0]

    def unbind(self) -> None:
        self._record(("unbind",))
        self.bound = False

    def bind(self) -> None:
        self._record(("bind",))
        self.bound = True

    def restart_ups_service(self) -> None:
        # Mirrors the real Hardware's contract: best-effort, never raises.
        self.calls.append(("restart_ups_service",))


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


class ParsePinctrlPinsTests(unittest.TestCase):
    def test_both_pins_healthy(self):
        output = (
            " 2: a3    pu | hi // GPIO2 = SDA1\n"
            " 3: a3    pu | hi // GPIO3 = SCL1\n"
        )
        pins = guard.parse_pinctrl_pins(output, (2, 3))
        self.assertEqual(pins[2], guard.PinReading("a3", "hi"))
        self.assertEqual(pins[3], guard.PinReading("a3", "hi"))

    def test_stuck_pattern_sda_low_scl_high(self):
        output = (
            " 2: a3    pu | lo // GPIO2 = SDA1\n"
            " 3: a3    pu | hi // GPIO3 = SCL1\n"
        )
        pins = guard.parse_pinctrl_pins(output, (2, 3))
        self.assertEqual(pins[2].level, "lo")
        self.assertEqual(pins[3].level, "hi")

    def test_pin_drifted_off_a3(self):
        output = (
            " 2: ip    pu | hi // GPIO2 = SDA1\n"
            " 3: a3    pu | hi // GPIO3 = SCL1\n"
        )
        pins = guard.parse_pinctrl_pins(output, (2, 3))
        self.assertEqual(pins[2].function, "ip")
        self.assertEqual(pins[3].function, "a3")

    def test_missing_pin_line_is_unknown(self):
        # Only pin 3's line present -- pin 2 must still get an entry.
        output = " 3: a3    pu | hi // GPIO3 = SCL1\n"
        pins = guard.parse_pinctrl_pins(output, (2, 3))
        self.assertEqual(pins[2], guard.PinReading("unknown", "unknown"))
        self.assertEqual(pins[3], guard.PinReading("a3", "hi"))

    def test_garbage_output_is_unknown_for_both(self):
        pins = guard.parse_pinctrl_pins("no such pin\n", (2, 3))
        self.assertEqual(pins[2], guard.PinReading("unknown", "unknown"))
        self.assertEqual(pins[3], guard.PinReading("unknown", "unknown"))


# ---------------------------------------------------------------------------
# The real Hardware class: subprocess/sysfs/filesystem plumbing only.
# Everything else in this file uses FakeHardware instead.
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

    def test_read_pins_calls_joint_pinctrl_get(self):
        hw = guard.Hardware()
        fake_result = mock.Mock(
            stdout=(
                " 2: a3    pu | lo // GPIO2 = SDA1\n"
                " 3: a3    pu | hi // GPIO3 = SCL1\n"
            )
        )
        with mock.patch("subprocess.run", return_value=fake_result) as run:
            pins = hw.read_pins()
        run.assert_called_once_with(
            ["pinctrl", "get", "2,3"], capture_output=True, text=True, timeout=5, check=True,
        )
        self.assertEqual(pins[2], guard.PinReading("a3", "lo"))
        self.assertEqual(pins[3], guard.PinReading("a3", "hi"))

    def test_read_pins_subprocess_failure_is_unknown_not_raise(self):
        hw = guard.Hardware()
        with mock.patch("subprocess.run", side_effect=FileNotFoundError("no pinctrl")):
            pins = hw.read_pins()
        self.assertEqual(pins[2], guard.PinReading("unknown", "unknown"))
        self.assertEqual(pins[3], guard.PinReading("unknown", "unknown"))

    def test_is_bound_checks_the_driver_symlink(self):
        hw = guard.Hardware()
        with mock.patch("os.path.exists", return_value=True) as exists:
            self.assertTrue(hw.is_bound())
        exists.assert_called_once_with(guard.DEVICE_LINK_PATH)
        with mock.patch("os.path.exists", return_value=False):
            self.assertFalse(hw.is_bound())

    def test_set_pin_calls_pinctrl_set(self):
        hw = guard.Hardware()
        with mock.patch("subprocess.run") as run:
            hw.set_pin(2, "ip", "pu")
        run.assert_called_once_with(["pinctrl", "set", "2", "ip", "pu"], timeout=5, check=True)

    def test_unbind_writes_device_name_to_sysfs_when_bound(self):
        hw = guard.Hardware()
        m = mock.mock_open()
        with mock.patch("os.path.exists", return_value=True):
            with mock.patch("builtins.open", m):
                hw.unbind()
        m.assert_called_once_with(guard.UNBIND_PATH, "w")
        m().write.assert_called_once_with(guard.DEVICE_NAME)

    def test_unbind_skipped_when_not_bound(self):
        # Rule A: unbind must be tolerant -- a no-op, not an error, when
        # the device is already unbound (e.g. self-heal running right
        # after a kill left it that way).
        hw = guard.Hardware()
        with mock.patch("os.path.exists", return_value=False):
            with mock.patch("subprocess.run") as run:
                with mock.patch("builtins.open") as m_open:
                    hw.unbind()  # must not raise
        run.assert_not_called()
        m_open.assert_not_called()

    def test_bind_writes_device_name_to_sysfs(self):
        hw = guard.Hardware()
        m = mock.mock_open()
        with mock.patch("builtins.open", m):
            hw.bind()
        m.assert_called_once_with(guard.BIND_PATH, "w")

    def test_dry_run_set_pin_does_not_touch_subprocess(self):
        hw = guard.Hardware(dry_run=True)
        with mock.patch("subprocess.run") as run:
            hw.set_pin(3, "op", "dl")
        run.assert_not_called()

    def test_dry_run_unbind_and_bind_do_not_touch_sysfs(self):
        hw = guard.Hardware(dry_run=True)
        with mock.patch("os.path.exists", return_value=True):
            with mock.patch("builtins.open") as m_open:
                hw.unbind()
                hw.bind()
        m_open.assert_not_called()

    def test_dry_run_reads_are_still_real(self):
        # Reading is non-destructive, so dry-run must not fake it -- the
        # whole point of --dry-run is watching real detection happen.
        hw = guard.Hardware(dry_run=True)
        fake_result = mock.Mock(stdout=" 2: a3    pu | lo // GPIO2 = SDA1\n")
        with mock.patch("subprocess.run", return_value=fake_result) as run:
            level = hw.read_sda()
        self.assertEqual(level, "lo")
        run.assert_called_once()

    def test_restart_ups_service_calls_systemctl_restart(self):
        hw = guard.Hardware()
        with mock.patch("subprocess.run") as run:
            hw.restart_ups_service()
        run.assert_called_once_with(
            ["systemctl", "restart", guard.UPS_SERVICE_NAME], timeout=15, check=True,
        )

    def test_restart_ups_service_dry_run_does_not_touch_subprocess(self):
        hw = guard.Hardware(dry_run=True)
        with mock.patch("subprocess.run") as run:
            hw.restart_ups_service()
        run.assert_not_called()

    def test_restart_ups_service_failure_is_logged_not_raised(self):
        hw = guard.Hardware()
        with mock.patch(
            "subprocess.run", side_effect=OSError("systemctl not found")
        ):
            hw.restart_ups_service()  # must not raise


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


class IsUpsStaleTests(unittest.TestCase):
    def test_unknown_age_is_not_stale(self):
        self.assertFalse(guard.is_ups_stale(None, 30.0))

    def test_age_under_threshold_is_not_stale(self):
        self.assertFalse(guard.is_ups_stale(29.9, 30.0))

    def test_age_over_threshold_is_stale(self):
        self.assertTrue(guard.is_ups_stale(30.1, 30.0))


# ---------------------------------------------------------------------------
# Joint SDA+SCL sampling: a wedged bus is SDA low AND SCL high on every
# sample; a single SCL-low (live transfer) or SDA-high (released) sample
# breaks the streak immediately.
# ---------------------------------------------------------------------------

class JointSamplingTests(unittest.TestCase):
    def test_full_streak_is_stuck_pattern(self):
        hw = FakeHardware(joint_sequence=[("lo", "hi")] * 5)
        samples = guard.sample_joint_levels(hw, 5, 0.1, sleep_fn=_noop_sleep)
        self.assertEqual(samples, [("lo", "hi")] * 5)
        self.assertTrue(guard.is_stuck_streak(samples, 5))

    def test_one_scl_low_sample_breaks_the_streak(self):
        # A live transfer toggles SCL -- even once -- and that must end
        # the burst immediately, not just fail the final verdict.
        hw = FakeHardware(joint_sequence=[("lo", "hi"), ("lo", "lo"), ("lo", "hi")])
        samples = guard.sample_joint_levels(hw, 3, 0.1, sleep_fn=_noop_sleep)
        self.assertEqual(samples, [("lo", "hi"), ("lo", "lo")])
        self.assertFalse(guard.is_stuck_streak(samples, 3))

    def test_one_sda_high_sample_breaks_the_streak(self):
        # A released bus reads SDA high -- even once -- and that must end
        # the burst immediately too.
        hw = FakeHardware(joint_sequence=[("lo", "hi"), ("hi", "hi"), ("lo", "hi")])
        samples = guard.sample_joint_levels(hw, 3, 0.1, sleep_fn=_noop_sleep)
        self.assertEqual(samples, [("lo", "hi"), ("hi", "hi")])
        self.assertFalse(guard.is_stuck_streak(samples, 3))

    def test_short_sample_list_is_not_a_streak(self):
        self.assertFalse(guard.is_stuck_streak([("lo", "hi"), ("lo", "hi")], 3))


# ---------------------------------------------------------------------------
# Stuck detection: both conditions required (UPS stale AND the joint
# streak).
# ---------------------------------------------------------------------------

class DetectStuckTests(unittest.TestCase):
    def test_stale_only_is_not_stuck(self):
        hw = FakeHardware(joint_sequence=[("hi", "hi")])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_low_only_is_not_stuck(self):
        # UPS is fresh -- joint sampling is skipped entirely (short-
        # circuit), so only one read_pins() happens regardless of what it
        # would have shown.
        hw = FakeHardware(joint_sequence=[("lo", "hi")] * 5)
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=5.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)
        self.assertEqual(len(hw.calls), 1)

    def test_missing_ups_status_is_not_stuck(self):
        hw = FakeHardware(joint_sequence=[("lo", "hi")] * 5)
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=None, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_stale_and_joint_streak_is_stuck(self):
        hw = FakeHardware(joint_sequence=[("lo", "hi")] * 3)
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertTrue(stuck)
        self.assertEqual(samples, [("lo", "hi")] * 3)

    def test_one_scl_high_sample_breaks_stuck_detection(self):
        hw = FakeHardware(joint_sequence=[("lo", "hi"), ("lo", "lo"), ("lo", "hi")])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_one_sda_high_sample_breaks_stuck_detection(self):
        hw = FakeHardware(joint_sequence=[("lo", "hi"), ("hi", "hi"), ("lo", "hi")])
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)

    def test_unknown_reading_is_not_stuck(self):
        # A pinctrl parse failure must never be mistaken for a stuck-low bus.
        hw = FakeHardware(joint_sequence=[("unknown", "unknown")] * 3)
        stuck, samples = guard.detect_stuck(
            hw, ups_age_s=60.0, stale_s=30.0, sample_count=3, sleep_fn=_noop_sleep
        )
        self.assertFalse(stuck)


# ---------------------------------------------------------------------------
# Recovery sequence (open-drain pin driving is unchanged from the prior
# round; the success check itself now lives in CheckRecoverySuccessTests).
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
        # Two low checks, then high on the third pulse; the next 3
        # readings are the post-rebind success check (2-of-3 also high).
        hw = FakeHardware(sda_sequence=["lo", "lo", "hi", "hi", "hi", "hi"])
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)
        self.assertIsNone(error)
        self.assertEqual(_count_pulses(hw.calls), 3)

    def test_sixteen_pulse_cap(self):
        hw = FakeHardware(sda_sequence=["lo"] * 20)  # SDA never frees
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)
        self.assertIn("SDA samples after recovery", error)
        self.assertEqual(_count_pulses(hw.calls), guard.MAX_PULSES)
        self.assertEqual(_count_pulses(hw.calls), 16)

    def test_stop_sequence_is_scl_low_sda_low_release_scl_release_sda(self):
        # Textbook STOP, open-drain style: with SCL already released (high)
        # at the end of the loop, drive SCL low first (so driving SDA low
        # next cannot look like a START), then SDA low, then release SCL,
        # then release SDA -- SDA goes low-to-high while SCL is high.
        hw = FakeHardware(sda_sequence=["lo", "lo", "hi", "hi", "hi", "hi"])
        guard.recover_bus(hw, sleep_fn=_noop_sleep)

        scl_low = ("set_pin", guard.SCL_PIN, ("op", "dl"))
        sda_low = ("set_pin", guard.SDA_PIN, ("op", "dl"))
        idx = None
        for i in range(len(hw.calls) - 1):
            if hw.calls[i] == scl_low and hw.calls[i + 1] == sda_low:
                idx = i
                break
        self.assertIsNotNone(idx, "STOP's SCL-low -> SDA-low pair not found")
        self.assertEqual(
            hw.calls[idx : idx + 4],
            [
                ("set_pin", guard.SCL_PIN, ("op", "dl")),
                ("set_pin", guard.SDA_PIN, ("op", "dl")),
                ("set_pin", guard.SCL_PIN, ("ip", "pu")),
                ("set_pin", guard.SDA_PIN, ("ip", "pu")),
            ],
        )

    def test_success_requires_two_of_three_sda_high_after_rebind(self):
        # Loop never sees high (runs the full cap), but the post-rebind
        # success check gets 2 of 3 high -- recovery is still reported ok.
        hw = FakeHardware(sda_sequence=["lo"] * 16 + ["lo", "hi", "hi"])
        ok, error = guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)
        self.assertIsNone(error)


# ---------------------------------------------------------------------------
# Recovery success check (rule D): 3 SDA samples 200ms apart, success if
# at least 2 read high -- one sample can land inside a live transfer.
# ---------------------------------------------------------------------------

class CheckRecoverySuccessTests(unittest.TestCase):
    def test_three_of_three_high_is_success(self):
        hw = FakeHardware(sda_sequence=["hi", "hi", "hi"])
        ok, levels = guard.check_recovery_success(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)
        self.assertEqual(levels, ["hi", "hi", "hi"])

    def test_two_of_three_high_is_success(self):
        hw = FakeHardware(sda_sequence=["lo", "hi", "hi"])
        ok, levels = guard.check_recovery_success(hw, sleep_fn=_noop_sleep)
        self.assertTrue(ok)

    def test_one_of_three_high_is_failure(self):
        hw = FakeHardware(sda_sequence=["hi", "lo", "lo"])
        ok, levels = guard.check_recovery_success(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)

    def test_zero_of_three_high_is_failure(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "lo"])
        ok, levels = guard.check_recovery_success(hw, sleep_fn=_noop_sleep)
        self.assertFalse(ok)

    def test_takes_exactly_three_samples(self):
        hw = FakeHardware(sda_sequence=["hi"] * 10)
        guard.check_recovery_success(hw, sleep_fn=_noop_sleep)
        reads = [c for c in hw.calls if c == ("read_sda",)]
        self.assertEqual(len(reads), guard.SUCCESS_CHECK_SAMPLES)
        self.assertEqual(len(reads), 3)

    def test_sleeps_between_samples_not_before_the_first(self):
        hw = FakeHardware(sda_sequence=["hi", "hi", "hi"])
        sleeps = []
        guard.check_recovery_success(hw, sleep_fn=sleeps.append)
        self.assertEqual(sleeps, [guard.SUCCESS_CHECK_INTERVAL_S] * 2)


# ---------------------------------------------------------------------------
# Open-drain emulation. I2C lines must never be driven high push-pull: if a
# wedged slave is still holding a line low, a push-pull high output fights
# it on the same wire. Every drive is "op","dl" (low); every release is
# "ip","pu" (input + pull-up, which reads back high); "op","dh" must never
# appear anywhere in the recovery sequence.
# ---------------------------------------------------------------------------

class OpenDrainEmulationTests(unittest.TestCase):
    def test_no_pin_is_ever_driven_high_push_pull(self):
        for sda_sequence in (
            ["lo", "lo", "hi", "hi", "hi", "hi"],  # stops early, partway through
            ["lo"] * 20,                            # runs the full 16-pulse cap
        ):
            hw = FakeHardware(sda_sequence=sda_sequence)
            guard.recover_bus(hw, sleep_fn=_noop_sleep)
            offenders = [
                c for c in hw.calls if c[0] == "set_pin" and c[2] == ("op", "dh")
            ]
            self.assertEqual(offenders, [], f"push-pull high drive found: {offenders}")

    def test_initial_setup_releases_both_lines_instead_of_driving_scl_high(self):
        hw = FakeHardware(sda_sequence=["lo", "lo", "hi", "hi", "hi", "hi"])
        guard.recover_bus(hw, sleep_fn=_noop_sleep)
        self.assertEqual(hw.calls[0], ("unbind",))
        self.assertEqual(hw.calls[1], ("set_pin", guard.SCL_PIN, ("ip", "pu")))
        self.assertEqual(hw.calls[2], ("set_pin", guard.SDA_PIN, ("ip", "pu")))

    def test_pulse_high_half_is_a_release_not_a_drive(self):
        # Every SCL set_pin call is one of: drive low, release, or the
        # final restore to the I2C alt function -- never a push-pull high.
        hw = FakeHardware(sda_sequence=["lo"] * 20)
        guard.recover_bus(hw, sleep_fn=_noop_sleep)
        scl_calls = [c for c in hw.calls if c[0] == "set_pin" and c[1] == guard.SCL_PIN]
        allowed = {("op", "dl"), ("ip", "pu"), ("a3", "pu")}
        for call in scl_calls:
            self.assertIn(call[2], allowed)


# ---------------------------------------------------------------------------
# Self-heal (rule A2): repairs a half-finished recovery left by a kill
# that skipped recover_bus's `finally` block.
# ---------------------------------------------------------------------------

class SelfHealTests(unittest.TestCase):
    def test_bound_with_both_pins_a3_is_healthy(self):
        hw = FakeHardware(bound=True, pin_functions={guard.SDA_PIN: "a3", guard.SCL_PIN: "a3"})
        self.assertTrue(guard.is_driver_healthy(hw))

    def test_unbound_is_unhealthy_regardless_of_pins(self):
        hw = FakeHardware(bound=False, pin_functions={guard.SDA_PIN: "a3", guard.SCL_PIN: "a3"})
        self.assertFalse(guard.is_driver_healthy(hw))

    def test_bound_but_sda_off_a3_is_unhealthy(self):
        hw = FakeHardware(bound=True, pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "a3"})
        self.assertFalse(guard.is_driver_healthy(hw))

    def test_bound_but_scl_off_a3_is_unhealthy(self):
        hw = FakeHardware(bound=True, pin_functions={guard.SDA_PIN: "a3", guard.SCL_PIN: "op"})
        self.assertFalse(guard.is_driver_healthy(hw))

    def test_unbound_state_is_repaired_and_reports_rebound(self):
        hw = FakeHardware(bound=False, pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "op"})
        rebound = guard.self_heal(hw)
        self.assertTrue(rebound)
        self.assertIn(("set_pin", guard.SCL_PIN, ("a3", "pu")), hw.calls)
        self.assertIn(("set_pin", guard.SDA_PIN, ("a3", "pu")), hw.calls)
        self.assertIn(("bind",), hw.calls)
        self.assertTrue(hw.is_bound())

    def test_pin_only_drift_is_repaired_but_not_reported_as_rebound(self):
        # The driver was never unbound -- only a pin drifted off a3 for
        # some unrelated reason -- so this must NOT unbind or claim a
        # rebind (rule B: no UPS restart is warranted here).
        hw = FakeHardware(bound=True, pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "a3"})
        rebound = guard.self_heal(hw)
        self.assertFalse(rebound)
        self.assertIn(("set_pin", guard.SDA_PIN, ("a3", "pu")), hw.calls)
        self.assertIn(("set_pin", guard.SCL_PIN, ("a3", "pu")), hw.calls)
        self.assertNotIn("unbind", [c[0] for c in hw.calls])
        self.assertNotIn("bind", [c[0] for c in hw.calls])

    def test_set_pin_failure_is_caught_and_logged_not_raised(self):
        hw = FakeHardware(
            bound=False, fail_at=1, fail_exc=RuntimeError("pinctrl set failed"),
        )
        with self.assertLogs(level="ERROR"):
            rebound = guard.self_heal(hw)  # must not raise
        self.assertFalse(rebound)


# ---------------------------------------------------------------------------
# Rate limiting (shared by bus recoveries and the UPS safety net).
# ---------------------------------------------------------------------------

class RateLimiterTests(unittest.TestCase):
    def test_allows_up_to_the_cap_then_blocks(self):
        limiter = guard.RateLimiter(max_count=3, window_s=3600.0)
        now = 1000.0
        for _ in range(3):
            self.assertTrue(limiter.allow(now))
            limiter.record(now)
            now += 1.0
        self.assertFalse(limiter.allow(now))
        self.assertEqual(limiter.count(now), 3)

    def test_window_frees_after_it_elapses(self):
        limiter = guard.RateLimiter(max_count=1, window_s=3600.0)
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
                "ts": 123.0, "sda_level": "hi", "scl_level": "hi", "ups_age_s": 1.5,
                "ups_stale": False, "stuck": False, "recoveries_total": 0,
                "last_recovery_ts": None, "last_result": None, "last_error": None,
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
# Kill safety (rule A1): a SIGTERM/SIGINT handler that only sets a flag,
# and a main loop that exits cleanly after the current cycle rather than
# being torn down mid-recovery.
# ---------------------------------------------------------------------------

class ShutdownFlagTests(unittest.TestCase):
    def test_handle_sets_requested(self):
        flag = guard.ShutdownFlag()
        self.assertFalse(flag.requested)
        flag.handle(signal.SIGTERM, None)
        self.assertTrue(flag.requested)

    def test_starts_false(self):
        self.assertFalse(guard.ShutdownFlag().requested)


class RunLoopTests(unittest.TestCase):
    """Drives _run_loop -- the piece of main() that would otherwise need a
    real signal to exercise -- against a minimal fake guard and a fake
    clock, with no real signal ever sent."""

    class _CountingGuard:
        def __init__(self):
            self.cycles = 0

        def run_cycle(self):
            self.cycles += 1
            return {}

    def test_once_runs_exactly_one_cycle_regardless_of_flag(self):
        g = self._CountingGuard()
        shutdown = guard.ShutdownFlag()
        guard._run_loop(g, once=True, shutdown=shutdown, interval_s=5.0, sleep_fn=_noop_sleep)
        self.assertEqual(g.cycles, 1)

    def test_exits_after_the_current_cycle_once_flag_set_during_sleep(self):
        g = self._CountingGuard()
        shutdown = guard.ShutdownFlag()

        def fake_sleep(_seconds):
            # Simulates a SIGTERM arriving during the inter-cycle sleep --
            # the handler would have set this same flag.
            shutdown.requested = True

        guard._run_loop(g, once=False, shutdown=shutdown, interval_s=5.0, sleep_fn=fake_sleep)
        # Ran exactly one cycle, then noticed the flag and stopped -- it
        # never started a second cycle.
        self.assertEqual(g.cycles, 1)

    def test_runs_multiple_cycles_until_flag_set(self):
        g = self._CountingGuard()
        shutdown = guard.ShutdownFlag()
        sleeps = {"n": 0}

        def fake_sleep(_seconds):
            sleeps["n"] += 1
            if sleeps["n"] >= 3:
                shutdown.requested = True

        guard._run_loop(g, once=False, shutdown=shutdown, interval_s=5.0, sleep_fn=fake_sleep)
        # 3 cycles, each followed by a sleep; the flag flips during the
        # 3rd sleep and is checked immediately after it, so the loop
        # stops there without attempting a 4th cycle.
        self.assertEqual(g.cycles, 3)

    def test_a_cycle_that_raises_does_not_stop_the_loop(self):
        calls = {"n": 0}

        class _FlakyGuard:
            def run_cycle(self):
                calls["n"] += 1
                if calls["n"] == 1:
                    raise RuntimeError("boom")
                return {}

        shutdown = guard.ShutdownFlag()
        sleeps = {"n": 0}

        def fake_sleep(_seconds):
            # Set the flag only after the SECOND cycle's sleep, so cycle 1
            # (which raises) is not the last one the loop attempts -- if
            # the exception had aborted the loop, cycle 2 would never run.
            sleeps["n"] += 1
            if sleeps["n"] >= 2:
                shutdown.requested = True

        with self.assertLogs(level="ERROR"):
            guard._run_loop(
                _FlakyGuard(), once=False, shutdown=shutdown, interval_s=5.0,
                sleep_fn=fake_sleep,
            )
        self.assertEqual(calls["n"], 2)  # the exception on cycle 1 did not abort the loop


# ---------------------------------------------------------------------------
# I2cBusGuard: ties self-heal, detection, recovery, rate limiting, the UPS
# safety net, and status publishing together.
# ---------------------------------------------------------------------------

class GuardCycleTests(unittest.TestCase):
    @staticmethod
    def _cfg(status_file, ups_status_file, **overrides):
        kwargs = dict(
            interval_s=5.0, stale_s=30.0, sample_count=10, sample_interval_s=0.1,
            max_per_hour=6, status_file=status_file, ups_status_file=ups_status_file,
            dry_run=False,
        )
        kwargs.update(overrides)
        return guard.GuardConfig(**kwargs)

    def test_not_stuck_cycle_never_calls_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "missing_ups.json")  # never created -> age unknown
            hw = FakeHardware(joint_sequence=[("lo", "hi")] * 3)
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
            # +1 for the self-heal health check every cycle does, then the
            # full 10-sample joint stuck streak.
            hw = FakeHardware(
                joint_sequence=[("lo", "hi")] * 20,
                sda_sequence=["hi"] * 10,  # recovery: 1 pulse check + 3 success reads
            )
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            result = g.run_cycle()
            with open(status_path) as f:
                on_disk = json.load(f)

        self.assertTrue(result["stuck"])
        self.assertEqual(result["sda_level"], "lo")
        self.assertEqual(result["scl_level"], "hi")
        self.assertEqual(g.recoveries_total, 1)
        self.assertEqual(g.last_result, "ok")
        self.assertEqual(result["last_result"], "ok")
        self.assertEqual(result["recoveries_total"], 1)
        self.assertEqual(on_disk, result)
        # Rule B: any recovery attempt restarts the UPS daemon once the
        # driver is bound again.
        self.assertIn(("restart_ups_service",), hw.calls)

    def test_failed_recovery_still_restarts_ups_once_bound(self):
        # Rule B: "after ANY recovery ... success or failure". SDA never
        # frees, so the recovery fails, but the finally block still binds
        # the driver -- the UPS restart must fire anyway.
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            hw = FakeHardware(
                joint_sequence=[("lo", "hi")] * 20,
                sda_sequence=["lo"] * 20,
            )
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            result = g.run_cycle()

        self.assertEqual(result["last_result"], "failed")
        self.assertTrue(hw.is_bound())  # finally's bind() still ran
        self.assertIn(("restart_ups_service",), hw.calls)

    def test_restart_ups_disabled_by_config(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            hw = FakeHardware(
                joint_sequence=[("lo", "hi")] * 20,
                sda_sequence=["hi"] * 10,
            )
            cfg = self._cfg(status_path, ups_path, restart_ups=False)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            result = g.run_cycle()

        self.assertEqual(result["last_result"], "ok")
        self.assertNotIn(("restart_ups_service",), hw.calls)

    def test_self_heal_rebinding_the_driver_restarts_ups(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            # Simulates the half-finished state a kill leaves: unbound,
            # pins off a3. The rest of the cycle (detection) sees a
            # healthy, un-stale bus so nothing else fires.
            hw = FakeHardware(
                bound=False,
                pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "op"},
                joint_sequence=[("hi", "hi")] * 5,
            )
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="WARNING") as captured:
                g.run_cycle()

        self.assertTrue(hw.is_bound())
        self.assertIn(("bind",), hw.calls)
        self.assertIn(("restart_ups_service",), hw.calls)
        heal_lines = [m for m in captured.output if "Self-heal:" in m]
        self.assertEqual(len(heal_lines), 1)

    def test_self_heal_repairs_a_simulated_half_finished_state_next_cycle(self):
        # Exactly the scenario rule A describes: a kill hit recover_bus
        # mid-sequence, so this cycle starts already unbound with pins
        # left as GPIO. The very next cycle must repair it on its own.
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "missing_ups.json")
            hw = FakeHardware(
                bound=False,
                pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "ip"},
                joint_sequence=[("hi", "hi")] * 5,
            )
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(hw, cfg, sleep_fn=_noop_sleep)
            g.run_cycle()

        self.assertTrue(hw.is_bound())
        self.assertEqual(hw.pin_functions[guard.SDA_PIN], "a3")
        self.assertEqual(hw.pin_functions[guard.SCL_PIN], "a3")

    def test_self_heal_pin_drift_alone_does_not_restart_ups(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            # UPS status fresh (age 1s) -- isolates self-heal's own
            # restart decision from the separate stale-bus safety net,
            # which would otherwise also fire on a healthy (hi, hi) bus.
            with open(ups_path, "w") as f:
                json.dump({"ts": 999.0}, f)
            hw = FakeHardware(
                bound=True,
                pin_functions={guard.SDA_PIN: "ip", guard.SCL_PIN: "a3"},
                joint_sequence=[("hi", "hi")] * 5,
            )
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            g.run_cycle()

        self.assertNotIn(("restart_ups_service",), hw.calls)
        self.assertNotIn("unbind", [c[0] for c in hw.calls])

    def test_rate_limited_stuck_logs_error_once_and_stops_attempting(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            # sample_count=2 keeps the joint-sequence bookkeeping small
            # across several cycles: 1 self-heal check + 2 detection
            # samples per cycle.
            hw = FakeHardware(
                joint_sequence=[("lo", "hi")] * 30,
                sda_sequence=["lo"] * 30,  # the one recovery attempt fails
            )
            cfg = self._cfg(status_path, ups_path, max_per_hour=1, sample_count=2)
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
            hw = FakeHardware(
                joint_sequence=[("lo", "hi")] * 40,
                sda_sequence=["lo"] * 60,
            )
            cfg = self._cfg(status_path, ups_path, max_per_hour=1, sample_count=2)
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

    def test_ups_stale_without_stuck_logs_warning_and_sets_status_field(self):
        # SDA/SCL are fine (not a stuck bus) but the UPS status is stale
        # for some other reason -- must still warn and flag the status.
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 940.0}, f)
            # age = 60s: stale (> stale_s=30) but under the 120s
            # safety-net floor, and the bus reads healthy -- isolates
            # this warning from both the stuck path and the safety net.
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 5)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="WARNING") as captured:
                result = g.run_cycle()

        self.assertFalse(result["stuck"])
        self.assertTrue(result["ups_stale"])
        self.assertNotIn(("restart_ups_service",), hw.calls)
        self.assertEqual(g.recoveries_total, 0)
        self.assertNotIn("unbind", [c[0] for c in hw.calls])
        stale_lines = [
            m for m in captured.output
            if "safe shutdown on power loss is NOT active" in m
        ]
        self.assertEqual(len(stale_lines), 1)

    def test_ups_stale_warning_throttled_to_five_minutes(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 999.0}, f)
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 10)
            cfg = self._cfg(status_path, ups_path)
            clock = {"m": 0.0}
            g = guard.I2cBusGuard(
                hw, cfg,
                now_fn=lambda: 1000.0 + clock["m"],
                monotonic_fn=lambda: clock["m"],
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="WARNING") as captured:
                g.run_cycle()
                clock["m"] += 60.0
                g.run_cycle()  # inside the 5-minute window -> no repeat
                clock["m"] += guard.WARNING_REPEAT_S
                g.run_cycle()  # 5 minutes elapsed -> warns again

        stale_lines = [
            m for m in captured.output
            if "safe shutdown on power loss is NOT active" in m
        ]
        self.assertEqual(len(stale_lines), 2)

    def test_missing_ups_status_logs_info_once_not_every_cycle(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "missing_ups.json")  # never created
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 10)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(hw, cfg, sleep_fn=_noop_sleep)
            with self.assertLogs(level="INFO") as captured:
                g.run_cycle()
                g.run_cycle()
                g.run_cycle()

        unknown_lines = [m for m in captured.output if "age unknown" in m]
        self.assertEqual(len(unknown_lines), 1)

    # -- UPS safety net (rule B, second clause) --------------------------

    def test_safety_net_restarts_when_stale_over_120s_and_bus_healthy(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 5)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            with self.assertLogs(level="WARNING") as captured:
                g.run_cycle()

        self.assertIn(("restart_ups_service",), hw.calls)
        reason_lines = [m for m in captured.output if "healthy bus" in m]
        self.assertEqual(len(reason_lines), 1)

    def test_safety_net_does_not_fire_under_120_seconds(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 940.0}, f)  # age = 60s: stale, but < 120s
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 5)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            g.run_cycle()

        self.assertNotIn(("restart_ups_service",), hw.calls)

    def test_safety_net_does_not_fire_when_bus_is_not_healthy(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            # Stale for 1000s (> 120s) but SCL reads low -- ambiguous, not
            # a confirmed-healthy bus, so the safety net must stay quiet.
            hw = FakeHardware(joint_sequence=[("hi", "lo")] * 5)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(
                hw, cfg, now_fn=lambda: 1000.0, monotonic_fn=lambda: 100.0,
                sleep_fn=_noop_sleep,
            )
            g.run_cycle()

        self.assertNotIn(("restart_ups_service",), hw.calls)

    def test_safety_net_never_fires_on_missing_status_file(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "missing_ups.json")
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 5)
            cfg = self._cfg(status_path, ups_path)
            g = guard.I2cBusGuard(hw, cfg, sleep_fn=_noop_sleep)
            g.run_cycle()

        self.assertNotIn(("restart_ups_service",), hw.calls)

    def test_safety_net_rate_limited_to_once_per_fifteen_minutes(self):
        with tempfile.TemporaryDirectory() as d:
            status_path = os.path.join(d, "status.json")
            ups_path = os.path.join(d, "ups.json")
            with open(ups_path, "w") as f:
                json.dump({"ts": 0.0}, f)
            hw = FakeHardware(joint_sequence=[("hi", "hi")] * 20)
            cfg = self._cfg(status_path, ups_path)
            clock = {"m": 0.0}
            g = guard.I2cBusGuard(
                hw, cfg,
                now_fn=lambda: 100000.0 + clock["m"],
                monotonic_fn=lambda: clock["m"],
                sleep_fn=_noop_sleep,
            )
            g.run_cycle()  # fires; age = 100000s, well past 120s
            clock["m"] += 60.0
            g.run_cycle()  # inside the 15-minute window -> no repeat
            restart_calls_before = hw.calls.count(("restart_ups_service",))
            clock["m"] += guard.UPS_SAFETY_NET_WINDOW_S
            g.run_cycle()  # window elapsed -> fires again

        self.assertEqual(restart_calls_before, 1)
        self.assertEqual(hw.calls.count(("restart_ups_service",)), 2)


# ---------------------------------------------------------------------------
# CLI wiring.
# ---------------------------------------------------------------------------

class ArgParserDefaultsTests(unittest.TestCase):
    def test_defaults_match_the_spec(self):
        args = guard.build_arg_parser().parse_args([])
        self.assertEqual(args.interval, 5.0)
        self.assertEqual(args.stale_s, 30.0)
        self.assertEqual(args.sample_count, 10)
        self.assertEqual(args.sample_interval_s, 0.1)
        self.assertEqual(args.max_per_hour, 6)
        self.assertEqual(args.status_file, "/tmp/i2c_guard_status.json")
        self.assertFalse(args.dry_run)
        self.assertFalse(args.once)
        self.assertTrue(args.restart_ups)

    def test_no_restart_ups_flag_disables_it(self):
        args = guard.build_arg_parser().parse_args(["--no-restart-ups"])
        self.assertFalse(args.restart_ups)

    def test_sample_count_and_interval_are_settable(self):
        args = guard.build_arg_parser().parse_args(
            ["--sample-count", "5", "--sample-interval-s", "0.25"]
        )
        self.assertEqual(args.sample_count, 5)
        self.assertEqual(args.sample_interval_s, 0.25)


class MainSmokeTests(unittest.TestCase):
    def test_once_dry_run_writes_a_status_file(self):
        # No pinctrl on a dev machine: Hardware.read_sda()/read_pins()
        # swallow the subprocess failure and report "unknown", so this
        # never looks stuck -- a real end-to-end smoke test of the CLI
        # wiring, signal handler install/restore, and self-heal path
        # (is_bound() is also real and reports False off-Pi, so self-heal
        # runs too) that cannot touch real hardware.
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
        self.assertIn(data["scl_level"], ("hi", "lo", "unknown"))
        self.assertFalse(data["stuck"])
        self.assertIsNone(data["ups_age_s"])
        self.assertFalse(data["ups_stale"])

    def test_signal_handlers_are_restored_after_main_returns(self):
        original_sigterm = signal.getsignal(signal.SIGTERM)
        original_sigint = signal.getsignal(signal.SIGINT)
        with tempfile.TemporaryDirectory() as d:
            guard.main([
                "--once", "--dry-run",
                "--status-file", os.path.join(d, "status.json"),
                "--ups-status-file", os.path.join(d, "no_such_ups.json"),
            ])
        self.assertEqual(signal.getsignal(signal.SIGTERM), original_sigterm)
        self.assertEqual(signal.getsignal(signal.SIGINT), original_sigint)


if __name__ == "__main__":
    unittest.main()
