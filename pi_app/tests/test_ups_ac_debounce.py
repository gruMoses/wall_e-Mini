"""Tests for scripts/upsPlus_power_daemon.py — AC-presence detection + debounce.

BUG BEING GUARDED AGAINST (see module docstring in the daemon for full
detail): the previous daemon used battery-current SIGN as the primary
AC-present signal. On a topped-off pack the charge controller taper-cycles,
flipping current sign every 1-2 minutes even though the charger stays
physically connected (charger input voltage stays ~4.7-5.1V throughout).
That flapping repeatedly armed a 30s shutdown-countdown grace window that
only survived because it always got cancelled ~8-9s later — a sustained
load or longer taper window would have let it complete and shut the Pi
down while genuinely on AC power.

The fix moves the authoritative signal to charger input voltage
(typec_mv / microusb_mv) and adds a debounce window before "missing" can
be declared. These tests exercise the pure decision logic
(is_ac_present_instant + AcPresenceTracker) with synthetic
(typec_mv, microusb_mv, elapsed) sequences and no hardware.

smbus2 / ina219 are hardware-only packages not installed on dev machines;
they are stubbed out below (matching the pattern used in
pi_app/tests/test_bms.py for bleak) purely so the daemon module imports
cleanly. Nothing under test touches the stubs.
"""

import os
import sys
import types
import unittest
from unittest import mock

# ---------------------------------------------------------------------------
# Stub out smbus2 / ina219 before importing the daemon so it imports cleanly
# on a machine with no I2C hardware or these hardware-only packages.
# ---------------------------------------------------------------------------
if "smbus2" not in sys.modules:
    fake_smbus2 = types.ModuleType("smbus2")

    class _FakeSMBus:
        def __init__(self, *_args, **_kwargs):
            pass

        def read_byte_data(self, *_args, **_kwargs):
            raise RuntimeError("no hardware in unit tests")

        def write_byte_data(self, *_args, **_kwargs):
            raise RuntimeError("no hardware in unit tests")

    fake_smbus2.SMBus = _FakeSMBus
    sys.modules["smbus2"] = fake_smbus2

if "ina219" not in sys.modules:
    fake_ina219 = types.ModuleType("ina219")

    class _FakeINA219:
        def __init__(self, *_args, **_kwargs):
            pass

        def configure(self):
            pass

        def current(self):
            return 0.0

        def voltage(self):
            return 0.0

    fake_ina219.INA219 = _FakeINA219
    sys.modules["ina219"] = fake_ina219

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "scripts"))

import upsPlus_power_daemon as daemon  # noqa: E402


class IsAcPresentInstantTests(unittest.TestCase):
    def test_typec_above_threshold_is_present(self):
        self.assertTrue(daemon.is_ac_present_instant(4800, 0))

    def test_microusb_above_threshold_is_present(self):
        self.assertTrue(daemon.is_ac_present_instant(0, 4500))

    def test_both_below_threshold_is_absent(self):
        self.assertFalse(daemon.is_ac_present_instant(0, 41))

    def test_threshold_boundary_is_exclusive(self):
        # Exactly at threshold should NOT count as present (strict >).
        self.assertFalse(
            daemon.is_ac_present_instant(
                daemon.AC_PRESENT_VOLTAGE_THRESHOLD_MV,
                daemon.AC_PRESENT_VOLTAGE_THRESHOLD_MV,
            )
        )


class AcPresenceTrackerTests(unittest.TestCase):
    def test_flapping_current_with_steady_voltage_stays_present(self):
        """Reproduces the live bug scenario: charger voltage stays high the
        entire time (typec ~5000mV) while battery current sign flips back
        and forth every ~1-2 minutes. Voltage is the only input the tracker
        sees, so it must report PRESENT continuously and never debounce
        toward MISSING."""
        tracker = daemon.AcPresenceTracker(debounce_s=10.0)
        t = 0.0
        # 5 minutes of steady high charger voltage sampled once per second,
        # mirroring the live journal's typec staying 4.7-5.1V throughout.
        readings = [5048, 4833, 5046, 4964, 4790, 4761, 5006, 4854, 4989, 4828]
        for i in range(300):
            typec_mv = readings[i % len(readings)]
            state = tracker.update(typec_mv, 0, t)
            self.assertTrue(
                state, f"tracker flipped to MISSING at t={t}s despite steady voltage"
            )
            t += 1.0

    def test_sustained_zero_voltage_declares_missing_after_debounce(self):
        tracker = daemon.AcPresenceTracker(debounce_s=10.0)
        t = 0.0
        # Start present.
        self.assertTrue(tracker.update(4900, 0, t))
        t += 1.0

        # Voltage drops to zero (charger genuinely unplugged).
        drop_start = t
        state = None
        while t - drop_start < 10.0:
            state = tracker.update(0, 0, t)
            self.assertTrue(
                state, f"declared MISSING too early at t-drop_start={t - drop_start}s"
            )
            t += 1.0

        # Once we cross the debounce window, it must flip to MISSING.
        state = tracker.update(0, 0, drop_start + 10.0)
        self.assertFalse(state, "did not declare MISSING after debounce window elapsed")

    def test_brief_voltage_dip_within_debounce_does_not_trip(self):
        """A single low reading (e.g. one bad I2C sample) that recovers
        before the debounce window elapses must NOT flip state — this is
        the core anti-flapping guarantee."""
        tracker = daemon.AcPresenceTracker(debounce_s=10.0)
        t = 0.0
        self.assertTrue(tracker.update(4900, 0, t))
        t += 1.0
        # Dip for 3 seconds (well under the 10s debounce).
        for _ in range(3):
            state = tracker.update(0, 0, t)
            self.assertTrue(state)
            t += 1.0
        # Voltage recovers.
        state = tracker.update(4900, 0, t)
        self.assertTrue(state)

    def test_charger_return_is_recognized_immediately(self):
        """Coming back to AC should not itself be debounced — only loss is."""
        tracker = daemon.AcPresenceTracker(debounce_s=10.0)
        t = 0.0
        # Declare missing first.
        while True:
            state = tracker.update(0, 0, t)
            t += 1.0
            if state is False:
                break
            self.assertLess(t, 100.0, "test setup failed to reach MISSING state")

        # A single high reading should flip back to PRESENT right away.
        state = tracker.update(4900, 0, t)
        self.assertTrue(state)

    def test_first_sample_low_requires_debounce_before_missing(self):
        """Even the very first sample the daemon ever sees should not
        instantly declare MISSING off a single low reading."""
        tracker = daemon.AcPresenceTracker(debounce_s=10.0)
        state = tracker.update(0, 0, 0.0)
        self.assertTrue(state, "first low sample should not immediately be MISSING")
        state = tracker.update(0, 0, 10.0)
        self.assertFalse(state, "should be MISSING once debounce window elapses")


# ---------------------------------------------------------------------------
# DETECT-ONLY (bench-test) mode.
#
# These tests cover the safety-critical guarantee that detect-only mode runs
# the ENTIRE detection/debounce/grace chain exactly like armed mode but never
# performs a shutdown action or writes any UPS register, and that it keeps
# monitoring across repeated unplug/replug cycles. They also prove the armed
# (default) path is unchanged.
# ---------------------------------------------------------------------------


class _LoopDone(BaseException):
    """Raised from the stubbed charger read to break the daemon's ``while True``.

    Subclasses BaseException (not Exception) on purpose: the main loop wraps
    the charger read in ``except Exception`` to swallow transient I2C errors,
    so a plain Exception here would be caught and the loop would spin. A
    BaseException propagates cleanly out of ``main()`` to end the test.
    """


PRESENT = (4900, 0)  # typec_mv above threshold -> AC present
ABSENT = (0, 0)  # both charger rails at 0mV -> AC absent


def _make_charger_reader(readings):
    """Return a read_charger_voltages replacement yielding scripted samples.

    Once the script is exhausted it raises _LoopDone to terminate main()'s
    loop deterministically.
    """
    it = iter(readings)

    def _read(*_args, **_kwargs):
        try:
            return next(it)
        except StopIteration:
            raise _LoopDone()

    return _read


_SNAPSHOT = {
    "typec_mv": 0,
    "microusb_mv": 0,
    "protect_mv": 3200,
    "shutdown_countdown_s": 0,
    "auto_power_on": 1,
    "sample_period_min": 2,
    "battery_v": 3.7,
    "battery_i_ma": -1200.0,
}


class RunShutdownSequenceTests(unittest.TestCase):
    """Direct tests of run_shutdown_sequence's action-site guards."""

    def _patches(self, **overrides):
        """Patch every side-effecting collaborator; return the mock dict."""
        patchers = {
            "stop_rover_service": mock.patch.object(daemon, "stop_rover_service"),
            "shed_usb_load": mock.patch.object(daemon, "shed_usb_load"),
            "write_reg_verified": mock.patch.object(
                daemon, "write_reg_verified", return_value=True
            ),
            "read_ups_snapshot": mock.patch.object(
                daemon, "read_ups_snapshot", return_value=dict(_SNAPSHOT)
            ),
            "os_system": mock.patch.object(daemon.os, "system"),
            "time_sleep": mock.patch.object(daemon.time, "sleep"),
        }
        started = {name: p.start() for name, p in patchers.items()}
        for p in patchers.values():
            self.addCleanup(p.stop)
        return started

    def test_detect_only_suppresses_every_shutdown_action(self):
        m = self._patches()
        daemon.run_shutdown_sequence(object(), 0x17, None, detect_only=True)

        # (a) NONE of the shutdown-sequence actions run in detect-only mode.
        m["stop_rover_service"].assert_not_called()
        m["shed_usb_load"].assert_not_called()
        m["write_reg_verified"].assert_not_called()  # no countdown register write
        m["os_system"].assert_not_called()  # no sync, no shutdown
        # No 600s post-shutdown park sleep either.
        for call in m["time_sleep"].call_args_list:
            self.assertNotEqual(call.args and call.args[0], 600)

    def test_detect_only_emits_would_begin_line_once(self):
        self._patches()
        with self.assertLogs(level="INFO") as cm:
            daemon.run_shutdown_sequence(object(), 0x17, None, detect_only=True)
        joined = "\n".join(cm.output)
        # (b) The single unmistakable "would begin shutdown" line is present.
        self.assertIn("DETECT-ONLY", joined)
        self.assertIn("would begin shutdown sequence NOW (suppressed)", joined)
        self.assertEqual(
            joined.count("would begin shutdown sequence NOW (suppressed)"),
            1,
            "the suppression line must be emitted exactly once per grace expiry",
        )

    def test_normal_mode_triggers_every_shutdown_action(self):
        m = self._patches()
        daemon.run_shutdown_sequence(object(), 0x17, None, detect_only=False)

        # (d) The armed/default path performs the full sequence in order.
        m["stop_rover_service"].assert_called_once()
        m["shed_usb_load"].assert_called_once()
        m["write_reg_verified"].assert_called_once_with(
            mock.ANY,
            mock.ANY,
            daemon.REG_SHUTDOWN_COUNTDOWN,
            daemon.UPS_SHUTDOWN_COUNTDOWN_SECONDS,
        )
        m["os_system"].assert_any_call("sudo sync")
        m["os_system"].assert_any_call("sudo /sbin/shutdown -h now")
        m["time_sleep"].assert_any_call(600)

    def test_normal_mode_does_not_emit_detect_only_line(self):
        self._patches()
        with self.assertLogs(level="INFO") as cm:
            daemon.run_shutdown_sequence(object(), 0x17, None, detect_only=False)
        self.assertNotIn("DETECT-ONLY", "\n".join(cm.output))


class ResolveDetectOnlyTests(unittest.TestCase):
    """(e) Both activation paths: CLI flag and env var."""

    def test_cli_flag_activates(self):
        with mock.patch.dict(os.environ, {"UPS_DETECT_ONLY": ""}):
            self.assertTrue(daemon._resolve_detect_only(["--detect-only"]))

    def test_env_var_activates(self):
        for truthy in ("1", "true", "TRUE", "yes", "on"):
            with self.subTest(value=truthy):
                with mock.patch.dict(os.environ, {"UPS_DETECT_ONLY": truthy}):
                    self.assertTrue(daemon._resolve_detect_only([]))

    def test_no_flag_no_env_is_armed(self):
        with mock.patch.dict(os.environ, {"UPS_DETECT_ONLY": ""}):
            self.assertFalse(daemon._resolve_detect_only([]))

    def test_env_falsey_is_armed(self):
        with mock.patch.dict(os.environ, {"UPS_DETECT_ONLY": "0"}):
            self.assertFalse(daemon._resolve_detect_only([]))


class DetectOnlyMainLoopTests(unittest.TestCase):
    """Integration tests that drive the real main() loop with scripted reads.

    Hardware collaborators are stubbed; run_shutdown_sequence is replaced with
    a spy so we can observe how the loop reacts at grace expiry without either
    halting the Pi (armed) or depending on the suppression internals (already
    covered above). A virtual clock advances only via the daemon's own
    time.sleep() calls, so the debounce/grace timing is faithful.
    """

    def _drive(self, readings, detect_only):
        clock = {"t": 0.0}

        def fake_sleep(dur):
            clock["t"] += dur

        def fake_monotonic():
            return clock["t"]

        shutdown_spy = mock.Mock()

        with mock.patch.object(daemon, "detect_addr", return_value=0x17), \
            mock.patch.object(
                daemon, "read_charger_voltages", _make_charger_reader(readings)
            ), \
            mock.patch.object(
                daemon, "read_ups_snapshot", return_value=dict(_SNAPSHOT)
            ), \
            mock.patch.object(daemon, "write_reg_verified", return_value=True) as m_write, \
            mock.patch.object(daemon, "run_shutdown_sequence", shutdown_spy), \
            mock.patch.object(daemon.time, "sleep", fake_sleep), \
            mock.patch.object(daemon.time, "monotonic", fake_monotonic):
            with self.assertLogs(level="INFO") as cm:
                try:
                    daemon.main(detect_only=detect_only)
                except _LoopDone:
                    pass
        return shutdown_spy, m_write, "\n".join(cm.output)

    @staticmethod
    def _one_grace_cycle():
        # 10 boot-grace iters + 2 to init PRESENT, then enough ABSENT samples
        # to clear the 10s debounce and the 30s grace window with margin.
        return [PRESENT] * 12 + [ABSENT] * 42

    def test_detect_only_continues_and_repeats_across_replug(self):
        # Two full unplug->grace-expiry cycles separated by a replug.
        readings = (
            self._one_grace_cycle()  # cycle 1 -> first grace expiry
            + [PRESENT] * 3  # replug: present-again + grace reset
            + [ABSENT] * 42  # cycle 2 -> second grace expiry
        )
        spy, m_write, logs = self._drive(readings, detect_only=True)

        # (c) The loop did NOT exit at first grace expiry: it reached a second.
        self.assertEqual(spy.call_count, 2, "grace expiry should recur after a replug")
        for call in spy.call_args_list:
            self.assertIs(call.kwargs["detect_only"], True)

        # Replug was detected and the grace timer reset between the two cycles.
        self.assertIn("UPS AC state changed -> present", logs)
        self.assertIn("Charger present again; resetting grace timer.", logs)

        # Startup banner shown; NO UPS register writes happened at all
        # (startup writes skipped, and the spied shutdown path writes nothing).
        self.assertIn("MODE: DETECT-ONLY", logs)
        m_write.assert_not_called()

    def test_normal_mode_reaches_shutdown_sequence_and_writes_startup_regs(self):
        # Single unplug to grace expiry; spy stands in for the real shutdown.
        spy, m_write, logs = self._drive(self._one_grace_cycle(), detect_only=False)

        # (d) Default path reaches the shutdown sequence at grace expiry.
        self.assertGreaterEqual(spy.call_count, 1)
        self.assertIs(spy.call_args_list[0].kwargs["detect_only"], False)

        # Armed mode performs its startup register writes and shows no banner.
        m_write.assert_called()  # auto-power-on + battery-protect thresholds
        self.assertNotIn("MODE: DETECT-ONLY", logs)


if __name__ == "__main__":
    unittest.main()
