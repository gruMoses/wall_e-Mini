"""Tests for two UPS-daemon hardening changes in scripts/upsPlus_power_daemon.py.

CHANGE 1 — STARTUP STALE-COUNTDOWN CLEAR (see the main() action-site comment):
  An armed shutdown-countdown (reg 0x18) is NOT cleared by AC returning — it
  keeps ticking and can hard-cut the 5V rail mid-recovery-boot (documented
  hardware gotcha). At startup, if AC is present AND the countdown register
  reads non-zero, the daemon writes 0 to clear it. If AC is ABSENT it must NOT
  clear (a live outage may legitimately be counting down). Detect-only mode
  suppresses the write at its own call site and logs what it WOULD clear.

CHANGE 2 — SHED LOG NOISE (see shed_usb_load):
  uhubctl exits non-zero with "No compatible devices detected at location N!"
  whenever nothing is enumerated at a target hub — expected on the USB3.0
  companion hub because the OAK negotiates USB2. That is not a shed failure, so
  it is demoted from WARNING to INFO. Genuine uhubctl failures stay WARNING.

smbus2 / ina219 are hardware-only packages; they are stubbed before importing
the daemon (same pattern as the sibling UPS test modules) so the module imports
on a dev machine with no I2C.
"""

import os
import sys
import types
import unittest
from unittest import mock

# ---------------------------------------------------------------------------
# Stub hardware-only modules before importing the daemon.
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


class _LoopDone(BaseException):
    """Break the daemon's ``while True`` from the stubbed charger read.

    Subclasses BaseException so the loop's ``except Exception`` (which swallows
    transient I2C errors) does not catch it — it propagates out of main() once
    the startup path (the code under test) has fully run.
    """


def _make_charger_reader(readings):
    it = iter(readings)

    def _read(*_args, **_kwargs):
        try:
            return next(it)
        except StopIteration:
            raise _LoopDone()

    return _read


def _snapshot(*, typec_mv, microusb_mv=0, countdown):
    return {
        "typec_mv": typec_mv,
        "microusb_mv": microusb_mv,
        "protect_mv": 3400,
        "shutdown_countdown_s": countdown,
        "auto_power_on": 1,
        "sample_period_min": 2,
        "battery_v": 3.7,
        "battery_i_ma": -1200.0,
    }


# ---------------------------------------------------------------------------
# CHANGE 1 — startup stale-countdown clear.
# ---------------------------------------------------------------------------
class StartupCountdownClearTests(unittest.TestCase):
    """Drive the real main() startup path; the infinite loop is broken by
    _LoopDone on the first charger read so only the startup code executes.

    write_reg_verified is spied so the countdown-clear write can be isolated
    from the unrelated startup writes (Back-to-AC reg 25, battery-protect regs
    17/18) by filtering for (REG_SHUTDOWN_COUNTDOWN, 0).
    """

    def _drive_startup(self, snapshot, *, detect_only=False):
        clock = {"t": 0.0}

        def fake_sleep(dur):
            clock["t"] += dur

        def fake_monotonic():
            return clock["t"]

        write_spy = mock.Mock(return_value=True)
        with mock.patch.object(daemon, "detect_addr", return_value=0x17), \
            mock.patch.object(
                daemon, "read_charger_voltages", _make_charger_reader([])
            ), \
            mock.patch.object(daemon, "read_ups_snapshot", return_value=dict(snapshot)), \
            mock.patch.object(daemon, "write_reg_verified", write_spy), \
            mock.patch.object(daemon, "read_reg_with_retry", return_value=0), \
            mock.patch.object(daemon, "shed_usb_load"), \
            mock.patch.object(daemon, "restore_usb_load", return_value=True), \
            mock.patch.object(daemon.time, "sleep", fake_sleep), \
            mock.patch.object(daemon.time, "monotonic", fake_monotonic):
            with self.assertLogs(level="INFO") as cm:
                try:
                    daemon.main(detect_only=detect_only)
                except _LoopDone:
                    pass
        return write_spy, "\n".join(cm.output)

    @staticmethod
    def _countdown_clears(write_spy):
        return [
            c
            for c in write_spy.call_args_list
            if c.args[2] == daemon.REG_SHUTDOWN_COUNTDOWN and c.args[3] == 0
        ]

    def test_clears_when_ac_present_and_countdown_nonzero(self):
        write_spy, logs = self._drive_startup(
            _snapshot(typec_mv=4900, countdown=30)
        )
        clears = self._countdown_clears(write_spy)
        self.assertEqual(
            len(clears), 1, "exactly one countdown-clear write expected"
        )
        # The clear targets the shutdown-countdown register with value 0.
        self.assertEqual(clears[0].args[2], daemon.REG_SHUTDOWN_COUNTDOWN)
        self.assertEqual(clears[0].args[3], 0)
        self.assertIn("Stale UPS shutdown countdown found at startup", logs)
        self.assertIn("clearing to 0", logs)
        self.assertIn("Stale UPS shutdown countdown cleared", logs)

    def test_does_not_clear_when_ac_absent(self):
        # Countdown non-zero but no charger voltage: a live outage may legitimately
        # be counting down — the daemon must NOT disarm it.
        write_spy, logs = self._drive_startup(
            _snapshot(typec_mv=0, microusb_mv=0, countdown=30)
        )
        self.assertEqual(self._countdown_clears(write_spy), [])
        self.assertNotIn("Stale UPS shutdown countdown found at startup", logs)

    def test_does_not_clear_when_countdown_already_zero(self):
        write_spy, logs = self._drive_startup(
            _snapshot(typec_mv=4900, countdown=0)
        )
        self.assertEqual(self._countdown_clears(write_spy), [])
        self.assertNotIn("Stale UPS shutdown countdown found at startup", logs)

    def test_detect_only_logs_would_clear_and_writes_nothing(self):
        write_spy, logs = self._drive_startup(
            _snapshot(typec_mv=4900, countdown=30), detect_only=True
        )
        # Detect-only suppresses EVERY startup register write at its own site.
        write_spy.assert_not_called()
        self.assertIn(
            "DETECT-ONLY: would clear stale UPS shutdown countdown", logs
        )
        # And it must not emit the armed-mode "clearing"/"cleared" lines.
        self.assertNotIn("clearing to 0", logs)


# ---------------------------------------------------------------------------
# CHANGE 2 — shed_usb_load log level for the "No compatible devices" case.
# ---------------------------------------------------------------------------
class ShedLogNoiseTests(unittest.TestCase):
    """The uhubctl 'No compatible devices' non-zero exit is expected (nothing
    enumerated at a target) and must log at INFO; any other non-zero exit is a
    genuine failure and stays WARNING. Shed behavior/ordering is unchanged —
    only the log level of one branch is affected."""

    def _run_shed_with(self, *, returncode, stderr):
        fake_run = mock.Mock(
            return_value=mock.Mock(returncode=returncode, stderr=stderr)
        )
        with mock.patch.object(
            daemon.shutil, "which", return_value="/usr/bin/uhubctl"
        ), mock.patch.object(daemon.subprocess, "run", fake_run):
            with self.assertLogs(level="INFO") as cm:
                daemon.shed_usb_load()
        return fake_run, cm

    def test_no_compatible_devices_logs_info_not_warning(self):
        fake_run, cm = self._run_shed_with(
            returncode=1, stderr="No compatible devices detected at location 4!"
        )
        joined = "\n".join(cm.output)
        # The demoted, informative line is present…
        self.assertIn("no device enumerated at", joined)
        self.assertIn("nothing to shed there", joined)
        # …and NOTHING was logged at WARNING or above for this expected case.
        warnings = [r for r in cm.records if r.levelno >= 30]  # logging.WARNING
        self.assertEqual(
            warnings, [], "the No-compatible-devices case must not warn"
        )
        # Shed behavior unchanged: still attempted both uhubctl hub targets.
        self.assertEqual(fake_run.call_count, 2)
        for c in fake_run.call_args_list:
            self.assertIn("off", c.args[0])

    def test_genuine_uhubctl_failure_still_warns(self):
        _fake_run, cm = self._run_shed_with(
            returncode=1, stderr="Permission denied opening /dev/...; bus error"
        )
        joined = "\n".join(cm.output)
        self.assertIn("uhubctl power off failed", joined)
        warnings = [r for r in cm.records if r.levelno >= 30]
        self.assertTrue(warnings, "a real uhubctl failure must still WARN")
        # And it must NOT be misclassified as the benign no-device case.
        self.assertNotIn("nothing to shed there", joined)


if __name__ == "__main__":
    unittest.main()
