"""Tests for early USB load-shedding + staggered restore in the UPS daemon.

BEHAVIOR UNDER TEST (see scripts/upsPlus_power_daemon.py, the USB_RESTORE_DELAY_S
constants block for the hardware theory):

  * The OAK-D hangs off a powered USB hub fed from the MAIN PACK. When the pack
    switches off, that hub's devices dump their full draw onto the Pi/UPS at the
    exact transfer instant — a hard cut. So the daemon SHEDS OAK-hub USB power
    the moment input loss is seen on THIS poll's instant charger reading
    (pre-debounce, ~1s), fire-once per outage.
  * RESTORE is late + staggered: USB power is only re-applied after AC has been
    continuously clean for USB_RESTORE_DELAY_S (15s), measured from the last
    unclean poll, so hub re-enumeration inrush is staggered clear of the UPS
    charge inrush at pack-return. A failed restore is retried, never latched.
  * DETECT-ONLY (bench) mode suppresses BOTH actions per site and stays inert.

These drive the real main() loop on a virtual clock with scripted charger
reads (same harness style as test_ups_ac_debounce.py) and spy on
shed_usb_load / restore_usb_load so the timing/idempotency guarantees are
exercised end to end without touching hardware.

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


# ---------------------------------------------------------------------------
# Scripted-loop harness (mirrors test_ups_ac_debounce.py).
# ---------------------------------------------------------------------------
class _LoopDone(BaseException):
    """Raised from the stubbed charger read to break the daemon's ``while True``.

    Subclasses BaseException so the loop's ``except Exception`` (which swallows
    transient I2C errors) does not catch it — it propagates cleanly out of
    main() to end the drive.
    """


PRESENT = (4900, 0)  # typec_mv above threshold -> AC present
ABSENT = (0, 0)      # both charger rails at 0mV -> AC absent

# 12 present readings: 10 consumed by BOOT_GRACE_SECONDS, then 2 steady
# full-body polls before any scripted outage begins.
_BOOT = [PRESENT] * 12

_SNAPSHOT = {
    "typec_mv": 0,
    "microusb_mv": 0,
    "protect_mv": 3400,
    "shutdown_countdown_s": 0,
    "auto_power_on": 1,
    "sample_period_min": 2,
    "battery_v": 3.7,
    "battery_i_ma": -1200.0,
}


def _make_charger_reader(readings):
    it = iter(readings)

    def _read(*_args, **_kwargs):
        try:
            return next(it)
        except StopIteration:
            raise _LoopDone()

    return _read


class _MainDriver(unittest.TestCase):
    """Base with a helper that drives main() on a virtual clock with spies."""

    def _drive(self, readings, *, detect_only=False, restore_return=True,
               restore_side_effect=None):
        clock = {"t": 0.0}

        def fake_sleep(dur):
            clock["t"] += dur

        def fake_monotonic():
            return clock["t"]

        shed_spy = mock.Mock(return_value=None)
        if restore_side_effect is not None:
            restore_spy = mock.Mock(side_effect=restore_side_effect)
        else:
            restore_spy = mock.Mock(return_value=restore_return)

        with mock.patch.object(daemon, "detect_addr", return_value=0x17), \
            mock.patch.object(
                daemon, "read_charger_voltages", _make_charger_reader(readings)
            ), \
            mock.patch.object(daemon, "read_ups_snapshot", return_value=dict(_SNAPSHOT)), \
            mock.patch.object(daemon, "write_reg_verified", return_value=True), \
            mock.patch.object(daemon, "shed_usb_load", shed_spy), \
            mock.patch.object(daemon, "restore_usb_load", restore_spy), \
            mock.patch.object(daemon.time, "sleep", fake_sleep), \
            mock.patch.object(daemon.time, "monotonic", fake_monotonic):
            with self.assertLogs(level="INFO") as cm:
                try:
                    daemon.main(detect_only=detect_only)
                except _LoopDone:
                    pass
        return shed_spy, restore_spy, "\n".join(cm.output)


class EarlyShedTests(_MainDriver):
    def test_a_sheds_once_on_first_instant_low_and_not_again(self):
        # (a) First instant-low full-body poll sheds exactly once; the two
        # further low polls do NOT re-shed (fire-once via usb_shed_active).
        shed, restore, logs = self._drive(_BOOT + [ABSENT] * 3)
        self.assertEqual(shed.call_count, 1)
        restore.assert_not_called()
        self.assertIn("shedding OAK hub USB power", logs)
        self.assertIn("(early shed)", logs)

    def test_shed_fires_pre_debounce_not_after_grace(self):
        # The shed must land on the FIRST low full-body poll, well before the
        # 10s debounce could flip the tracker to MISSING (only 3 low polls here,
        # far short of debounce/grace) — proving it is instant-driven.
        shed, _restore, _logs = self._drive(_BOOT + [ABSENT] * 3)
        self.assertEqual(shed.call_count, 1)


class StaggeredRestoreTests(_MainDriver):
    def test_b_restore_fires_after_15s_continuous_present_and_clears_flag(self):
        # (b) shed, then AC returns; restore fires exactly once after 15s of
        # continuous clean AC and clears the flag (so it does not re-fire on the
        # remaining present polls).
        shed, restore, logs = self._drive(_BOOT + [ABSENT] * 3 + [PRESENT] * 20)
        self.assertEqual(shed.call_count, 1)
        self.assertEqual(restore.call_count, 1)
        self.assertIn("restoring USB power", logs)

    def test_c_flap_present_5s_then_low_again_does_not_restore(self):
        # (c) AC returns for only 5s (<15s) then drops again: restore must NOT
        # fire, and the still-active shed must NOT re-shed on the second drop.
        shed, restore, _logs = self._drive(
            _BOOT + [ABSENT] * 3 + [PRESENT] * 5 + [ABSENT] * 3
        )
        self.assertEqual(shed.call_count, 1)
        restore.assert_not_called()

    def test_d_restore_failure_is_retried_next_poll(self):
        # (d) First restore attempt reports failure (returns False): the flag is
        # NOT cleared, so the next poll (conditions still hold) retries and
        # succeeds. Exactly two restore attempts, no more once it succeeds.
        shed, restore, logs = self._drive(
            _BOOT + [ABSENT] * 3 + [PRESENT] * 20,
            restore_side_effect=[False, True],
        )
        self.assertEqual(shed.call_count, 1)
        self.assertEqual(restore.call_count, 2)
        self.assertIn("USB restore incomplete", logs)


class DetectOnlyTests(_MainDriver):
    def test_e_detect_only_suppresses_both_actions(self):
        # (e) In detect-only mode NEITHER shed nor restore is called; both
        # suppression lines are logged and the drive is zero-side-effect.
        shed, restore, logs = self._drive(
            _BOOT + [ABSENT] * 3 + [PRESENT] * 20, detect_only=True
        )
        shed.assert_not_called()
        restore.assert_not_called()
        self.assertIn("DETECT-ONLY: would shed USB power (suppressed).", logs)
        self.assertIn("DETECT-ONLY: would restore USB power (suppressed).", logs)
        self.assertIn("MODE: DETECT-ONLY", logs)


class ShutdownSequenceShedTests(unittest.TestCase):
    """(f) The grace-expiry shutdown path sheds ONLY when the early shed has
    not already fired this outage (QA 2026-07-10: re-shedding = two uhubctl
    spawns with 5s timeouts each against already-off ports — pure
    critical-path risk under the ~20s firmware guillotine). shed_usb_load
    itself stays idempotent-safe for direct callers and edge paths."""

    def _sequence_patches(self):
        patchers = [
            mock.patch.object(daemon, "shed_usb_load"),
            mock.patch.object(daemon, "write_reg_verified", return_value=True),
            mock.patch.object(daemon, "read_ups_snapshot", return_value=dict(_SNAPSHOT)),
            mock.patch.object(daemon.os, "system"),
            mock.patch.object(daemon.time, "sleep"),
        ]
        mocks = [p.start() for p in patchers]
        for p in patchers:
            self.addCleanup(p.stop)
        return mocks[0]  # the shed spy

    def test_shutdown_sequence_sheds_when_not_already_shed(self):
        # Belt-and-braces path: a direct caller that never early-shed.
        shed_spy = self._sequence_patches()
        daemon.run_shutdown_sequence(object(), 0x17, None, detect_only=False)
        shed_spy.assert_called_once()

    def test_shutdown_sequence_skips_shed_when_early_shed_fired(self):
        # The armed main-loop path: early shed fired within ~1s of input loss,
        # so at grace expiry the sequence must NOT spawn uhubctl again.
        shed_spy = self._sequence_patches()
        with self.assertLogs(level="INFO") as cm:
            daemon.run_shutdown_sequence(
                object(), 0x17, None, detect_only=False, usb_already_shed=True
            )
        shed_spy.assert_not_called()
        self.assertIn("skipping re-shed", "\n".join(cm.output))

    def test_real_shed_is_idempotent_safe_across_two_calls(self):
        # shed_usb_load stays idempotent for direct callers/edge paths: two
        # consecutive calls must not raise and must only ever power OFF
        # (never slip in a power-on).
        fake_run = mock.Mock(return_value=mock.Mock(returncode=0, stderr=""))
        with mock.patch.object(daemon.shutil, "which", return_value="/usr/bin/uhubctl"), \
            mock.patch.object(daemon.subprocess, "run", fake_run):
            daemon.shed_usb_load()
            daemon.shed_usb_load()
        # 2 hub targets x 2 calls, every invocation an "-a off".
        self.assertEqual(fake_run.call_count, 4)
        for c in fake_run.call_args_list:
            self.assertIn("off", c.args[0])
            self.assertNotIn("on", c.args[0])


class ShedRestoreMechanismTests(unittest.TestCase):
    """The uhubctl-vs-sysfs target mechanism and shed/restore symmetry."""

    def test_targets_exclude_ch340_rc_link(self):
        uhubctl_targets, sysfs_targets = daemon._usb_power_targets()
        # CH340 VESC serial (RC link) lives at Bus 3 Port 2 / path 3-2 and must
        # NEVER be a shed/restore target.
        self.assertNotIn(("3", "2"), [(loc, port) for loc, port, _ in uhubctl_targets])
        for path, _label in sysfs_targets:
            self.assertNotIn("3-2", path)
        # Both act on the same OAK hub locations.
        self.assertIn(("3", "1", "USB2.0 hub (OAK-D)"), uhubctl_targets)
        self.assertIn(("4", "1", "USB3.0 companion hub"), uhubctl_targets)

    def test_restore_uses_power_on_and_returns_true_on_success(self):
        uhubctl_targets, _sysfs = daemon._usb_power_targets()
        fake_run = mock.Mock(return_value=mock.Mock(returncode=0, stderr=""))
        with mock.patch.object(daemon.shutil, "which", return_value="/usr/bin/uhubctl"), \
            mock.patch.object(daemon.subprocess, "run", fake_run):
            ok = daemon.restore_usb_load()
        self.assertTrue(ok)
        self.assertEqual(fake_run.call_count, len(uhubctl_targets))
        for c in fake_run.call_args_list:
            self.assertIn("on", c.args[0])
            self.assertNotIn("off", c.args[0])

    def test_restore_returns_false_when_a_target_fails(self):
        fake_run = mock.Mock(return_value=mock.Mock(returncode=1, stderr="nope"))
        with mock.patch.object(daemon.shutil, "which", return_value="/usr/bin/uhubctl"), \
            mock.patch.object(daemon.subprocess, "run", fake_run):
            self.assertFalse(daemon.restore_usb_load())

    def test_restore_sysfs_fallback_writes_one_to_same_paths(self):
        # With uhubctl absent, restore re-authorizes the SAME sysfs paths shed
        # deauthorized, writing "1".
        _uhubctl, sysfs_targets = daemon._usb_power_targets()
        writes = {}

        class _FakeFile:
            def __init__(self, path):
                self._path = path

            def write(self, data):
                writes[self._path] = data

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        def fake_open(path, mode):
            return _FakeFile(path)

        with mock.patch.object(daemon.shutil, "which", return_value=None), \
            mock.patch("builtins.open", side_effect=fake_open):
            ok = daemon.restore_usb_load()
        self.assertTrue(ok)
        for path, _label in sysfs_targets:
            self.assertEqual(writes.get(path), "1")


class UsbShedConstants(unittest.TestCase):
    """(g) The new knob is the agreed value and detection/grace constants are
    untouched (the pre-existing DecisionLogicUnchanged guard still passes as-is)."""

    def test_restore_delay_constant(self):
        self.assertEqual(daemon.USB_RESTORE_DELAY_S, 15.0)

    def test_restore_delay_outlasts_recovery_quiet(self):
        # Must sit past the I2C bus-quiet recovery holdoff so bus-quiet ends
        # first and the hub re-enumeration is staggered away from charge inrush.
        self.assertGreater(daemon.USB_RESTORE_DELAY_S, daemon.AC_RECOVERY_QUIET_S)

    def test_detection_and_grace_constants_untouched(self):
        # The USB-shed feature does not touch these; their values are the
        # guillotine-tuned budget set 2026-07-10 (debounce 4s + grace 4s so the
        # OS halt finishes before the ~20s UPS firmware output guillotine).
        self.assertEqual(daemon.AC_PRESENT_VOLTAGE_THRESHOLD_MV, 4000)
        self.assertEqual(daemon.AC_LOSS_DEBOUNCE_S, 4.0)
        self.assertEqual(daemon.NO_CHARGE_GRACE_SECONDS, 4)


if __name__ == "__main__":
    unittest.main()
