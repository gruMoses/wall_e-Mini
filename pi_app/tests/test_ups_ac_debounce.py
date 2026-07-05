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


if __name__ == "__main__":
    unittest.main()
