"""Tests for the UPS watcher's battery-discharge loss detection (2026-09-27).

The UPS MCU's charger-voltage register froze at 5057 mV (every MCU register
stopped updating) after the 2026-09-25 I2C lock-up, so a main-power-off on
2026-09-26 17:25 was never detected: no clean shutdown, and the 18650 ran
flat 66 minutes later. The INA219 gauge read the outage correctly (-4 to
-7 A). DischargeLossTracker declares input loss from a sustained discharge;
these tests pin that it catches the recorded outage and never fires on the
charger taper pattern the module docstring describes.
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

Tracker = daemon.DischargeLossTracker


def _feed(tracker, samples, start=1000.0):
    """samples: [(t_offset_s, batt_ma)] -> list of verdicts after each."""
    out = []
    for t, ma in samples:
        out.append(tracker.update(ma, start + t, start + t + 0.5))
    return out


class TestDischargeLossTracker(unittest.TestCase):
    def test_recorded_outage_is_detected_after_60_s(self):
        # 2026-09-27 12:04:55-12:06:05 INA219 readings (10 s cadence).
        samples = [(0, 1605.9), (10, -4693.9), (20, -4032.0), (30, -4442.0),
                   (40, -4209.8), (50, -5545.9), (60, -5371.7), (70, -4247.8)]
        verdicts = _feed(Tracker(), samples)
        self.assertEqual(verdicts[:7], [False] * 7)   # streak 10 -> 60 s: 50 s
        self.assertTrue(verdicts[7])                   # 10 -> 70 s: 60 s

    def test_charger_taper_dips_never_trigger(self):
        # One -3372 mA reading every 90 s between positive readings, for an hour.
        tracker = Tracker()
        t = 0.0
        for _ in range(40):
            self.assertFalse(tracker.update(-3372.0, 1000 + t, 1000 + t))
            self.assertFalse(tracker.update(2370.0, 1000 + t + 10, 1000 + t + 10))
            t += 90
        self.assertFalse(tracker.lost(1000 + t))

    def test_one_positive_reading_resets_the_streak(self):
        samples = [(0, -4000), (10, -4000), (20, -4000), (30, -4000),
                   (40, 500), (50, -4000), (60, -4000), (70, -4000)]
        self.assertFalse(any(_feed(Tracker(), samples)))

    def test_repeated_sample_does_not_extend_the_streak(self):
        tracker = Tracker()
        tracker.update(-4000, 1000.0, 1000.0)
        for k in range(100):                      # same ts polled at 1 Hz
            self.assertFalse(tracker.update(-4000, 1000.0, 1000.0 + k))

    def test_needs_enough_samples(self):
        # Two readings 60 s apart are not a sustained-discharge record.
        tracker = Tracker()
        tracker.update(-4000, 1000.0, 1000.0)
        self.assertFalse(tracker.update(-4000, 1060.0, 1060.0))

    def test_stale_reading_is_not_evidence(self):
        tracker = Tracker()
        samples = [(t, -5000) for t in range(0, 80, 10)]
        self.assertTrue(_feed(tracker, samples)[-1])
        self.assertFalse(tracker.lost(1000 + 70 + 31))   # gauge went silent

    def test_threshold_is_strict(self):
        samples = [(t, -2500.0) for t in range(0, 80, 10)]
        self.assertFalse(any(_feed(Tracker(), samples)))
        samples = [(t, -2501.0) for t in range(0, 80, 10)]
        self.assertTrue(_feed(Tracker(), samples)[-1])

    def test_missing_values_are_ignored(self):
        tracker = Tracker()
        self.assertFalse(tracker.update(None, None, 1000.0))
        self.assertFalse(tracker.update("x", 1000.0, 1000.0))
        self.assertEqual(tracker.streak_s, 0.0)

    def test_detection_bounds_the_blind_time(self):
        # From the first discharge reading, loss is declared within 70 s at
        # the 10 s supplemental cadence (the outage ran 66 min undetected).
        tracker = Tracker()
        t_first = None
        for k in range(20):
            t = 1000.0 + 10 * k
            if tracker.update(-4500, t, t):
                t_first = 10 * k
                break
        self.assertIsNotNone(t_first)
        self.assertLessEqual(t_first, 70)


class TestFrozenRegisterWatch(unittest.TestCase):
    def test_unchanged_value_accumulates_and_change_resets(self):
        watch = daemon.FrozenRegisterWatch()
        self.assertEqual(watch.update(5057, 0.0), 0.0)
        self.assertEqual(watch.update(5057, 1800.0), 1800.0)
        self.assertEqual(watch.update(5048, 1801.0), 0.0)

    def test_warning_repeats_hourly(self):
        watch = daemon.FrozenRegisterWatch()
        self.assertTrue(watch.should_warn(0.0))
        self.assertFalse(watch.should_warn(3599.0))
        self.assertTrue(watch.should_warn(3600.0))


if __name__ == "__main__":
    unittest.main()
