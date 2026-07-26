"""Tests for the marginal-boot-supply warning.

Diagnosed 2026-07-26: the 5V rail feeding the UPSPlus is a 3A buck off the 13S
pack. After the robot sits, the 18650 goes flat; on power-up the UPS demands
~2.9A to charge it on top of the Pi 5 + OAK-D boot load, exceeding the buck's
3A rating. The regulator drops out of regulation and the Pi hard-cuts with no
OS shutdown and no log (boot on 2026-07-26 08:44 lasted 115s, dirty journal).

Measured input rail vs 18650 charge draw:

    0.19A -> 4893mV    1.24A -> 4864mV    ~2.0A -> ~4.8V    2.88A -> 4184mV

The first three are a gentle resistive droop; the last is the buck dropping
out. This warning fires on the leading indicators so a doomed boot announces
itself instead of just dying.

The warning is advisory ONLY. It must never gate, delay, or alter the
AC-detection / shutdown decision path.
"""

import os
import sys
import types
import unittest
from unittest import mock

# Stub hardware-only modules before importing the daemon (sibling-test pattern).
if "smbus2" not in sys.modules:
    fake_smbus2 = types.ModuleType("smbus2")

    class _FakeSMBus:
        def __init__(self, *_args, **_kwargs):
            pass

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


def _snap(batt_v=4.19, batt_ma=400.0, typec_mv=4850):
    return {
        "battery_v": batt_v,
        "battery_i_ma": batt_ma,
        "typec_mv": typec_mv,
    }


class TestHealthyBootsDoNotWarn(unittest.TestCase):
    """Every real healthy boot in the journal must stay silent."""

    def test_recorded_healthy_boots(self):
        # (batt_v, batt_ma, typec_mv) from the actual UPS initial snapshots.
        for batt_v, batt_ma, typec in (
            (4.200, 192.0, 4893),    # boot -4
            (4.196, 372.0, 4845),    # boot -3
            (4.196, 1238.0, 4864),   # boot -2
            (4.188, 1952.0, 4802),   # boot  0 (the good retry)
        ):
            with self.subTest(batt_v=batt_v):
                self.assertFalse(
                    daemon.warn_marginal_boot_supply(_snap(batt_v, batt_ma, typec)),
                    "healthy boot must not warn",
                )


class TestFailedBootWarns(unittest.TestCase):
    def test_the_boot_that_actually_died(self):
        """boot -1: batt 3.904V, 2875.9mA, typec 4184mV -> died at 115s."""
        self.assertTrue(
            daemon.warn_marginal_boot_supply(_snap(3.904, 2875.9, 4184))
        )

    def test_warning_names_all_three_reasons(self):
        with mock.patch.object(daemon.logging, "warning") as warned:
            daemon.warn_marginal_boot_supply(_snap(3.904, 2875.9, 4184))
        self.assertEqual(warned.call_count, 1)
        reasons = warned.call_args[0][1]
        self.assertIn("flat from sitting", reasons)
        self.assertIn("near the 3A buck limit", reasons)
        self.assertIn("sagging", reasons)

    def test_each_indicator_fires_independently(self):
        self.assertTrue(daemon.warn_marginal_boot_supply(_snap(batt_v=3.90)))
        self.assertTrue(daemon.warn_marginal_boot_supply(_snap(batt_ma=2500.0)))
        self.assertTrue(daemon.warn_marginal_boot_supply(_snap(typec_mv=4300)))


class TestRobustness(unittest.TestCase):
    """It is advisory telemetry — it must never raise into the caller."""

    def test_missing_fields(self):
        self.assertFalse(daemon.warn_marginal_boot_supply({}))

    def test_none_fields(self):
        self.assertFalse(
            daemon.warn_marginal_boot_supply(
                {"battery_v": None, "battery_i_ma": None, "typec_mv": None}
            )
        )

    def test_garbage_snapshot_does_not_raise(self):
        for bad in (None, "nonsense", 42, []):
            with self.subTest(bad=bad):
                self.assertFalse(daemon.warn_marginal_boot_supply(bad))

    def test_zero_typec_is_not_a_sag(self):
        """typec==0 means nothing plugged in, which is the AC-loss path's job,
        not a 'sagging rail' warning."""
        self.assertFalse(daemon.warn_marginal_boot_supply(_snap(typec_mv=0)))


if __name__ == "__main__":
    unittest.main()
