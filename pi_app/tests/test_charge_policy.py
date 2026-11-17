"""Tests for the charger plug policy (pi_app/control/charge_policy.py)."""

import math
import unittest

from config import ChargerPlugConfig
from pi_app.control.charge_policy import ChargeInputs, ChargePolicy


def cfg(**kw):
    return ChargerPlugConfig(host="192.0.2.1", **kw)


def inp(soc=None, v=None, a=None, fresh=True, plug_on=False):
    return ChargeInputs(soc_pct=soc, pack_v=v, pack_a=a, bms_fresh=fresh, plug_on=plug_on)


class ManualModes(unittest.TestCase):
    def test_off_and_on_ignore_the_bms(self):
        p = ChargePolicy(cfg(), "off")
        self.assertIs(p.decide(inp(fresh=False), 0.0).want_on, False)
        p.set_mode("on")
        self.assertIs(p.decide(inp(fresh=False), 0.0).want_on, True)

    def test_set_mode_rejects_unknown(self):
        p = ChargePolicy(cfg())
        with self.assertRaises(ValueError):
            p.set_mode("maybe")

    def test_unknown_initial_mode_falls_back_to_hold(self):
        self.assertEqual(ChargePolicy(cfg(), "bogus").mode, "hold")


class HoldMode(unittest.TestCase):
    def test_band(self):
        p = ChargePolicy(cfg(), "hold")
        self.assertIs(p.decide(inp(soc=100.0, v=54.5), 0).want_on, False)
        self.assertIs(p.decide(inp(soc=44.0, v=48.0), 1).want_on, True)
        self.assertIsNone(p.decide(inp(soc=50.0, v=48.5), 2).want_on)
        self.assertIs(p.decide(inp(soc=56.0, v=49.0), 3).want_on, False)
        self.assertIs(p.decide(inp(soc=45.0, v=48.0), 4).want_on, True)   # at the edge
        self.assertIs(p.decide(inp(soc=55.0, v=49.0), 5).want_on, False)

    def test_voltage_guard_beats_a_low_soc(self):
        p = ChargePolicy(cfg(), "hold")
        d = p.decide(inp(soc=44.0, v=52.5), 0)
        self.assertIs(d.want_on, False)
        self.assertIn("guard", d.reason)
        # No voltage: the SOC alone decides.
        self.assertIs(p.decide(inp(soc=44.0, v=None), 1).want_on, True)

    def test_stale_bms_waits_then_switches_off(self):
        p = ChargePolicy(cfg(soc_unknown_off_s=120.0), "hold")
        d0 = p.decide(inp(soc=44.0, fresh=False, plug_on=True), 1000.0)
        self.assertIsNone(d0.want_on)
        d1 = p.decide(inp(soc=44.0, fresh=False, plug_on=True), 1119.0)
        self.assertIsNone(d1.want_on)
        d2 = p.decide(inp(soc=44.0, fresh=False, plug_on=True), 1120.0)
        self.assertIs(d2.want_on, False)
        self.assertIn("no BMS reading", d2.reason)
        # A fresh reading resets the timer and the normal rule applies.
        self.assertIs(p.decide(inp(soc=44.0, v=48.0), 1130.0).want_on, True)
        self.assertIsNone(p.decide(inp(soc=44.0, fresh=False), 1140.0).want_on)

    def test_implausible_soc_counts_as_unknown(self):
        p = ChargePolicy(cfg(soc_unknown_off_s=10.0), "hold")
        for bad in (None, -1.0, 150.0, math.nan, "x"):
            q = ChargePolicy(cfg(soc_unknown_off_s=10.0), "hold")
            self.assertIsNone(q.decide(inp(soc=bad, fresh=True), 0.0).want_on, bad)
            self.assertIs(q.decide(inp(soc=bad, fresh=True), 10.0).want_on, False, bad)
        del p

    def test_implausible_voltage_is_ignored_not_trusted(self):
        p = ChargePolicy(cfg(), "hold")
        self.assertIs(p.decide(inp(soc=44.0, v=0.0), 0).want_on, True)
        self.assertIs(p.decide(inp(soc=44.0, v=999.0), 1).want_on, True)


class FullMode(unittest.TestCase):
    def test_charges_then_finishes_then_tops_up(self):
        c = cfg(full_done_hold_s=120.0)
        p = ChargePolicy(c, "full")
        self.assertIs(p.decide(inp(soc=50.0, v=48.0, a=0.0, plug_on=False), 0).want_on, True)
        # Full reached, current tapered, plug on: hold the done condition first.
        d = p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 100)
        self.assertIs(d.want_on, True)
        self.assertIn("finishing", d.reason)
        d = p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 219)
        self.assertIs(d.want_on, True)
        d = p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 220)
        self.assertIs(d.want_on, False)
        self.assertIn("complete", d.reason)
        self.assertTrue(p.topped)
        # Topped: stays off while SOC is high, tops up at the threshold.
        self.assertIs(p.decide(inp(soc=99.0, v=54.0, a=0.0, plug_on=False), 300).want_on, False)
        d = p.decide(inp(soc=95.0, v=53.0, a=0.0, plug_on=False), 400)
        self.assertIs(d.want_on, True)
        self.assertIn("top-up", d.reason)
        self.assertFalse(p.topped)

    def test_done_condition_needs_the_plug_on_and_a_tapered_current(self):
        p = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        # Plug off at 100 %: still "charging" (no taper evidence without the charger).
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=0.0, plug_on=False), 0).want_on, True)
        # Plug state unknown: the same.
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=0.0, plug_on=None), 1).want_on, True)
        # Current unknown: not done.
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=None, plug_on=True), 2).want_on, True)
        # Current still flowing: not done.
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=1.5, plug_on=True), 3).want_on, True)
        # All three: done.
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 4).want_on, False)

    def test_voltage_alone_can_say_full(self):
        p = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        self.assertIs(p.decide(inp(soc=97.0, v=54.4, a=0.2, plug_on=True), 0).want_on, False)

    def test_done_timer_resets_when_the_condition_drops(self):
        p = ChargePolicy(cfg(full_done_hold_s=120.0), "full")
        p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 0)
        p.decide(inp(soc=100.0, v=54.5, a=2.0, plug_on=True), 60)     # current back up
        d = p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 130)
        self.assertIs(d.want_on, True)                                  # timer restarted at 130
        self.assertIs(p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 250).want_on, False)

    def test_returns_to_hold_after_the_pack_was_used(self):
        p = ChargePolicy(cfg(), "full")
        p.decide(inp(soc=30.0, v=46.0, a=0.0), 0)       # below return pct: not "used" yet
        self.assertEqual(p.mode, "full")
        p.decide(inp(soc=70.0, v=50.0, a=3.9, plug_on=True), 1)
        self.assertTrue(p.seen_above_return)
        d = p.decide(inp(soc=55.0, v=49.0, a=-5.0, plug_on=False), 2)
        self.assertEqual(d.mode, "hold")
        self.assertEqual(p.mode, "hold")
        self.assertIs(d.want_on, False)                # hold rule on the same tick: 55 >= 55
        self.assertFalse(p.seen_above_return)
        self.assertFalse(p.topped)

    def test_full_mode_ignores_the_hold_voltage_guard(self):
        p = ChargePolicy(cfg(), "full")
        self.assertIs(p.decide(inp(soc=90.0, v=53.5, a=3.0, plug_on=True), 0).want_on, True)

    def test_stale_bms_switches_off_in_full_too(self):
        p = ChargePolicy(cfg(soc_unknown_off_s=120.0), "full")
        p.decide(inp(fresh=False, plug_on=True), 0)
        self.assertIs(p.decide(inp(fresh=False, plug_on=True), 120).want_on, False)

    def test_set_mode_clears_full_state(self):
        p = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        p.decide(inp(soc=100.0, v=54.5, a=0.1, plug_on=True), 0)
        self.assertTrue(p.topped)
        p.set_mode("full")
        self.assertFalse(p.topped)
        self.assertFalse(p.seen_above_return)


if __name__ == "__main__":
    unittest.main()
