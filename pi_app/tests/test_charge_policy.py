"""Tests for the charger plug policy (pi_app/control/charge_policy.py)."""

import unittest

from config import ChargerPlugConfig
from pi_app.control.charge_policy import ChargeInputs, ChargePolicy, soc_from_cell_v

REST = 300.0   # rest_after_charge_s: the first rested reading after a start


def cfg(**kw):
    return ChargerPlugConfig(**kw)


def inp(cell=3.75, spread=30.0, a=-0.3, fresh=True, plug_on=False, soc=50.0, temp=24.0,
        flags=(), fet=True, cells=13, vesc=None, armed=False, v=None, vtemp=20.0):
    return ChargeInputs(
        soc_pct=soc, pack_v=(v if v is not None else round(cell * 13, 1)), pack_a=a,
        bms_fresh=fresh, plug_on=plug_on,
        cell_min_mv=cell * 1000 - spread / 2, cell_max_mv=cell * 1000 + spread / 2,
        temp_min_c=temp, temp_max_c=temp, charge_fet_on=fet, error_flags=tuple(flags),
        num_cells=cells, vesc_v=vesc, vesc_temp_c=vtemp, armed=armed)


def settled(mode="hold", **kw):
    """A policy that has seen the plug off for the rest time."""
    p = ChargePolicy(cfg(**kw), mode)
    p.decide(inp(), 0.0)
    return p


def codes(dec):
    return [a["code"] for a in dec.alerts]


class Curve(unittest.TestCase):
    def test_interpolation_and_clamps(self):
        t = cfg().ocv_table
        self.assertEqual(soc_from_cell_v(3.0, t), 7.0)
        self.assertEqual(soc_from_cell_v(4.5, t), 100.0)
        self.assertAlmostEqual(soc_from_cell_v(3.721, t), 47.0)
        self.assertAlmostEqual(soc_from_cell_v(3.7355, t), 49.5, places=1)


class ManualModes(unittest.TestCase):
    def test_off_and_on_ignore_the_bms(self):
        p = ChargePolicy(cfg(), "off")
        self.assertIs(p.decide(inp(fresh=False), 0.0).want_on, False)
        p.set_mode("on")
        self.assertIs(p.decide(inp(fresh=False), 1.0).want_on, True)

    def test_set_mode_rejects_unknown(self):
        with self.assertRaises(ValueError):
            ChargePolicy(cfg()).set_mode("maybe")

    def test_unknown_initial_mode_falls_back_to_hold(self):
        self.assertEqual(ChargePolicy(cfg(), "bogus").mode, "hold")


class Hold(unittest.TestCase):
    def test_settles_before_the_first_decision(self):
        p = ChargePolicy(cfg(), "hold")
        d = p.decide(inp(cell=3.60), 0.0)
        self.assertIs(d.want_on, False)
        self.assertIn("settling", d.reason)
        self.assertIs(p.decide(inp(cell=3.60), REST - 1).want_on, False)
        self.assertIs(p.decide(inp(cell=3.60), REST).want_on, True)

    def test_band_is_on_cell_voltage_not_bms_percent(self):
        p = settled()
        d = p.decide(inp(cell=3.76, soc=20.0), REST)        # BMS says 20 %, voltage says fine
        self.assertIs(d.want_on, False)
        self.assertIn("in the band", d.reason)
        d = p.decide(inp(cell=3.69, soc=90.0), REST + 10)   # BMS says 90 %, voltage says low
        self.assertIs(d.want_on, True)
        self.assertEqual(d.info["soc_by_voltage"], 41)

    def test_top_up_is_sized_from_the_rest_curve(self):
        p = settled()
        d = p.decide(inp(cell=3.70), REST)
        self.assertIs(d.want_on, True)
        self.assertIn("0.99 Ah", d.reason)                  # (57 - 42.8) % of 7 Ah

    def test_weak_cell_starts_a_top_up(self):
        p = settled()
        d = p.decide(inp(cell=3.72, spread=250.0), REST)    # mid above 3.70, weakest at 3.595
        self.assertIs(d.want_on, True)

    def test_top_up_ends_on_amp_hours(self):
        p = settled()
        self.assertIs(p.decide(inp(cell=3.70), REST).want_on, True)
        t = REST + 10
        ticks = 0
        while True:
            d = p.decide(inp(cell=3.73 + 0.0007 * ticks, a=3.6, plug_on=True), t)
            if d.want_on is False:
                break
            t += 10
            ticks += 1
            self.assertLess(ticks, 200)
        self.assertIn("top-up complete", d.reason)
        self.assertIsNone(p.fault)
        self.assertAlmostEqual(ticks * 10 * 3.6 / 3600.0, 0.99, delta=0.03)
        # Plug off: the session is over, then it settles again before a reading.
        d = p.decide(inp(cell=3.80), t + 10)
        self.assertIsNone(d.info["session"])
        self.assertIn("settling", d.reason)

    def test_voltage_ceiling_and_time_limit(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        d = p.decide(inp(cell=3.84, a=3.6, plug_on=True), REST + 10)
        d = p.decide(inp(cell=3.84, a=3.6, plug_on=True), REST + 20)
        self.assertIs(d.want_on, False)
        self.assertIn("top of the band", d.reason)
        q = settled()
        q.decide(inp(cell=3.70), REST)
        q.decide(inp(cell=3.75, a=1.0, plug_on=True), REST + 10)
        d = q.decide(inp(cell=3.75, a=1.0, plug_on=True), REST + 10 + 1790)
        self.assertIs(d.want_on, True)                       # limit is 600 + 0.994 / 3 A = 1793 s
        d = q.decide(inp(cell=3.75, a=1.0, plug_on=True), REST + 10 + 1800)
        self.assertIs(d.want_on, False)
        self.assertIn("time limit", d.reason)
        self.assertIsNone(q.fault)

    def test_no_current_backs_off_and_retries(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        p.decide(inp(cell=3.70, a=-0.3, plug_on=True), REST + 10)
        d = p.decide(inp(cell=3.70, a=-0.3, plug_on=True), REST + 10 + 299)
        self.assertIs(d.want_on, True)
        d = p.decide(inp(cell=3.70, a=-0.3, plug_on=True), REST + 10 + 300)
        self.assertIs(d.want_on, False)
        self.assertIn("no_current", codes(d))
        t_off = REST + 320
        d = p.decide(inp(cell=3.70), t_off)                  # plug off now
        self.assertIs(d.want_on, False)
        self.assertIn("no_current", codes(d))
        d = p.decide(inp(cell=3.70), t_off + 880)            # still inside the 900 s retry wait
        self.assertIs(d.want_on, False)
        d = p.decide(inp(cell=3.70), t_off + 900)            # retry wait over, 30 min spacing not
        self.assertIs(d.want_on, False)
        self.assertIn("next charge in", d.reason)
        d = p.decide(inp(cell=3.70), REST + 10 + 1800)       # 30 min after that charge started
        self.assertIs(d.want_on, True)

    def test_hold_charges_start_at_least_30_min_apart(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        p.decide(inp(cell=3.72, a=3.6, plug_on=True), REST + 10)       # the charge starts here
        p.decide(inp(cell=3.84, a=3.6, plug_on=True), REST + 60)       # ...and ends at the ceiling
        p.decide(inp(cell=3.78), REST + 70)                             # plug off
        d = p.decide(inp(cell=3.69), REST + 70 + REST)                  # rested and low again, too soon
        self.assertIs(d.want_on, False)
        self.assertIn("next charge in", d.reason)
        d = p.decide(inp(cell=3.69), REST + 10 + 1800)
        self.assertIs(d.want_on, True)

    def test_never_starts_above_the_pack_guard_and_adopts_a_running_plug(self):
        p = settled()
        d = p.decide(inp(cell=4.19, spread=7.0), REST)
        self.assertIs(d.want_on, False)
        q = ChargePolicy(cfg(), "hold")
        d = q.decide(inp(cell=4.19, spread=7.0, a=0.0, plug_on=True), 0.0)   # deployed onto a live charger
        self.assertIs(d.want_on, False)
        self.assertIn("top of the band", d.reason)

    def test_armed_blocks_a_charge(self):
        p = settled()
        d = p.decide(inp(cell=3.60, armed=True), REST)
        self.assertIs(d.want_on, False)
        self.assertEqual(d.phase, "armed")
        p.set_mode("on")
        self.assertIs(p.decide(inp(cell=3.60, armed=True), REST + 1).want_on, True)   # manual is manual


class Inhibits(unittest.TestCase):
    def check(self, word, mode="hold", **kw):
        p = settled(mode)
        d = p.decide(inp(cell=3.60, **kw), REST)
        self.assertIs(d.want_on, False, kw)
        self.assertEqual(d.phase, "inhibited", kw)
        self.assertIn(word, d.reason)
        self.assertIn("inhibit", codes(d))
        self.assertIsNone(p.fault)
        # It clears by itself.
        self.assertIs(p.decide(inp(cell=3.60), REST + 10).want_on, True)

    def test_temperature_window(self):
        self.check("too cold", temp=4.0)
        self.check("too warm", temp=46.0)

    def test_full_charge_needs_a_warmer_pack(self):
        self.check("too cold", mode="full", temp=8.0)
        p = settled("hold")
        self.assertIs(p.decide(inp(cell=3.60, temp=8.0), REST).want_on, True)

    def test_bms_temperature_trip_and_open_fet(self):
        self.check("Charge UTP Trip", flags=("Charge UTP Trip",))
        self.check("Charge OTP Trip", flags=("Charge OTP Trip",))
        self.check("charge FET", fet=False)

    def test_high_cell_stops_a_charge(self):
        p = settled("full")
        d = p.decide(inp(cell=4.22, spread=30.0), REST)       # highest cell 4.235 V
        self.assertIs(d.want_on, False)
        self.assertIn("highest cell", d.reason)

    def test_disagreeing_voltages_inhibit_then_latch(self):
        p = settled()
        d = p.decide(inp(cell=3.60, v=40.0), REST)             # one bad sample: pause only
        self.assertEqual(d.phase, "inhibited")
        self.assertIsNone(p.fault)
        self.assertIs(p.decide(inp(cell=3.60), REST + 10).want_on, True)
        for i in range(3):                                      # three in a row: a person looks
            p.decide(inp(cell=3.60, v=40.0), REST + 20 + 10 * i)
        self.assertEqual(p.fault["code"], "bms_data")

    def test_no_temperature_reading(self):
        p = settled()
        d = p.decide(ChargeInputs(soc_pct=40, pack_v=46.8, pack_a=-0.3, bms_fresh=True, plug_on=False,
                                  cell_min_mv=3585, cell_max_mv=3615, charge_fet_on=True, num_cells=13), REST)
        self.assertIs(d.want_on, False)
        self.assertIn("temperature", d.reason)

    def test_warnings_do_not_inhibit(self):
        p = settled("full")
        d = p.decide(inp(cell=4.15, spread=8.0, a=2.0, plug_on=True,
                         flags=("Cell OVP Warning", "Pack OVP Warning")), REST)
        self.assertIs(d.want_on, True)


class Faults(unittest.TestCase):
    def test_bms_hardware_flag_latches_and_blocks_manual_on(self):
        p = settled()
        d = p.decide(inp(cell=3.60, flags=("Charge FET Failure",)), REST)
        self.assertIs(d.want_on, False)
        self.assertEqual(p.fault["code"], "bms_flag")
        self.assertEqual(d.alerts[0]["level"], "fault")
        d = p.decide(inp(cell=3.60), REST + 10)               # the flag is gone: still latched
        self.assertIs(d.want_on, False)
        p.set_mode("on")
        self.assertIs(p.decide(inp(cell=3.60), REST + 20).want_on, False)
        self.assertTrue(p.clear_fault())
        self.assertIs(p.decide(inp(cell=3.60), REST + 30).want_on, True)
        self.assertFalse(p.clear_fault())

    def test_bms_overvoltage_or_overcurrent_trip_latches(self):
        for flag in ("Cell OVP Trip", "Pack OVP Trip", "Charge OCP Trip"):
            p = settled("full")
            d = p.decide(inp(cell=4.20, spread=40.0, a=0.0, plug_on=True, flags=(flag,)), REST)
            self.assertIs(d.want_on, False, flag)
            self.assertEqual(p.fault["code"], "bms_flag", flag)
            d = p.decide(inp(cell=4.18, spread=20.0), REST + 10)     # released: still latched
            self.assertIs(d.want_on, False, flag)

    def test_deeply_discharged_cell(self):
        p = settled()
        p.decide(inp(cell=3.1, spread=400.0, a=-20.0), REST)   # 2.90 V under load: sag, not a fault
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=3.1, spread=400.0, a=-0.3), REST + 10)   # 2.90 V at rest
        self.assertEqual(p.fault["code"], "cell_low")
        self.assertIn("by hand", p.fault["text"])
        self.assertIs(d.want_on, False)
        q = settled()
        q.decide(inp(cell=2.9, spread=900.0, a=-25.0), REST)   # 2.45 V even under load
        self.assertEqual(q.fault["code"], "cell_low")

    def test_amp_hours_without_a_voltage_rise(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        t = REST + 10
        while p.fault is None and t < REST + 2000:
            p.decide(inp(cell=3.73, a=3.6, plug_on=True), t)     # 3.6 A in, the cells do not move
            t += 10
        self.assertEqual(p.fault["code"], "charge_no_rise")
        self.assertAlmostEqual((t - REST - 20) * 3.6 / 3600.0, 0.6, delta=0.03)

    def test_cell_count_and_over_temperature(self):
        p = settled()
        p.decide(inp(cells=12), REST)
        self.assertEqual(p.fault["code"], "cell_count")
        q = settled()
        q.decide(inp(temp=50.0), REST)
        self.assertEqual(q.fault["code"], "over_temp")

    def test_temperature_rise_during_a_charge(self):
        p = settled("full")
        p.decide(inp(cell=3.80, temp=24.0), REST)
        p.decide(inp(cell=3.85, a=3.6, plug_on=True, temp=24.0), REST + 10)
        d = p.decide(inp(cell=3.90, a=3.6, plug_on=True, temp=33.0), REST + 600)
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=3.95, a=3.6, plug_on=True, temp=34.0), REST + 700)
        self.assertEqual(p.fault["code"], "temp_rise")
        self.assertIs(d.want_on, False)

    def test_overfill_latches(self):
        p = settled("full")
        p.decide(inp(cell=3.80), REST)
        t = REST + 10
        while p.fault is None and t < REST + 12000:
            p.decide(inp(cell=4.0, spread=8.0, a=3.6, plug_on=True), t)
            t += 10
        self.assertEqual(p.fault["code"], "charge_overfill")
        self.assertAlmostEqual((t - REST - 20) * 3.6 / 3600.0, 8.5, delta=0.1)

    def test_full_charge_that_never_finishes(self):
        p = settled("full")
        p.decide(inp(cell=3.80), REST)
        p.decide(inp(cell=4.0, spread=8.0, a=1.0, plug_on=True), REST + 10)
        t = REST + 20
        while t < REST + 10 + 14400:
            d = p.decide(inp(cell=4.0, spread=8.0, a=1.0, plug_on=True), t)
            self.assertIsNone(p.fault)
            t += 600
        d = p.decide(inp(cell=4.0, spread=8.0, a=1.0, plug_on=True), REST + 10 + 14400)
        self.assertEqual(p.fault["code"], "charge_overrun")
        self.assertIs(d.want_on, False)

    def test_plug_that_does_not_switch_off(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        t0 = REST + 10
        p.decide(inp(cell=3.75, a=0.6, plug_on=True), t0)
        limit = 600 + 0.994 / 3.0 * 3600
        d = p.decide(inp(cell=3.75, a=0.6, plug_on=True), t0 + limit + 1)
        self.assertIs(d.want_on, False)                        # the policy says off...
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=3.75, a=0.6, plug_on=True), t0 + limit + 299)   # ...the plug stays on
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=3.75, a=0.6, plug_on=True), t0 + limit + 301)
        self.assertEqual(p.fault["code"], "charge_overrun")

    def test_restore_fault(self):
        p = ChargePolicy(cfg(), "hold")
        p.restore_fault({"code": "cell_low", "text": "lowest cell 2.70 V"})
        self.assertIs(p.decide(inp(cell=3.60), 0.0).want_on, False)
        p.restore_fault({"junk": 1})                           # ignored
        self.assertEqual(p.fault["code"], "cell_low")


class Spread(unittest.TestCase):
    def level(self, cell, spread, hold_s=300.0):
        p = settled()
        p.decide(inp(cell=cell, spread=spread), REST)
        d = p.decide(inp(cell=cell, spread=spread), REST + hold_s)
        return p, d

    def test_this_packs_normal_is_quiet(self):
        _, d = self.level(3.75, 30.0)
        self.assertNotIn("cell_spread", codes(d))
        _, d = self.level(4.10, 8.0)
        self.assertNotIn("cell_spread", codes(d))

    def test_notice_and_warning_in_the_middle(self):
        _, d = self.level(3.75, 48.0)
        a = [x for x in d.alerts if x["code"] == "cell_spread"][0]
        self.assertEqual(a["level"], "notice")
        self.assertIn("full charge", a["text"])
        _, d = self.level(3.75, 65.0)
        self.assertEqual([x for x in d.alerts if x["code"] == "cell_spread"][0]["level"], "warning")

    def test_debounced(self):
        _, d = self.level(3.75, 48.0, hold_s=290.0)
        self.assertNotIn("cell_spread", codes(d))

    def test_top_zone_is_stricter(self):
        _, d = self.level(4.10, 25.0)
        self.assertEqual([x for x in d.alerts if x["code"] == "cell_spread"][0]["level"], "notice")

    def test_a_failing_group_latches_a_fault(self):
        p, d = self.level(3.75, 110.0)
        self.assertEqual(p.fault["code"], "cell_spread")
        self.assertIs(d.want_on, False)

    def test_not_evaluated_under_the_knee_or_while_charging(self):
        p, d = self.level(3.50, 150.0)
        self.assertIsNone(p.fault)
        self.assertNotIn("cell_spread", codes(d))
        q = settled("full")
        q.decide(inp(cell=3.80), REST)
        for i in range(60):                                    # under charge only the fault level counts
            d = q.decide(inp(cell=3.9, spread=80.0, a=3.6, plug_on=True), REST + 10 + i * 10)
        self.assertIsNone(q.fault)
        self.assertNotIn("cell_spread", codes(d))

    def test_a_cell_that_pulls_away_under_charge_latches(self):
        p = settled("full")
        p.decide(inp(cell=3.80), REST)
        p.decide(inp(cell=3.9, spread=120.0, a=3.6, plug_on=True), REST + 10)
        p.decide(inp(cell=3.9, spread=120.0, a=3.6, plug_on=True), REST + 60)
        self.assertIsNone(p.fault)                              # 60 s debounce
        d = p.decide(inp(cell=3.9, spread=120.0, a=3.6, plug_on=True), REST + 70)
        self.assertEqual(p.fault["code"], "cell_spread")
        self.assertIs(d.want_on, False)


class OffPlugCharge(unittest.TestCase):
    def test_notice_when_the_pack_charges_with_the_plug_off(self):
        p = settled()
        d = p.decide(inp(cell=3.9, a=3.6, plug_on=False), REST)
        self.assertNotIn("charge_plug_off", codes(d))
        d = p.decide(inp(cell=3.9, a=3.6, plug_on=False), REST + 120)
        self.assertIn("charge_plug_off", codes(d))
        self.assertEqual([a for a in d.alerts if a["code"] == "charge_plug_off"][0]["level"], "warning")
        self.assertIs(d.want_on, False)
        d = p.decide(inp(cell=3.9, a=-0.3, plug_on=False), REST + 130)
        self.assertNotIn("charge_plug_off", codes(d))


class Full(unittest.TestCase):
    def test_charge_finish_rest_top_up(self):
        p = settled("full")
        d = p.decide(inp(cell=3.75), REST)
        self.assertIs(d.want_on, True)
        t = REST + 10
        d = p.decide(inp(cell=4.10, spread=8.0, a=3.6, plug_on=True, soc=90.0), t)
        self.assertIs(d.want_on, True)
        # Full and tapered: held for 120 s first.
        d = p.decide(inp(cell=4.19, spread=8.0, a=0.2, plug_on=True, soc=100.0), t + 100)
        self.assertIs(d.want_on, True)
        self.assertIn("finishing", d.reason)
        d = p.decide(inp(cell=4.19, spread=8.0, a=0.1, plug_on=True, soc=100.0), t + 219)
        self.assertIs(d.want_on, True)
        d = p.decide(inp(cell=4.195, spread=8.0, a=0.1, plug_on=True, soc=100.0), t + 220)
        self.assertIs(d.want_on, False)
        self.assertTrue(d.full_completed)
        self.assertTrue(p.topped)
        # Resting above the top-up level: stays off.
        d = p.decide(inp(cell=4.15, spread=6.0), t + 230)
        self.assertIs(d.want_on, False)
        self.assertFalse(d.full_completed)
        d = p.decide(inp(cell=4.10, spread=6.0), t + 230 + REST)
        self.assertIs(d.want_on, False)
        d = p.decide(inp(cell=4.07, spread=6.0), t + 240 + REST)
        self.assertIs(d.want_on, True)
        self.assertIn("top-up", d.reason)

    def test_done_needs_taper_and_the_plug_on(self):
        p = settled("full")
        p.decide(inp(cell=4.0), REST)
        d = p.decide(inp(cell=4.19, spread=8.0, a=1.5, plug_on=True, soc=100.0), REST + 10)
        self.assertIs(d.want_on, True)
        self.assertNotIn("finishing", d.reason)
        q = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        d = q.decide(inp(cell=4.19, spread=8.0, a=0.0, plug_on=False, soc=100.0), 0.0)
        self.assertIs(d.want_on, True)                          # no taper evidence with the plug off

    def test_cell_voltage_alone_can_say_full(self):
        p = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        d = p.decide(inp(cell=4.18, spread=8.0, a=0.2, plug_on=True, soc=93.0, v=54.2), 0.0)
        self.assertIs(d.want_on, False)
        self.assertTrue(d.full_completed)

    def test_a_high_cell_is_the_top_with_no_retry_loop(self):
        p = settled("full")
        p.decide(inp(cell=4.0), REST)
        d = p.decide(inp(cell=4.215, spread=40.0, a=2.0, plug_on=True), REST + 10)   # highest 4.235 V
        self.assertIs(d.want_on, False)
        self.assertEqual(d.phase, "inhibited")
        self.assertTrue(p.topped)
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=4.19, spread=30.0, a=-0.3), REST + 20)    # it relaxed: do not go again
        self.assertIs(d.want_on, False)
        self.assertIn("complete", d.reason)

    def test_bms_percent_100_with_the_charger_lead_off_is_not_full(self):
        p = settled("full")
        p.decide(inp(cell=3.75, soc=100.0), REST)
        d = p.decide(inp(cell=3.75, soc=100.0, a=-0.3, plug_on=True), REST + 10)
        d = p.decide(inp(cell=3.75, soc=100.0, a=-0.3, plug_on=True), REST + 200)
        self.assertIs(d.want_on, True)
        self.assertNotIn("finishing", d.reason)
        self.assertFalse(d.full_completed)
        d = p.decide(inp(cell=3.75, soc=100.0, a=-0.3, plug_on=True), REST + 311)
        self.assertIs(d.want_on, False)
        self.assertIn("no_current", codes(d))
        self.assertFalse(p.topped)

    def test_idle_drain_on_a_full_pack_is_not_a_taper(self):
        p = ChargePolicy(cfg(full_done_hold_s=0.0), "full")
        d = p.decide(inp(cell=4.19, spread=8.0, a=-0.3, plug_on=True), 0.0)   # lead off, pack full
        self.assertFalse(d.full_completed)

    def test_full_expires_to_hold_after_48_h(self):
        p = ChargePolicy(cfg(), "full")
        p.set_mode("full", 0.0)
        p.decide(inp(cell=4.19, spread=8.0, a=0.1, plug_on=True), 0.0)
        d = p.decide(inp(cell=4.15, spread=8.0), 172799.0)
        self.assertEqual(d.mode, "full")
        d = p.decide(inp(cell=4.15, spread=8.0), 172800.0)
        self.assertEqual(d.mode, "hold")
        self.assertIs(d.want_on, False)
        # The age survives a restart.
        q = ChargePolicy(cfg(), "full")
        q.restore_mode_age(172000.0)
        q.decide(inp(cell=4.15, spread=8.0), 5000.0)
        self.assertEqual(q.mode, "full")
        self.assertEqual(q.decide(inp(cell=4.15, spread=8.0), 5800.0).mode, "hold")

    def test_returns_to_hold_after_a_drive(self):
        p = settled("full")
        p.decide(inp(cell=3.60), REST)                           # low at the start: not "used" yet
        self.assertEqual(p.mode, "full")
        p.decide(inp(cell=4.05, spread=8.0, a=3.6, plug_on=True), REST + 10)
        p.decide(inp(cell=3.85, a=-0.3, plug_on=False), REST + 20)      # plug off, not rested yet
        self.assertEqual(p.mode, "full")
        d = p.decide(inp(cell=3.79, a=-0.3, plug_on=False), REST + 20 + REST)
        self.assertEqual(d.mode, "hold")
        self.assertIs(d.want_on, False)                          # 3.79 V is inside the hold band
        self.assertFalse(p.topped)

    def test_no_current_in_full(self):
        p = settled("full")
        p.decide(inp(cell=3.75), REST)
        p.decide(inp(cell=3.75, a=-0.3, plug_on=True), REST + 10)
        d = p.decide(inp(cell=3.75, a=-0.3, plug_on=True), REST + 311)
        self.assertIs(d.want_on, False)
        self.assertIn("no_current", codes(d))


class Dark(unittest.TestCase):
    def test_waits_then_off_and_nothing_without_a_vesc_voltage(self):
        p = ChargePolicy(cfg(), "hold")
        self.assertIsNone(p.decide(inp(fresh=False, plug_on=True), 0.0).want_on)
        self.assertIsNone(p.decide(inp(fresh=False, plug_on=True), 119.0).want_on)
        d = p.decide(inp(fresh=False, plug_on=True), 120.0)
        self.assertIs(d.want_on, False)
        self.assertIn("bms_dark", codes(d))
        for t in (700.0, 5000.0, 50000.0):
            self.assertIs(p.decide(inp(fresh=False), t).want_on, False)

    def test_wake_pulse_schedule(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=48.0), 599.0).want_on, False)
        d = p.decide(inp(fresh=False, vesc=48.0), 600.0)
        self.assertIs(d.want_on, True)
        self.assertEqual(d.phase, "wake")
        self.assertIn("wake pulse 1 of 3", d.reason)
        self.assertIs(p.decide(inp(fresh=False, vesc=48.3, plug_on=True), 700.0).want_on, True)
        self.assertIs(p.decide(inp(fresh=False, vesc=48.3, plug_on=True), 780.0).want_on, False)
        self.assertIsNone(p.fault)
        # Next pulse 180 s + 30 min after the first.
        self.assertIs(p.decide(inp(fresh=False, vesc=48.0), 600.0 + 180 + 1799).want_on, False)
        d = p.decide(inp(fresh=False, vesc=48.0), 600.0 + 180 + 1800)
        self.assertIs(d.want_on, True)
        self.assertIn("wake pulse 2 of 3", d.reason)

    def test_no_wake_pulse_on_a_high_pack_and_a_daily_cap(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=52.5), 700.0).want_on, False)
        q = ChargePolicy(cfg(wake_backoff_s=(60.0,)), "hold")
        q.decide(inp(fresh=False), 0.0)
        t, on = 600.0, False
        while t < 20000.0:
            d = q.decide(inp(fresh=False, vesc=48.4 if on else 48.0, plug_on=on), t)
            on = bool(d.want_on)
            t += 10
        self.assertEqual(len(q._wake_times), 3)                 # three a day, however short the backoff
        self.assertIsNone(q.fault)

    def test_rescue_top_up(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        d = p.decide(inp(fresh=False, vesc=46.5), 120.0)
        self.assertIs(d.want_on, True)
        self.assertEqual(d.phase, "rescue")
        self.assertIs(p.decide(inp(fresh=False, vesc=46.9, plug_on=True), 800.0).want_on, True)
        self.assertIs(p.decide(inp(fresh=False, vesc=46.9, plug_on=True), 840.0).want_on, False)
        # No wake pulse chained onto it, and no second rescue inside the hour
        # (start to start), even though the pack is still low.
        self.assertIs(p.decide(inp(fresh=False, vesc=47.2), 850.0).want_on, False)
        self.assertIs(p.decide(inp(fresh=False, vesc=47.1), 2000.0).want_on, False)
        d = p.decide(inp(fresh=False, vesc=46.6), 840.0 + 3 * 3600)   # drained back three hours later
        self.assertIs(d.want_on, True)
        self.assertEqual(d.phase, "rescue")

    def test_blind_charging_needs_a_healthy_last_reading_and_a_vesc_temperature(self):
        p = settled()
        p.decide(inp(cell=3.60, temp=3.0), REST)                # last reading: too cold
        p.decide(inp(fresh=False, vesc=46.0), REST + 10)
        d = p.decide(inp(fresh=False, vesc=46.0), REST + 10 + 700)
        self.assertIs(d.want_on, False)
        self.assertIn("too cold", d.reason)
        for vt, word in ((None, "no VESC temperature"), (5.0, "too cold"), (50.0, "too warm")):
            q = ChargePolicy(cfg(), "hold")
            q.decide(inp(fresh=False), 0.0)
            d = q.decide(inp(fresh=False, vesc=46.0, vtemp=vt), 700.0)
            self.assertIs(d.want_on, False, vt)
            self.assertIn(word, d.reason)

    def test_no_voltage_answer_ends_blind_charging(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=48.0), 600.0).want_on, True)
        self.assertIs(p.decide(inp(fresh=False, vesc=48.0, plug_on=True), 700.0).want_on, True)
        d = p.decide(inp(fresh=False, vesc=48.1, plug_on=True), 720.0)   # 120 s in: +0.1 V only
        self.assertIs(d.want_on, False)
        self.assertIn("did not rise", d.reason)
        for t in (3000.0, 9000.0, 30000.0):                     # no more pulses, no rescue
            self.assertIs(p.decide(inp(fresh=False, vesc=46.0), t).want_on, False)
        # A fresh reading ends the block.
        p.decide(inp(cell=3.75), 30010.0)
        p.decide(inp(fresh=False, vesc=46.0), 30020.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=46.0), 30020.0 + 120).want_on, True)

    def test_rescue_stops_when_the_pack_does_not_keep_the_charge(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=46.5), 120.0).want_on, True)
        p.decide(inp(fresh=False, vesc=46.9, plug_on=True), 500.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=47.0, plug_on=True), 840.0).want_on, False)
        # Back at the start voltage one hour later: a healthy pack keeps a rescue about 3 h.
        d = p.decide(inp(fresh=False, vesc=46.6), 120.0 + 3600)
        self.assertIs(d.want_on, False)
        self.assertIn("does not hold", d.reason)
        self.assertIs(p.decide(inp(fresh=False, vesc=46.0), 20000.0).want_on, False)

    def test_rescue_repeats_for_a_healthy_pack_and_has_a_daily_cap(self):
        p = ChargePolicy(cfg(rescue_max_per_day=2, wake_enabled=False), "hold")
        p.decide(inp(fresh=False), 0.0)
        t0 = 120.0
        for n in range(2):
            self.assertIs(p.decide(inp(fresh=False, vesc=46.7), t0).want_on, True, n)
            p.decide(inp(fresh=False, vesc=47.1, plug_on=True), t0 + 400)
            self.assertIs(p.decide(inp(fresh=False, vesc=47.2, plug_on=True), t0 + 720).want_on, False)
            t0 += 720 + 3 * 3600                                 # drained back three hours later
        d = p.decide(inp(fresh=False, vesc=46.7), t0)
        self.assertIs(d.want_on, False)
        self.assertIn("no more", d.reason)

    def test_a_pack_that_started_lower_gets_a_second_rescue(self):
        p = ChargePolicy(cfg(wake_enabled=False), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=45.3), 120.0).want_on, True)
        p.decide(inp(fresh=False, vesc=45.8, plug_on=True), 500.0)
        p.decide(inp(fresh=False, vesc=45.9, plug_on=True), 840.0)
        d = p.decide(inp(fresh=False, vesc=45.7), 120.0 + 3600)   # still low, but 0.4 V up
        self.assertIs(d.want_on, True)
        self.assertEqual(d.phase, "rescue")

    def test_rescue_and_wake_can_be_switched_off(self):
        p = ChargePolicy(cfg(rescue_enabled=False, wake_enabled=False), "hold")
        p.decide(inp(fresh=False), 0.0)
        for t in (120.0, 700.0, 4000.0):
            self.assertIs(p.decide(inp(fresh=False, vesc=45.0), t).want_on, False)

    def test_reading_returns_during_a_wake_pulse(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        p.decide(inp(fresh=False, vesc=47.5), 600.0)
        p.decide(inp(fresh=False, vesc=47.8, plug_on=True), 610.0)
        d = p.decide(inp(cell=3.66, a=3.6, plug_on=True), 650.0)      # the BMS woke up
        self.assertIs(d.want_on, True)
        self.assertEqual(d.info["session"]["kind"], "hold")
        t = 660.0
        while d.want_on and t < 3000.0:
            d = p.decide(inp(cell=3.72, a=3.6, plug_on=True), t)
            t += 10
        self.assertIn("top-up complete", d.reason)                      # 0.7 Ah default, no fault
        self.assertIsNone(p.fault)

    def test_action_stops_when_the_vesc_voltage_climbs_or_vanishes(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        p.decide(inp(fresh=False, vesc=48.0), 600.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=53.6, plug_on=True), 620.0).want_on, False)
        q = ChargePolicy(cfg(), "hold")
        q.decide(inp(fresh=False), 0.0)
        q.decide(inp(fresh=False, vesc=48.0), 600.0)
        self.assertIs(q.decide(inp(fresh=False, vesc=None, plug_on=True), 620.0).want_on, False)

    def test_armed_blocks_dark_actions(self):
        p = ChargePolicy(cfg(), "hold")
        p.decide(inp(fresh=False), 0.0)
        self.assertIs(p.decide(inp(fresh=False, vesc=46.0, armed=True), 700.0).want_on, False)

    def test_unreachable_plug_keeps_its_last_known_state(self):
        p = settled()
        p.decide(inp(cell=3.70), REST)
        p.decide(inp(cell=3.75, a=3.6, plug_on=True), REST + 10)
        d = p.decide(inp(cell=3.76, a=3.6, plug_on=None), REST + 20)   # plug silent, still charging
        self.assertIsNotNone(d.info["session"])
        p.note_plug(False)                                              # the manager switched it off
        d = p.decide(inp(cell=3.76, a=-0.3, plug_on=None), REST + 30)
        self.assertIsNone(d.info["session"])




class Agreement(unittest.TestCase):
    def test_bms_and_vesc_pack_voltage(self):
        p = settled()
        for i in range(2):
            d = p.decide(inp(cell=3.75, vesc=50.9), REST + 10 * i)      # BMS 48.8 V, VESC 50.9 V
            self.assertNotIn("voltage_mismatch", codes(d))
        d = p.decide(inp(cell=3.75, vesc=50.9), REST + 20)
        self.assertIn("voltage_mismatch", codes(d))
        self.assertIsNone(p.fault)
        d = p.decide(inp(cell=3.75, vesc=48.9), REST + 30)
        self.assertNotIn("voltage_mismatch", codes(d))


if __name__ == "__main__":
    unittest.main()
