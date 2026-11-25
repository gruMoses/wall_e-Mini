"""Tests for ChargeManager (pi_app/control/charge_manager.py) with a fake
BMS and a fake plug; no threads, ``tick()`` is driven by hand."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from config import ChargerPlugConfig
from pi_app.control.charge_manager import ChargeManager

REST = 300.0


class FakeBms:
    def __init__(self):
        self.cell = 3.75
        self.spread = 30.0
        self.soc = 50.0
        self.a = -0.3
        self.temp = 24
        self.flags = []
        self.fet = True
        self.connected = True
        self.link_state = "up"
        self.age = 5.0

    def get_state(self):
        return SimpleNamespace(
            soc_pct=self.soc, pack_voltage_v=round(self.cell * 13, 1), pack_current_a=self.a,
            cell_min_mv=self.cell * 1000 - self.spread / 2, cell_max_mv=self.cell * 1000 + self.spread / 2,
            temp_min_c=self.temp, temp_max_c=self.temp, charge_fet_on=self.fet,
            error_flags=list(self.flags), num_cells=13, connected=self.connected)

    def get_link_health(self):
        return SimpleNamespace(state=self.link_state, data_age_s=self.age, dark_for_s=None)


class FakePlug:
    def __init__(self, on=False):
        self.on = on
        self.host = "192.0.2.9"
        self.fail_get = False
        self.fail_set = False
        self.sets = []
        self.cb = None
        self.timer = None            # seconds armed, None = no countdown
        self.arms = 0
        self.fail_timer = False
        self.on_arm = None           # called inside arm_off_timer (a call in flight)

    def arm_off_timer(self, seconds):
        if self.on_arm is not None:
            self.on_arm()
        if self.fail_timer:
            raise RuntimeError("countdown refused")
        self.timer = seconds
        self.arms += 1
        return True

    def clear_off_timer(self):
        self.timer = None
        return True

    def set_host_change_callback(self, fn):
        self.cb = fn

    def get_state(self):
        if self.fail_get:
            raise RuntimeError("no route to plug")
        return self.on

    def set_state(self, on):
        if self.fail_set:
            raise RuntimeError("plug refused")
        self.on = bool(on)
        self.sets.append(bool(on))
        return True


class Wall:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def make(tmp, state=None, stopping=False, vesc=None, inhibit=True, **kw):
    cfg = ChargerPlugConfig(**kw)
    path = Path(tmp) / "charger_state.json"
    if state is not None:
        path.write_text(json.dumps(state))
    bms, plug, wall = FakeBms(), FakePlug(), Wall()
    mgr = ChargeManager(cfg, bms, plug, path, system_stopping=lambda: stopping,
                        clock=lambda: 0.0, vesc_voltage_fn=(lambda: vesc), wall_clock=wall,
                        vesc_temp_fn=(lambda: 21.0), drive_inhibit_enabled=inhibit)
    return mgr, bms, plug, path, wall


def saved(path):
    return json.loads(path.read_text())


class Persistence(unittest.TestCase):
    def test_mode_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, path, _ = make(tmp)
            self.assertEqual(mgr.get_status()["mode"], "hold")
            self.assertTrue(mgr.set_mode("full")[0])
            self.assertEqual(saved(path)["mode"], "full")
            mgr2, _, _, _, _ = make(tmp)
            self.assertEqual(mgr2.get_status()["mode"], "full")

    def test_bad_file_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "charger_state.json").write_text("{not json")
            mgr, _, _, _, _ = make(tmp)
            self.assertEqual(mgr.get_status()["mode"], "hold")

    def test_set_mode_rejects_junk(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, _, _ = make(tmp)
            for junk in ("warp", None, 3):
                self.assertFalse(mgr.set_mode(junk)[0])

    def test_fault_survives_a_restart_and_clears_on_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, path, _ = make(tmp)
            plug.on = True
            bms.flags = ["Charge FET Failure"]
            st = mgr.tick(now=0.0)
            self.assertFalse(plug.on)                           # off on the first poll
            self.assertIsNone(st["fault"])                      # latched on the third
            mgr.tick(now=10.0)
            st = mgr.tick(now=20.0)
            self.assertEqual(st["fault"]["code"], "bms_flag")
            self.assertIsNotNone(st["fault"]["epoch"])
            self.assertFalse(plug.on)
            self.assertEqual(saved(path)["fault"]["code"], "bms_flag")
            # A new process (deploy restart): still latched, even in manual on.
            mgr2, bms2, plug2, _, _ = make(tmp)
            mgr2.set_mode("on")
            st = mgr2.tick(now=0.0)
            self.assertEqual(st["fault"]["code"], "bms_flag")
            self.assertEqual(plug2.sets, [])
            self.assertEqual(st["alerts"][0]["level"], "fault")
            ok, _ = mgr2.clear_fault()
            self.assertTrue(ok)
            self.assertIsNone(saved(path)["fault"])
            self.assertTrue(mgr2.tick(now=1.0)["plug_on"])
            self.assertFalse(mgr2.clear_fault()[0])

    def test_plug_address_found_by_mac_is_saved_and_reused(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, path, _ = make(tmp)
            plug.host = "192.0.2.77"
            plug.cb("192.0.2.77")
            self.assertEqual(saved(path)["plug_host"], "192.0.2.77")
            self.assertEqual(mgr.get_status()["host"], "192.0.2.77")
            mgr2, _, plug2, _, _ = make(tmp)
            self.assertEqual(plug2.host, "192.0.2.77")

    def test_last_full_charge_is_stamped_and_a_stale_one_is_flagged(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, path, wall = make(tmp, state={"mode": "full"}, full_done_hold_s=0.0)
            plug.on = True
            bms.cell, bms.spread, bms.a, bms.soc = 4.19, 8.0, 0.1, 100.0
            wall.t += 5000.0
            st = mgr.tick(now=0.0)
            self.assertFalse(plug.on)
            self.assertEqual(saved(path)["last_full_epoch"], wall.t)
            self.assertNotIn("full_due", [a["code"] for a in st["alerts"]])
        with tempfile.TemporaryDirectory() as tmp:
            old = Wall().t - 31 * 86400.0
            mgr, _, _, _, _ = make(tmp, state={"mode": "hold", "last_full_epoch": old})
            st = mgr.tick(now=0.0)
            self.assertIn("full_due", [a["code"] for a in st["alerts"]])
            self.assertAlmostEqual(st["days_since_full"], 31.0, places=1)


class Switching(unittest.TestCase):
    def test_hold_charges_on_rested_cell_voltage(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp)
            bms.cell = 3.69
            st = mgr.tick(now=0.0)
            self.assertFalse(plug.on)                       # settling first
            st = mgr.tick(now=REST)
            self.assertTrue(plug.on)
            self.assertEqual(st["switch_count"], 1)
            self.assertEqual(st["phase"], "charging")
            self.assertEqual(st["cell_spread_mv"], 30.0)

    def test_off_is_immediate_on_waits_for_the_interval(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"}, min_switch_interval_s=60.0)
            st = mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            mgr.note_armed(True)
            st = mgr.tick(now=5.0)
            self.assertFalse(plug.on)                       # off at once, 5 s after the last switch
            self.assertTrue(st["armed"])
            mgr.note_armed(False)
            st = mgr.tick(now=10.0)
            self.assertFalse(plug.on)
            self.assertIn("switch in", st["reason"])
            st = mgr.tick(now=66.0)
            self.assertTrue(plug.on)
            self.assertEqual(st["switch_count"], 3)

    def test_no_switch_when_plug_already_matches(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp)
            bms.cell, bms.spread = 4.19, 7.0
            st = mgr.tick(now=0.0)
            self.assertEqual(plug.sets, [])
            self.assertIs(st["want_on"], False)

    def test_operator_mode_change_applies_at_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"}, min_switch_interval_s=600.0)
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            mgr.set_mode("off")
            mgr.tick(now=1.0)
            self.assertFalse(plug.on)
            mgr.set_mode("on")
            st = mgr.tick(now=2.0)
            self.assertTrue(plug.on)                        # inside the interval: the operator wins
            self.assertEqual(st["switch_count"], 3)

    def test_unreachable_plug_is_reported_and_never_switched(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            plug.fail_get = True
            st = mgr.tick(now=0.0)
            self.assertEqual(plug.sets, [])
            self.assertIn("plug unreachable", st["reason"])
            self.assertIsNone(st["plug_on"])
            self.assertNotIn("plug_unreachable", [a["code"] for a in st["alerts"]])
            mgr.tick(now=10.0)
            st = mgr.tick(now=20.0)                         # third failure in a row
            self.assertIn("plug_unreachable", [a["code"] for a in st["alerts"]])

    def test_failed_switch_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            plug.fail_set = True
            st = mgr.tick(now=0.0)
            self.assertIn("switch FAILED", st["reason"])
            self.assertIs(st["plug_on"], False)

    def test_dark_bms_switches_off_then_pulses_on_the_vesc_voltage(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, vesc=48.0)
            plug.on = True
            bms.link_state = "dark"
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            st = mgr.tick(now=120.0)
            self.assertFalse(plug.on)
            self.assertFalse(st["bms_fresh"])
            self.assertEqual(st["vesc_v"], 48.0)
            self.assertEqual(st["vesc_temp_c"], 21.0)
            st = mgr.tick(now=600.0)
            self.assertTrue(plug.on)
            self.assertEqual(st["phase"], "wake")
            st = mgr.tick(now=785.0)
            self.assertFalse(plug.on)
            self.assertIsNone(st["fault"])

    def test_old_sample_is_not_fresh(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, bms_max_age_s=30.0)
            bms.age = 45.0
            st = mgr.tick(now=0.0)
            self.assertFalse(st["bms_fresh"])

    def test_inhibit_shows_as_an_alert(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            bms.temp = 3
            st = mgr.tick(now=0.0)
            self.assertFalse(plug.on)
            self.assertEqual(st["phase"], "inhibited")
            self.assertIn("inhibit", [a["code"] for a in st["alerts"]])
            self.assertIsNone(st["fault"])

    def test_status_is_json_serialisable(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, vesc=48.2)
            json.dumps(mgr.tick(now=0.0))
            json.dumps(mgr.get_status())


class DeadMan(unittest.TestCase):
    """The plug's own countdown ends a charge when the robot is gone."""

    def test_armed_while_charging_refreshed_and_removed(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            st = mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            self.assertEqual(plug.timer, 300.0)                 # armed in the same poll as the switch
            self.assertEqual(st["deadman"], "armed")
            mgr.tick(now=30.0)
            self.assertEqual(plug.arms, 1)
            mgr.tick(now=60.0)
            self.assertEqual(plug.arms, 2)                      # re-armed every 60 s
            mgr.set_mode("off")
            st = mgr.tick(now=70.0)
            self.assertFalse(plug.on)
            self.assertIsNone(plug.timer)
            self.assertEqual(st["deadman"], "off")

    def test_manual_on_has_no_countdown(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            mgr.tick(now=0.0)
            self.assertEqual(plug.timer, 300.0)
            mgr.set_mode("on")
            mgr.tick(now=5.0)
            self.assertTrue(plug.on)
            self.assertIsNone(plug.timer)                       # a leftover would end the manual charge
            mgr.tick(now=200.0)
            self.assertEqual(plug.arms, 1)

    def test_adopts_a_running_charge_after_a_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            plug.on = True
            bms.cell, bms.a = 3.9, 3.6
            mgr.tick(now=0.0)
            self.assertEqual(plug.sets, [])
            self.assertEqual(plug.timer, 300.0)

    def test_failure_is_flagged(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            plug.fail_timer = True
            for t in (0.0, 10.0):
                st = mgr.tick(now=t)
                self.assertNotIn("deadman_unarmed", [a["code"] for a in st["alerts"]])
            st = mgr.tick(now=20.0)
            self.assertIn("deadman_unarmed", [a["code"] for a in st["alerts"]])
            self.assertTrue(plug.on)                            # the charge goes on; the page says why it matters
            plug.fail_timer = False
            st = mgr.tick(now=30.0)
            self.assertNotIn("deadman_unarmed", [a["code"] for a in st["alerts"]])
            self.assertEqual(plug.timer, 300.0)

    def test_a_plug_without_a_countdown_still_works(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            plug.arm_off_timer = None
            st = mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            self.assertEqual(st["deadman"], "off")


class ModeAge(unittest.TestCase):
    def test_full_expiry_survives_a_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, path, wall = make(tmp)
            mgr.set_mode("full")
            self.assertEqual(saved(path)["mode_epoch"], wall.t)
            started = wall.t
            # 47 h later a new process: still full.
            state = saved(path)
            mgr2, bms2, plug2, _, wall2 = make(tmp, state=state)
            wall2.t = started + 47 * 3600.0
            mgr2b = ChargeManager(ChargerPlugConfig(), bms2, plug2, path, system_stopping=lambda: False,
                                  clock=lambda: 0.0, wall_clock=wall2, vesc_temp_fn=lambda: 21.0)
            bms2.cell, bms2.spread = 4.15, 8.0
            self.assertEqual(mgr2b.tick(now=0.0)["mode"], "full")
            st = mgr2b.tick(now=3600.0 + 1)                      # the 48th hour passes
            self.assertEqual(st["mode"], "hold")
            self.assertEqual(saved(path)["mode"], "hold")


def running_charge(test, tmp, mode="full"):
    """A manager with a charge running (charge current on 2 polls, disarmed).
    Returns (mgr, bms, plug, time of the last poll)."""
    mgr, bms, plug, _, _ = make(tmp, state={"mode": mode})
    bms.cell, bms.spread = (3.69, 30.0) if mode == "hold" else (3.90, 8.0)
    t = 0.0
    mgr.tick(now=t)
    if mode == "hold":
        t = REST                                   # hold starts on a rested reading
        mgr.tick(now=t)
    test.assertTrue(plug.on)
    test.assertFalse(mgr.holds_plug_on(t))         # no charge current seen yet
    bms.a = 3.6
    for _ in range(2):
        t += 10.0
        mgr.tick(now=t)
    test.assertTrue(mgr.holds_plug_on(t))
    return mgr, bms, plug, t


class ArmedChargeInhibit(unittest.TestCase):
    """Audit N1 (Kevin, 2026-11-25): a charge already running goes on at the
    arm, and holds_plug_on() keeps the charger inhibit on while the plug is on
    in it, also in the taper under the BMS inhibit's 2.0 A."""

    def test_arm_during_a_running_charge_keeps_the_plug_on_and_the_inhibit(self):
        for mode in ("hold", "full"):
            with tempfile.TemporaryDirectory() as tmp:
                mgr, bms, plug, t = running_charge(self, tmp, mode)
                arms0 = plug.arms
                mgr.note_armed(True)
                for _ in range(18):
                    t += 10.0
                    bms.cell += 0.001
                    st = mgr.tick(now=t)
                    self.assertTrue(plug.on, (mode, t))
                    self.assertTrue(mgr.holds_plug_on(t), (mode, t))
                self.assertTrue(st["armed"])
                self.assertTrue(st["holds_plug_on"])
                self.assertIn("robot armed, charge already running", st["reason"])
                self.assertEqual(plug.sets, [True], mode)              # never switched off
                # The dead-man is still re-armed every 60 s while armed.
                self.assertEqual(plug.arms - arms0, 3, mode)
                self.assertEqual(plug.timer, 300.0)
                self.assertEqual(st["deadman"], "armed")

    def test_taper_under_2_a_with_the_plug_on_and_armed_keeps_the_inhibit(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")
            mgr.note_armed(True)
            bms.cell = 4.15
            for a in (1.9, 1.4, 0.9, 0.6):                 # the BMS inhibit is released here
                bms.a = a
                t += 10.0
                mgr.tick(now=t)
                self.assertTrue(plug.on, a)
                self.assertTrue(mgr.holds_plug_on(t), a)
                self.assertTrue(mgr.holds_plug_on(t + 9.9), a)   # and between the polls

    def test_the_end_of_the_charge_while_armed_switches_off_and_releases(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "hold")
            mgr.note_armed(True)
            bms.cell = 3.84                                 # top of the band under charge
            t += 10.0
            st = mgr.tick(now=t)
            self.assertFalse(plug.on)
            self.assertFalse(mgr.holds_plug_on(t))          # released with the switch, not later
            self.assertFalse(st["holds_plug_on"])
            self.assertIn("top of the band", st["reason"])
            self.assertIsNone(plug.timer)                   # the dead-man removed with the plug off
            bms.cell, bms.a = 3.60, -0.3                    # low again: no new charge while armed
            for k in range(1, 10):
                st = mgr.tick(now=t + 600.0 * k)
                self.assertFalse(plug.on)
                self.assertEqual(st["phase"], "armed")
                self.assertFalse(mgr.holds_plug_on(t + 600.0 * k))
            self.assertEqual(plug.sets, [True, False])

    def test_arm_with_no_session_starts_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp)
            mgr.note_armed(True)
            bms.cell = 3.60
            for t in (0.0, REST, REST + 600.0):
                st = mgr.tick(now=t)
                self.assertFalse(mgr.holds_plug_on(t))
            self.assertEqual(plug.sets, [])
            self.assertEqual(st["phase"], "armed")

    def test_plug_on_with_the_lead_off_never_holds_the_inhibit(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp)
            bms.cell = 3.69
            mgr.tick(now=0.0)
            mgr.tick(now=REST)
            self.assertTrue(plug.on)
            for t in (REST + 10.0, REST + 20.0, REST + 30.0):    # no current: the lead is off the robot
                mgr.tick(now=t)
                self.assertFalse(mgr.holds_plug_on(t))
            mgr.note_armed(True)
            mgr.tick(now=REST + 40.0)
            self.assertFalse(plug.on)                       # off at the arm, as before
            self.assertFalse(mgr.holds_plug_on(REST + 40.0))

    def test_disarm_changes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "hold")
            mgr.note_armed(True)
            mgr.tick(now=t + 10.0)
            mgr.note_armed(False)
            st = mgr.tick(now=t + 20.0)
            self.assertTrue(plug.on)
            self.assertTrue(mgr.holds_plug_on(t + 20.0))
            self.assertNotIn("robot armed", st["reason"])
            bms.cell = 3.84                                 # the end, disarmed: as before
            mgr.tick(now=t + 30.0)
            self.assertFalse(plug.on)
            self.assertFalse(mgr.holds_plug_on(t + 30.0))

    def test_an_unreachable_plug_or_a_hung_thread_releases_after_the_deadman(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")
            mgr.note_armed(True)
            seen = t
            plug.fail_get = True
            # 300 s countdown + 60 s refresh: one re-arm in flight cannot outlast it.
            for dt in (10.0, 100.0, 299.0, 359.0):
                mgr.tick(now=seen + dt)
                self.assertTrue(mgr.holds_plug_on(seen + dt), dt)    # it may still be on
            st = mgr.tick(now=seen + 360.0)
            self.assertFalse(mgr.holds_plug_on(seen + 360.0))        # its own countdown has ended it
            self.assertEqual(st["phase"], "armed")                   # and the policy no longer keeps it
            plug.fail_get = False
            plug.on = False                                          # the countdown switched it off
            mgr.tick(now=seen + 370.0)
            self.assertFalse(mgr.holds_plug_on(seen + 370.0))
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")
            self.assertTrue(mgr.holds_plug_on(t + 359.0))             # no tick at all (thread hung)
            self.assertFalse(mgr.holds_plug_on(t + 360.0))

    def test_a_charge_that_becomes_running_with_no_plug_answer_seeds_the_term(self):
        """Grok review of e920479 (must-fix): the plug goes on, then stops
        answering; the BMS shows a taper current (1.0 to 2.0 A, under the BMS
        inhibit) on two disarmed polls. The term is seeded on that poll, is
        never refreshed by a failed read, and the charge is kept while armed
        only while it holds."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            bms.cell, bms.spread = 4.10, 8.0
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)                             # switched on, countdown armed at 0
            plug.fail_get = True
            bms.a = 1.5
            mgr.tick(now=10.0)
            self.assertFalse(mgr.holds_plug_on(10.0))            # one poll of current: not running yet
            st = mgr.tick(now=20.0)                              # running, the plug did not answer
            self.assertTrue(mgr.holds_plug_on(20.0))
            self.assertTrue(st["holds_plug_on"])
            mgr.note_armed(True)
            st = mgr.tick(now=30.0)
            self.assertIs(st["want_on"], True)                   # kept: the term holds the robot
            self.assertTrue(mgr.holds_plug_on(30.0))
            for t in (100.0, 200.0, 300.0):                      # failed reads do not refresh it
                mgr.tick(now=t)
            self.assertTrue(mgr.holds_plug_on(379.0))
            self.assertFalse(mgr.holds_plug_on(380.0))           # seed + 360 s
            st = mgr.tick(now=390.0)
            self.assertIs(st["want_on"], False)                  # no longer kept while armed
            self.assertEqual(st["phase"], "armed")
            self.assertFalse(mgr.holds_plug_on(390.0))
            self.assertEqual(plug.arms, 1)                       # the countdown from t=0 was never pushed out

    def test_the_term_is_stored_before_a_blocking_countdown_rearm(self):
        """Grok should-fix 1: arm_off_timer can block for the plug timeout and
        restarts the plug's 5 min countdown; the term must already cover it."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")      # countdown armed at 0, last poll 20
            self.assertEqual(t, 20.0)
            probe = []
            late = 80.0                  # a late poll (slow plug calls delay the thread)
            plug.on_arm = lambda: probe.append((mgr.holds_plug_on(late + 303.0),
                                                mgr.holds_plug_on(late + 360.0)))
            mgr.tick(now=late)                                         # re-arm due here
            # While the re-arm is in flight, a 3 s call + the new 300 s
            # countdown is already covered (the hold of the poll at 20 s ends at 380 s).
            self.assertEqual(probe, [(True, False)])

    def test_a_failed_off_does_not_push_the_countdown_out(self):
        """Grok should-fix 2 (also on main before this change): when the OFF
        switch fails, the countdown already in the plug must end the charge."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "hold")      # countdown armed at REST
            mgr.note_armed(True)
            arms0 = plug.arms
            plug.fail_set = True
            bms.cell = 3.84                                            # the policy wants it off
            for k in range(1, 30):
                st = mgr.tick(now=t + 10.0 * k)
                self.assertTrue(plug.on)
                self.assertIs(st["want_on"], False)
                self.assertTrue(mgr.holds_plug_on(t + 10.0 * k))       # powered: the robot stays held
            self.assertEqual(plug.arms, arms0)                         # never re-armed in 290 s
            self.assertEqual(plug.timer, 300.0)                        # the running countdown is left alone
            plug.on = False                                            # the countdown switched it off
            mgr.tick(now=t + 300.0)
            self.assertFalse(mgr.holds_plug_on(t + 300.0))

    def test_a_failed_off_after_the_term_ran_out_never_requalifies_the_charge(self):
        """Grok re-review of 57bbfc1 (must-fix): the term runs out with the
        plug silent, the plug answers "on" again and the OFF fails. The hold
        written on that poll keeps the robot held (the plug is on) but must
        not keep the charge or re-arm the countdown: it stays OFF until the
        plug is seen off. Run well past 360 s."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")      # term until 20 + 360
            mgr.note_armed(True)
            plug.fail_get = True
            for k in range(1, 37):                                     # silent plug, 30 .. 380 s
                st = mgr.tick(now=t + 10.0 * k)
            self.assertFalse(mgr.holds_plug_on(380.0))
            self.assertIs(st["want_on"], False)                        # the term ran out: stop
            plug.fail_get = False
            plug.fail_set = True                                       # the plug is still on, OFF fails
            arms0 = plug.arms
            for k in range(1, 90):                                     # 390 .. 1270 s
                now = 380.0 + 10.0 * k
                st = mgr.tick(now=now)
                self.assertTrue(plug.on, now)
                self.assertIs(st["want_on"], False, now)               # never kept again
                self.assertTrue(mgr.holds_plug_on(now), now)           # powered: the robot stays held
            self.assertEqual(plug.arms, arms0)                         # the countdown never pushed out
            self.assertIn("waiting for the plug to go off", st["reason"])
            plug.fail_set = False
            mgr.tick(now=1280.0)
            self.assertFalse(plug.on)
            self.assertFalse(mgr.holds_plug_on(1280.0))
            st = mgr.tick(now=1290.0)                                  # seen off: the session is over
            self.assertIsNone(st["session"])
            self.assertEqual(st["phase"], "armed")                     # and no new charge while armed

    def test_an_expired_term_is_cleared_and_a_new_charge_seeds_it_again(self):
        """Grok re-review should-fix 2: a hold that ran out on failed reads is
        cleared ("None" means not holding), so it can never block a later
        seed. (Since 8751e9f's follow-up every charge kind counts, so the
        running edge comes only with a new session.)"""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"})
            bms.cell, bms.spread = 4.10, 8.0
            mgr.tick(now=0.0)
            plug.fail_get = True
            bms.a = 1.5
            mgr.tick(now=10.0)
            mgr.tick(now=20.0)                                         # seeded, until 380
            for t in range(30, 400, 10):
                mgr.tick(now=float(t))
            self.assertFalse(mgr.holds_plug_on(390.0))
            self.assertIsNone(mgr._plug_hold_until)                    # cleared, not left stale
            plug.fail_get = False
            plug.on = False                                            # the countdown switched it off
            mgr.tick(now=400.0)                                        # seen off; full starts again
            self.assertTrue(plug.on)
            plug.fail_get = True
            mgr.tick(now=410.0)
            mgr.tick(now=420.0)                                        # running again, plug silent
            self.assertTrue(mgr.holds_plug_on(420.0))
            self.assertTrue(mgr.holds_plug_on(779.0))
            self.assertFalse(mgr.holds_plug_on(780.0))

    def test_the_term_and_the_countdown_use_one_refresh_default(self):
        """Grok re-review should-fix 3: a config without plug_deadman_refresh_s
        gives 60 s to both, so the term still outlasts the countdown."""
        class NoRefresh:
            def __init__(self, cfg):
                self._c = cfg

            def __getattr__(self, name):
                if name == "plug_deadman_refresh_s":
                    raise AttributeError(name)
                return getattr(self._c, name)

        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "full")      # last poll at 20
            mgr._cfg = NoRefresh(mgr._cfg)
            mgr.tick(now=30.0)
            self.assertTrue(mgr.holds_plug_on(30.0 + 359.0))
            self.assertFalse(mgr.holds_plug_on(30.0 + 360.0))          # 300 + 60, not 300 + 0
            arms0 = plug.arms
            mgr.tick(now=59.0)
            self.assertEqual(plug.arms, arms0)
            mgr.tick(now=60.0)                                         # re-armed after 60 s
            self.assertEqual(plug.arms, arms0 + 1)

    def test_charger_inhibit_switched_off_in_config_stops_the_charge_at_the_arm(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "full"}, inhibit=False)
            bms.cell, bms.spread = 3.90, 8.0
            mgr.tick(now=0.0)
            bms.a = 3.6
            mgr.tick(now=10.0)
            mgr.tick(now=20.0)
            self.assertFalse(mgr.holds_plug_on(20.0))
            mgr.note_armed(True)
            mgr.tick(now=30.0)
            self.assertFalse(plug.on)                                  # as before this change

    def test_plug_on_and_plug_off_after_an_armed_off_keep_the_robot_held(self):
        """Grok re-review of 8751e9f (must-fix): PLUG OFF and PLUG ON rebuild
        the session as "manual". After an armed OFF with the plug still
        reading on, neither frees the robot, PLUG ON does not switch the
        charger back on, and neither removes or re-arms the countdown."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "hold")      # countdown armed at REST
            mgr.note_armed(True)
            plug.fail_set = True
            bms.a = 1.2                                                # taper: the BMS term is off
            bms.cell = 3.84                                            # top of the band: OFF, it fails
            st = mgr.tick(now=t + 10.0)
            self.assertIs(st["want_on"], False)
            self.assertTrue(plug.on)
            arms0, timer0 = plug.arms, plug.timer
            self.assertEqual(timer0, 300.0)
            for k, mode in enumerate(("off", "on", "full", "on", "off"), start=2):
                mgr.set_mode(mode)
                now = t + 10.0 * k
                st = mgr.tick(now=now)
                self.assertIs(st["want_on"], False, mode)              # never back on
                self.assertTrue(plug.on, mode)
                self.assertTrue(mgr.holds_plug_on(now), mode)          # the robot stays held
                self.assertEqual((plug.arms, plug.timer), (arms0, timer0), mode)   # countdown left alone
            plug.fail_set = False
            mgr.set_mode("on")
            st = mgr.tick(now=t + 100.0)
            self.assertFalse(plug.on)                                  # the OFF goes through
            self.assertFalse(mgr.holds_plug_on(t + 100.0))
            st = mgr.tick(now=t + 110.0)                               # seen off: PLUG ON, armed
            self.assertFalse(plug.on)                                  # no new charge while armed
            self.assertEqual(st["phase"], "armed")
            mgr.note_armed(False)
            mgr.tick(now=t + 200.0)
            self.assertTrue(plug.on)                                   # disarmed: manual is manual
            self.assertIsNone(plug.timer)                              # and has no countdown

    def test_a_plug_on_charge_running_at_the_arm_keeps_going_with_a_countdown(self):
        """PLUG ON follows the same armed rule as hold and full: a charge that
        runs at the arm goes on and holds the robot; while armed it has the
        countdown, so the plug term may run out with it; nothing starts."""
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "on"})
            bms.cell, bms.spread = 3.90, 8.0
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            self.assertIsNone(plug.timer)                              # manual: no countdown
            bms.a = 3.6
            mgr.tick(now=10.0)
            mgr.tick(now=20.0)
            self.assertTrue(mgr.holds_plug_on(20.0))
            mgr.note_armed(True)
            st = mgr.tick(now=30.0)
            self.assertTrue(plug.on)
            self.assertIs(st["want_on"], True)
            self.assertTrue(mgr.holds_plug_on(30.0))
            self.assertEqual(plug.timer, 300.0)                        # countdown while armed
            arms0 = plug.arms
            mgr.tick(now=90.0)
            self.assertEqual(plug.arms, arms0 + 1)                     # refreshed every 60 s
            mgr.note_armed(False)
            mgr.tick(now=100.0)
            self.assertTrue(plug.on)
            self.assertIsNone(plug.timer)                              # disarmed: removed again

    def test_plug_on_with_no_charge_goes_off_at_the_arm_and_starts_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _, _ = make(tmp, state={"mode": "on"})
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)                                   # no current: the lead is off
            mgr.note_armed(True)
            st = mgr.tick(now=10.0)
            self.assertFalse(plug.on)
            self.assertEqual(st["phase"], "armed")
            for t in (20.0, 100.0, 1000.0):
                mgr.tick(now=t)
                self.assertFalse(plug.on)
            mgr.note_armed(False)
            mgr.tick(now=1010.0)
            self.assertTrue(plug.on)


class ArmedChargeReleaseRamps(unittest.TestCase):
    """The end of a charge while armed releases the inhibit through the slew
    ramp of cce9b06: a held stick does not reach the motors in one tick. The
    inhibit is fed as main.py feeds it (BMS term or plug term)."""

    def test_release_after_the_charge_ends_ramps_from_neutral(self):
        from unittest.mock import patch
        from config import config as default_config
        from pi_app.control.controller import Controller, RCInputs
        from pi_app.control.mapping import CENTER_OUTPUT_VALUE as C
        from pi_app.control.safety import SafetyParams

        class Motor:
            def set_tracks(self, left_byte, right_byte):
                pass

            def stop(self):
                pass

            def get_telemetry(self):
                return SimpleNamespace(left_rpm=0, right_rpm=0, pack_low_latched=False)

        class Relay:
            def set_armed(self, armed):
                pass

        class Shutdown:
            def schedule_shutdown(self, delay_seconds):
                pass

        self.assertTrue(default_config.slew_limiter.snap_first_command)    # the production config
        step = int(round(float(default_config.slew_limiter.manual_accel_bps) * 0.1))
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, t = running_charge(self, tmp, "hold")
            bms.a = 1.5                                     # under 2.0 A: the BMS term is off
            clock = {"t": 10.0}
            with patch("pi_app.control.controller.time.monotonic", side_effect=lambda: clock["t"]):
                c = Controller(motor_driver=Motor(), arm_relay=Relay(), shutdown_scheduler=Shutdown(),
                               safety_params=SafetyParams(debounce_seconds=0.0, arm_high_persist_s=0.0))

                def tick(stick):
                    clock["t"] += 0.1
                    plug_term = mgr.holds_plug_on(t)
                    c.set_charger_inhibit(False or plug_term, plug_charge=plug_term)
                    cmd, _, telem = c.process(
                        RCInputs(ch1_us=stick, ch2_us=stick, ch3_us=1900, ch4_us=1000, ch5_us=1000,
                                 last_update_mono_s=clock["t"]), now_epoch_s=clock["t"])
                    self.assertTrue(cmd.is_armed)
                    return (cmd.left_byte, cmd.right_byte), telem["charger_inhibit"]

                tick(1500)                                  # armed at rest during the charge
                mgr.note_armed(True)
                t += 10.0
                mgr.tick(now=t)
                engaged = [tick(2000) for _ in range(3)]    # stick full forward
                bms.cell = 3.84                             # the charge ends
                t += 10.0
                mgr.tick(now=t)
                self.assertFalse(plug.on)
                released = [tick(2000) for _ in range(3)]
        for out, inhibit in engaged:
            self.assertEqual(out, (C, C))
            self.assertTrue(inhibit)
        for k, (out, inhibit) in enumerate(released, start=1):
            self.assertFalse(inhibit)
            self.assertEqual(out, (C + k * step, C + k * step), released)


class Stop(unittest.TestCase):
    def test_stop_switches_off_only_on_system_shutdown_in_auto_modes(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _, _ = make(tmp, stopping=True)
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [False])
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _, _ = make(tmp, stopping=False)   # a deploy restart
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [])
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _, _ = make(tmp, state={"mode": "on"}, stopping=True)
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [])

    def test_stop_without_start_is_safe(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, _, _ = make(tmp, stopping=False)
            mgr.stop()


if __name__ == "__main__":
    unittest.main()
