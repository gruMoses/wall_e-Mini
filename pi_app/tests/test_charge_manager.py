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

    def arm_off_timer(self, seconds):
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


def make(tmp, state=None, stopping=False, vesc=None, **kw):
    cfg = ChargerPlugConfig(**kw)
    path = Path(tmp) / "charger_state.json"
    if state is not None:
        path.write_text(json.dumps(state))
    bms, plug, wall = FakeBms(), FakePlug(), Wall()
    mgr = ChargeManager(cfg, bms, plug, path, system_stopping=lambda: stopping,
                        clock=lambda: 0.0, vesc_voltage_fn=(lambda: vesc), wall_clock=wall,
                        vesc_temp_fn=(lambda: 21.0))
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
