"""Tests for ChargeManager (pi_app/control/charge_manager.py) with a fake
BMS and a fake plug; no threads, ``tick()`` is driven by hand."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from config import ChargerPlugConfig
from pi_app.control.charge_manager import ChargeManager


class FakeBms:
    def __init__(self):
        self.soc = 50.0
        self.v = 48.5
        self.a = 0.0
        self.connected = True
        self.link_state = "up"
        self.age = 5.0

    def get_state(self):
        return SimpleNamespace(soc_pct=self.soc, pack_voltage_v=self.v,
                               pack_current_a=self.a, connected=self.connected)

    def get_link_health(self):
        return SimpleNamespace(state=self.link_state, data_age_s=self.age, dark_for_s=None)


class FakePlug:
    def __init__(self, on=False):
        self.on = on
        self.fail_get = False
        self.fail_set = False
        self.sets = []

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


def make(tmp, mode_file=None, stopping=False, **kw):
    cfg = ChargerPlugConfig(host="192.0.2.9", **kw)
    path = Path(tmp) / "charger_state.json"
    if mode_file is not None:
        path.write_text(json.dumps({"mode": mode_file}))
    bms, plug = FakeBms(), FakePlug()
    mgr = ChargeManager(cfg, bms, plug, path, system_stopping=lambda: stopping,
                        clock=lambda: 0.0)
    return mgr, bms, plug, path


class Persistence(unittest.TestCase):
    def test_default_mode_and_file_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, path = make(tmp)
            self.assertEqual(mgr.get_status()["mode"], "hold")
            ok, _ = mgr.set_mode("full")
            self.assertTrue(ok)
            self.assertEqual(json.loads(path.read_text())["mode"], "full")
            mgr2, _, _, _ = make(tmp, mode_file="full")
            self.assertEqual(mgr2.get_status()["mode"], "full")

    def test_bad_file_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "charger_state.json"
            path.write_text("{not json")
            mgr, _, _, _ = make(tmp)
            self.assertEqual(mgr.get_status()["mode"], "hold")

    def test_set_mode_rejects_junk(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, _ = make(tmp)
            self.assertFalse(mgr.set_mode("warp")[0])
            self.assertFalse(mgr.set_mode(None)[0])
            self.assertFalse(mgr.set_mode(3)[0])


class Switching(unittest.TestCase):
    def test_hold_cycle_with_min_interval(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp, min_switch_interval_s=60.0)
            bms.soc = 44.0
            st = mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            self.assertEqual(st["switch_count"], 1)
            self.assertIs(st["plug_on"], True)
            # SOC jumps over the top of the band 5 s later: switch deferred.
            bms.soc = 56.0
            st = mgr.tick(now=5.0)
            self.assertTrue(plug.on)
            self.assertIn("switch in", st["reason"])
            # ...and done once the interval has passed.
            st = mgr.tick(now=61.0)
            self.assertFalse(plug.on)
            self.assertEqual(st["switch_count"], 2)

    def test_no_switch_when_plug_already_matches(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp)
            bms.soc = 100.0
            plug.on = False
            st = mgr.tick(now=0.0)
            self.assertEqual(plug.sets, [])
            self.assertEqual(st["switch_count"], 0)
            self.assertIs(st["want_on"], False)

    def test_operator_mode_change_applies_at_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp, min_switch_interval_s=600.0)
            bms.soc = 44.0
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            mgr.set_mode("off")
            st = mgr.tick(now=1.0)
            self.assertFalse(plug.on)
            self.assertEqual(st["mode"], "off")
            self.assertEqual(st["switch_count"], 2)

    def test_unreachable_plug_sets_error_and_never_switches(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp)
            bms.soc = 44.0
            plug.fail_get = True
            st = mgr.tick(now=0.0)
            self.assertEqual(plug.sets, [])
            self.assertIn("plug unreachable", st["reason"])
            self.assertIsNotNone(st["plug_error"])
            self.assertIsNone(st["plug_on"])

    def test_failed_switch_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp)
            bms.soc = 44.0
            plug.fail_set = True
            st = mgr.tick(now=0.0)
            self.assertIn("switch FAILED", st["reason"])
            self.assertIs(st["plug_on"], False)

    def test_dark_bms_switches_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp, soc_unknown_off_s=120.0)
            plug.on = True
            bms.link_state = "dark"
            mgr.tick(now=0.0)
            self.assertTrue(plug.on)
            mgr.tick(now=119.0)
            self.assertTrue(plug.on)
            st = mgr.tick(now=120.0)
            self.assertFalse(plug.on)
            self.assertFalse(st["bms_fresh"])

    def test_old_sample_is_not_fresh(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, _ = make(tmp, bms_max_age_s=30.0, soc_unknown_off_s=0.0)
            bms.soc = 44.0
            bms.age = 45.0
            st = mgr.tick(now=0.0)
            self.assertFalse(plug.on)
            self.assertFalse(st["bms_fresh"])

    def test_full_returns_to_hold_and_persists(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, bms, plug, path = make(tmp, mode_file="full")
            bms.soc = 70.0
            mgr.tick(now=0.0)
            bms.soc = 55.0
            st = mgr.tick(now=100.0)
            self.assertEqual(st["mode"], "hold")
            self.assertEqual(json.loads(path.read_text())["mode"], "hold")


class Stop(unittest.TestCase):
    def test_stop_switches_off_only_on_system_shutdown_in_auto_modes(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _ = make(tmp, stopping=True)
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [False])
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _ = make(tmp, stopping=False)   # a deploy restart
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [])
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, plug, _ = make(tmp, mode_file="on", stopping=True)
            plug.on = True
            mgr.stop()
            self.assertEqual(plug.sets, [])

    def test_stop_without_start_is_safe(self):
        with tempfile.TemporaryDirectory() as tmp:
            mgr, _, _, _ = make(tmp, stopping=False)
            mgr.stop()


if __name__ == "__main__":
    unittest.main()
