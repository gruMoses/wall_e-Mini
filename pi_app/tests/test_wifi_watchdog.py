"""Tests for bin/wifi_watchdog.py, the Wi-Fi reconnect watchdog.

bin/ is not a package, so the script is loaded via importlib from its file
path (it is stdlib-only). The state machine runs against FakeNm on a fake
clock; FakeNm.up advances that clock by the time a real `nmcli --wait 150`
would take. The nmcli parsing runs against a fake command runner.

Each review finding of 2026-09-27 has a test here: a router outage must not
bounce a working link; NM's own activation must not be superseded before
the stuck net; candidates come from NM's band-aware available list; retry
spacing counts from the end of a try; the fail backoff expires; the dark
path re-activates a lone profile.
"""

import importlib.util
import os
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_MODULE_PATH = os.path.join(_REPO_ROOT, "bin", "wifi_watchdog.py")


def _load():
    spec = importlib.util.spec_from_file_location("wifi_watchdog", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolve string annotations through sys.modules.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ww = _load()


def setUpModule():
    ww.LOG.disabled = True


def tearDownModule():
    ww.LOG.disabled = False


PRIMARY = ww.Profile(name="preconfigured", uuid="u-primary", priority=0, ssid="WHITE ROCK ROAD")
FALLBACK = ww.Profile(name="wwr outdoor", uuid="u-fallback", priority=-10, ssid="wwr outdoor")
TICK = 15


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeNm:
    def __init__(self, clock, state=30, profiles=(PRIMARY, FALLBACK), available=None, seen=None,
                 up_ok=None, up_s_ok=8.0, up_s_fail=150.0, gateway="192.168.86.1",
                 gateway_answers=True, active="u-primary"):
        self.clock = clock
        self.state = state
        self._profiles = list(profiles)
        self.available = {p.uuid for p in profiles} if available is None else set(available)
        self.seen = {"WHITE ROCK ROAD": 60, "wwr outdoor": 55} if seen is None else seen
        self.up_ok = up_ok if up_ok is not None else (lambda uuid: True)
        self.up_s_ok, self.up_s_fail = up_s_ok, up_s_fail
        self.ups = []
        self.gw = gateway
        self.answers = gateway_answers
        self.active = active

    def device_state(self):
        return self.state

    def active_uuid(self):
        return self.active

    def available_uuids(self):
        return set(self.available)

    def profiles(self):
        return list(self._profiles)

    def visible(self):
        return dict(self.seen)

    def up(self, uuid):
        self.ups.append((self.clock.t, uuid))
        ok = self.up_ok(uuid)
        self.clock.t += self.up_s_ok if ok else self.up_s_fail
        if ok:
            self.state = 100
            self.active = uuid
        return ok, "" if ok else "Error: Timeout expired (150 seconds)"

    def gateway(self):
        return self.gw

    def gateway_answers(self, gw):
        return self.answers(self.clock.t) if callable(self.answers) else self.answers


def _run(nm, clock, until, dog=None, **cfg):
    dog = dog or ww.Watchdog(nm=nm, cfg=ww.WatchdogConfig(**cfg), clock=clock)
    while clock.t < until:
        dog.tick(clock.t)
        clock.t += TICK
    return dog


def _uuids(nm):
    return [u for _, u in nm.ups]


class TestSplitTerse(unittest.TestCase):

    def test_plain_and_escaped(self):
        self.assertEqual(ww.split_terse("WHITE ROCK ROAD:48"), ["WHITE ROCK ROAD", "48"])
        self.assertEqual(ww.split_terse("a\\:b:c"), ["a:b", "c"])
        self.assertEqual(ww.split_terse("back\\\\slash:1"), ["back\\slash", "1"])
        self.assertEqual(ww.split_terse(":153:52"), ["", "153", "52"])


class TestNmParsing(unittest.TestCase):

    def _nm(self, table):
        calls = []

        def runner(cmd, timeout):
            calls.append(cmd)
            key = " ".join(cmd)
            for prefix, result in table:
                if key.startswith(prefix):
                    return result
            return 1, ""

        nm = ww.Nm(runner=runner)
        nm.calls = calls
        return nm

    def test_device_state(self):
        self.assertEqual(self._nm([("nmcli -g GENERAL.STATE", (0, "100 (connected)\n"))]).device_state(), 100)
        self.assertIsNone(self._nm([]).device_state())

    def test_active_uuid_is_lowercased(self):
        nm = self._nm([("nmcli -g GENERAL.CON-UUID", (0, "2C24EA8A-5F37-4BA3-8DC7-17336CA4862D\n"))])
        self.assertEqual(nm.active_uuid(), "2c24ea8a-5f37-4ba3-8dc7-17336ca4862d")

    def test_available_uuids_one_or_several(self):
        one = self._nm([("nmcli -g CONNECTIONS.AVAILABLE-CONNECTIONS",
                         (0, "2c24ea8a-5f37-4ba3-8dc7-17336ca4862d | preconfigured\n"))])
        self.assertEqual(one.available_uuids(), {"2c24ea8a-5f37-4ba3-8dc7-17336ca4862d"})
        two = self._nm([("nmcli -g CONNECTIONS.AVAILABLE-CONNECTIONS", (0,
                         "2c24ea8a-5f37-4ba3-8dc7-17336ca4862d | preconfigured | "
                         "0fa0b1c2-d3e4-45f6-8789-0a1b2c3d4e5f | wwr outdoor\n"))])
        self.assertEqual(len(two.available_uuids()), 2)
        self.assertEqual(self._nm([]).available_uuids(), set())

    def test_profiles_keep_only_autoconnect_wifi(self):
        nm = self._nm([
            ("nmcli -t -f NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show", (0, "\n".join([
                "preconfigured:u1:802-11-wireless:yes:0",
                "wwr outdoor:u2:802-11-wireless:yes:-10",
                "manual only:u3:802-11-wireless:no:5",
                "Wired connection 1:u4:802-3-ethernet:yes:-999",
            ]))),
            ("nmcli -g 802-11-wireless.ssid connection show uuid u1", (0, "WHITE ROCK ROAD\n")),
            ("nmcli -g 802-11-wireless.ssid connection show uuid u2", (0, "wwr outdoor\n")),
        ])
        self.assertEqual([(p.name, p.priority, p.ssid) for p in nm.profiles()],
                         [("preconfigured", 0, "WHITE ROCK ROAD"), ("wwr outdoor", -10, "wwr outdoor")])

    def test_visible_keeps_best_signal_and_falls_back_to_the_cache(self):
        nm = self._nm([("nmcli -t -f SSID,SIGNAL device wifi list ifname wlan0 --rescan yes", (0, "\n".join([
            "WHITE ROCK ROAD:47", "WHITE ROCK ROAD:69", ":52", "wwr outdoor:55"])))])
        self.assertEqual(nm.visible(), {"WHITE ROCK ROAD": 69, "wwr outdoor": 55})
        cached = self._nm([
            ("nmcli -t -f SSID,SIGNAL device wifi list ifname wlan0 --rescan yes", (1, "refused")),
            ("nmcli -t -f SSID,SIGNAL device wifi list ifname wlan0 --rescan no", (0, "wwr outdoor:40")),
        ])
        self.assertEqual(cached.visible(), {"wwr outdoor": 40})

    def test_gateway_and_answers(self):
        nm = self._nm([("ip -4 route show default dev wlan0", (0, "default via 192.168.86.1 proto static metric 600\n"))])
        self.assertEqual(nm.gateway(), "192.168.86.1")
        self.assertTrue(self._nm([("ping", (0, ""))]).gateway_answers("g"))
        self.assertTrue(self._nm([("ping", (1, "")), ("ip neigh show", (0, "g lladdr 10:5a STALE"))]).gateway_answers("g"))
        for bad in ("FAILED", "INCOMPLETE"):
            self.assertFalse(self._nm([("ping", (1, "")), ("ip neigh show", (0, "g " + bad))]).gateway_answers("g"))

    def test_up_command(self):
        nm = self._nm([("nmcli --wait 150 connection up uuid u2 ifname wlan0", (0, "ok"))])
        self.assertTrue(nm.up("u2")[0])
        self.assertEqual(nm.calls[-1], ["nmcli", "--wait", "150", "connection", "up", "uuid", "u2", "ifname", "wlan0"])


class TestOffline(unittest.TestCase):

    def test_connected_with_a_live_gateway_never_acts(self):
        c = Clock(); nm = FakeNm(c, state=100)
        _run(nm, c, 3600)
        self.assertEqual(nm.ups, [])

    def test_field_case_nm_idle_after_the_block(self):
        # 2026-09-27: NM blocked the profile and sat in state 30. Act at +120 s.
        c = Clock(); nm = FakeNm(c, state=30)
        dog = _run(nm, c, 120)
        self.assertEqual(nm.ups, [])
        _run(nm, c, 136, dog=dog)
        self.assertEqual(_uuids(nm), ["u-primary"])
        self.assertEqual(nm.state, 100)

    def test_does_not_supersede_nm_activation_before_the_stuck_net(self):
        c = Clock(); nm = FakeNm(c, state=50)  # NM activating (config)
        dog = _run(nm, c, 599)
        self.assertEqual(nm.ups, [])
        _run(nm, c, 616, dog=dog)
        self.assertEqual(_uuids(nm), ["u-primary"])

    def test_idle_clock_restarts_when_nm_tries_again(self):
        c = Clock(); nm = FakeNm(c, state=30)
        dog = ww.Watchdog(nm=nm, cfg=ww.WatchdogConfig(), clock=c)
        for t in range(0, 105, TICK):
            c.t = t; dog.tick(t)
        nm.state = 50            # NM starts its own attempt at ~105 s
        c.t = 105; dog.tick(105)
        nm.state = 30            # and gives up
        for t in range(120, 225, TICK):
            c.t = t; dog.tick(t)
        self.assertEqual(nm.ups, [])  # 120 s of NEW idle not reached before 240

    def test_only_profiles_nm_lists_as_available(self):
        # 5 GHz-only primary: its SSID is visible on 2.4 GHz, but NM does not
        # list it as available, so it is never tried.
        c = Clock(); nm = FakeNm(c, state=30, available={"u-fallback"})
        _run(nm, c, 136)
        self.assertEqual(_uuids(nm), ["u-fallback"])

    def test_nothing_available_means_no_try(self):
        c = Clock(); nm = FakeNm(c, state=30, available=set())
        dog = _run(nm, c, 900)
        self.assertEqual(nm.ups, [])
        self.assertIsNotNone(dog.last_attempt_end)

    def test_retry_spacing_counts_from_the_end_of_a_try(self):
        c = Clock(); nm = FakeNm(c, state=30, profiles=(PRIMARY,), up_ok=lambda u: False)
        _run(nm, c, 1200)
        starts = [t for t, _ in nm.ups]
        self.assertGreaterEqual(len(starts), 3)
        for a, b in zip(starts, starts[1:]):
            self.assertGreaterEqual(b - (a + 150.0), 60.0 - 1e-9)

    def test_failed_primary_goes_behind_the_fallback_and_they_alternate(self):
        c = Clock(); nm = FakeNm(c, state=30, up_ok=lambda u: False)
        _run(nm, c, 2400)
        self.assertEqual(_uuids(nm)[:6], ["u-primary", "u-fallback"] * 3)

    def test_backoff_expires_and_priority_returns(self):
        c = Clock(); nm = FakeNm(c, state=30, up_ok=lambda u: u == "u-fallback")
        dog = _run(nm, c, 400)
        self.assertEqual(_uuids(nm), ["u-primary", "u-fallback"])
        # Later the fallback link drops; after the 600 s backoff the primary
        # is fresh again and goes first.
        c.t = 2000.0
        nm.state = 30
        _run(nm, c, 2200, dog=dog)
        self.assertEqual(_uuids(nm)[2], "u-primary")

    def test_equal_priority_prefers_the_stronger_signal(self):
        a = ww.Profile(name="a", uuid="ua", priority=0, ssid="A")
        b = ww.Profile(name="b", uuid="ub", priority=0, ssid="B")
        c = Clock(); nm = FakeNm(c, state=30, profiles=(a, b), seen={"A": 30, "B": 80})
        _run(nm, c, 136)
        self.assertEqual(_uuids(nm), ["ub"])


class TestDarkGateway(unittest.TestCase):

    def test_router_outage_never_bounces_a_working_link(self):
        # The gateway answered, then a 25-minute router outage.
        c = Clock(); nm = FakeNm(c, state=100, gateway_answers=lambda t: t < 300 or t > 1800)
        _run(nm, c, 2400)
        self.assertEqual(nm.ups, [])

    def test_never_answered_gateway_switches_profile(self):
        # Wrong subnet: associated on the fallback, the gateway never answers.
        c = Clock(); nm = FakeNm(c, state=100, gateway_answers=False, active="u-fallback")
        dog = _run(nm, c, 175)
        self.assertEqual(nm.ups, [])
        _run(nm, c, 240, dog=dog)
        self.assertEqual(_uuids(nm), ["u-primary"])

    def test_dark_path_reactivates_a_lone_profile(self):
        c = Clock(); nm = FakeNm(c, state=100, profiles=(PRIMARY,), gateway_answers=False)
        _run(nm, c, 240)
        self.assertEqual(_uuids(nm), ["u-primary"])

    def test_after_a_reactivation_the_dark_window_starts_again(self):
        c = Clock(); nm = FakeNm(c, state=100, profiles=(PRIMARY,), gateway_answers=False)
        _run(nm, c, 420)
        starts = [t for t, _ in nm.ups]
        self.assertEqual(len(starts), 2)
        self.assertGreaterEqual(starts[1] - starts[0], 180.0)

    def test_no_default_route_is_not_a_dark_gateway(self):
        c = Clock(); nm = FakeNm(c, state=100, gateway=None, gateway_answers=False)
        _run(nm, c, 900)
        self.assertEqual(nm.ups, [])


if __name__ == "__main__":
    unittest.main()
