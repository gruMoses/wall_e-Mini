"""Tests for the Kasa plug driver (pi_app/hardware/kasa_plug.py): cipher,
framing, the plug calls against a fake plug on 127.0.0.1, identity by MAC
and discovery by broadcast."""

import json
import socket
import struct
import threading
import unittest

from pi_app.hardware.kasa_plug import (KasaLegacyPlug, PlugError, decrypt, discover,
                                       discover_by_mac, encrypt, normalize_mac)

MAC = "58:D8:12:F1:ED:DE"


class Cipher(unittest.TestCase):
    def test_roundtrip_and_first_byte(self):
        msg = b'{"system":{"get_sysinfo":{}}}'
        enc = encrypt(msg)
        self.assertEqual(enc[0], 171 ^ ord("{"))   # 0xd0, the well-known first byte
        self.assertNotEqual(enc, msg)
        self.assertEqual(decrypt(enc), msg)

    def test_empty(self):
        self.assertEqual(encrypt(b""), b"")
        self.assertEqual(decrypt(b""), b"")

    def test_normalize_mac(self):
        self.assertEqual(normalize_mac("58:d8:12:f1:ed:de"), "58D812F1EDDE")
        self.assertEqual(normalize_mac("58-D8-12-F1-ED-DE"), "58D812F1EDDE")
        self.assertEqual(normalize_mac("58D812F1EDDE"), "58D812F1EDDE")
        for bad in ("", "58:D8", None, 12, "zz:zz:zz:zz:zz:zz"):
            self.assertEqual(normalize_mac(bad), "")


class _FakePlug:
    """Answers get_sysinfo / set_relay_state like an EP10 on a loopback port."""

    def __init__(self, relay=0, reply_override=None, mac=MAC):
        self.relay = relay
        self.mac = mac
        self.reply_override = reply_override
        self.requests = []
        self.rules = []              # the plug's countdown rules
        self.refuse_edit = False
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(4)
        self.port = self._srv.getsockname()[1]
        self._stop = False
        self._t = threading.Thread(target=self._serve, daemon=True)
        self._t.start()

    def close(self):
        self._stop = True
        try:
            self._srv.close()
        except OSError:
            pass

    def _serve(self):
        while not self._stop:
            try:
                conn, _ = self._srv.accept()
            except OSError:
                return
            with conn:
                try:
                    (n,) = struct.unpack(">I", conn.recv(4))
                    raw = b""
                    while len(raw) < n:
                        chunk = conn.recv(n - len(raw))
                        if not chunk:
                            break
                        raw += chunk
                    req = json.loads(decrypt(raw).decode())
                    self.requests.append(req)
                    if self.reply_override is not None:
                        reply = self.reply_override
                    elif "count_down" in req:
                        cd = req["count_down"]
                        if "delete_all_rules" in cd:
                            self.rules = []
                            reply = {"count_down": {"delete_all_rules": {"err_code": 0}}}
                        elif "add_rule" in cd:
                            if self.rules:
                                reply = {"count_down": {"add_rule": {"err_code": -10}}}
                            else:
                                self.rules = [dict(cd["add_rule"], id="RULE1", remain=cd["add_rule"]["delay"])]
                                reply = {"count_down": {"add_rule": {"id": "RULE1", "err_code": 0}}}
                        elif "edit_rule" in cd:
                            ok = (not self.refuse_edit and self.rules
                                  and cd["edit_rule"].get("id") == self.rules[0]["id"])
                            if ok:
                                self.rules = [dict(cd["edit_rule"], remain=cd["edit_rule"]["delay"])]
                            reply = {"count_down": {"edit_rule": {"err_code": 0 if ok else -14}}}
                        else:
                            reply = {"count_down": {"get_rules": {"rule_list": self.rules, "err_code": 0}}}
                    elif "get_sysinfo" in req.get("system", {}):
                        reply = {"system": {"get_sysinfo": {"model": "EP10(US)", "relay_state": self.relay,
                                                            "alias": "First plug", "mac": self.mac,
                                                            "err_code": 0}}}
                    else:
                        self.relay = int(req["system"]["set_relay_state"]["state"])
                        reply = {"system": {"set_relay_state": {"err_code": 0}}}
                    body = encrypt(json.dumps(reply).encode())
                    conn.sendall(struct.pack(">I", len(body)) + body)
                except Exception:
                    pass


class Driver(unittest.TestCase):
    def test_get_and_set_state(self):
        fake = _FakePlug(relay=0)
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC)
            self.assertIs(plug.get_state(), False)
            self.assertTrue(plug.set_state(True))
            self.assertIs(plug.get_state(), True)
            self.assertTrue(plug.set_state(False))
            self.assertIs(plug.get_state(), False)
            self.assertIn({"system": {"set_relay_state": {"state": 1}}}, fake.requests)
        finally:
            fake.close()

    def test_works_without_a_mac(self):
        fake = _FakePlug(relay=1)
        try:
            self.assertIs(KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0).get_state(), True)
        finally:
            fake.close()

    def test_wrong_device_is_never_switched(self):
        fake = _FakePlug(relay=0, mac="AA:BB:CC:DD:EE:FF")       # another Kasa plug took the address
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC,
                                  discover_fn=lambda *a, **k: None)
            with self.assertRaises(PlugError):
                plug.get_state()
            with self.assertRaises(PlugError):
                plug.set_state(True)
            self.assertEqual(fake.relay, 0)
            self.assertFalse(any("set_relay_state" in r.get("system", {}) for r in fake.requests))
        finally:
            fake.close()

    def test_error_code_raises(self):
        fake = _FakePlug(reply_override={"system": {"set_relay_state": {"err_code": -3}}})
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0)
            with self.assertRaises(PlugError):
                plug.set_state(True)
        finally:
            fake.close()

    def test_missing_relay_state_raises(self):
        fake = _FakePlug(reply_override={"system": {"get_sysinfo": {"err_code": 0}}})
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0)
            with self.assertRaises(PlugError):
                plug.get_state()
        finally:
            fake.close()

    def test_unreachable_raises_plug_error(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()   # nothing listens here now
        plug = KasaLegacyPlug("127.0.0.1", port, timeout_s=0.5)
        with self.assertRaises(PlugError):
            plug.get_state()

    def test_needs_a_host_or_a_mac(self):
        with self.assertRaises(ValueError):
            KasaLegacyPlug("")
        KasaLegacyPlug("", mac=MAC)                              # found by MAC on first use


class OffTimer(unittest.TestCase):
    """The dead-man countdown that lives in the plug."""

    def test_arm_rearm_in_place_and_clear(self):
        fake = _FakePlug(relay=1)
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC)
            self.assertTrue(plug.arm_off_timer(300))
            self.assertEqual(fake.rules[0]["delay"], 300)
            self.assertEqual(fake.rules[0]["act"], 0)            # act 0 = switch OFF
            self.assertEqual(plug.off_timer_remaining(), 300)
            n = len(fake.requests)
            self.assertTrue(plug.arm_off_timer(240))             # edit in place: no gap without a rule
            self.assertEqual(fake.rules[0]["delay"], 240)
            self.assertFalse(any("delete_all_rules" in r.get("count_down", {}) for r in fake.requests[n:]))
            self.assertTrue(plug.clear_off_timer())
            self.assertEqual(fake.rules, [])
            self.assertIsNone(plug.off_timer_remaining())
        finally:
            fake.close()

    def test_new_process_replaces_a_leftover_rule(self):
        fake = _FakePlug(relay=1)
        fake.rules = [{"id": "OLD", "enable": 1, "delay": 99, "act": 1, "name": "x", "remain": 50}]
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC)
            self.assertTrue(plug.arm_off_timer(300))
            self.assertEqual(len(fake.rules), 1)
            self.assertEqual(fake.rules[0]["act"], 0)
        finally:
            fake.close()

    def test_refused_edit_falls_back_to_a_new_rule(self):
        fake = _FakePlug(relay=1)
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC)
            plug.arm_off_timer(300)
            fake.refuse_edit = True
            self.assertTrue(plug.arm_off_timer(200))
            self.assertEqual(fake.rules[0]["delay"], 200)
        finally:
            fake.close()

    def test_wrong_device_gets_no_rule(self):
        fake = _FakePlug(relay=1, mac="AA:BB:CC:DD:EE:FF")
        try:
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0, mac=MAC,
                                  discover_fn=lambda *a, **k: None)
            with self.assertRaises(PlugError):
                plug.arm_off_timer(300)
            self.assertEqual(fake.rules, [])
        finally:
            fake.close()


class FindByMac(unittest.TestCase):
    """The address moves: the driver follows the MAC."""

    def _plug(self, hosts, found, clock):
        calls = []

        def discover_fn(mac, timeout_s=2.5, port=9999, targets=()):
            calls.append(mac)
            return found[0]

        changes = []
        plug = KasaLegacyPlug("10.0.0.1", mac=MAC, discover_fn=discover_fn,
                              on_host_change=changes.append, clock=clock)

        def query(request):
            info = hosts.get(plug.host)
            if info is None:
                raise PlugError(f"{plug.host}: no route")
            if "get_sysinfo" in request["system"]:
                return {"system": {"get_sysinfo": dict(info, err_code=0)}}
            info["relay_state"] = request["system"]["set_relay_state"]["state"]
            return {"system": {"set_relay_state": {"err_code": 0}}}

        plug.query = query
        return plug, calls, changes

    def test_moves_to_the_new_address(self):
        hosts = {"10.0.0.2": {"mac": MAC, "relay_state": 0}}
        t = [0.0]
        plug, calls, changes = self._plug(hosts, ["10.0.0.2"], lambda: t[0])
        self.assertIs(plug.get_state(), False)
        self.assertEqual(plug.host, "10.0.0.2")
        self.assertEqual(changes, ["10.0.0.2"])
        self.assertEqual(calls, ["58D812F1EDDE"])
        self.assertTrue(plug.set_state(True))
        self.assertEqual(hosts["10.0.0.2"]["relay_state"], 1)

    def test_also_when_a_stranger_answers_at_the_old_address(self):
        hosts = {"10.0.0.1": {"mac": "AA:BB:CC:DD:EE:FF", "relay_state": 0},
                 "10.0.0.2": {"mac": MAC, "relay_state": 1}}
        plug, _, _ = self._plug(hosts, ["10.0.0.2"], lambda: 0.0)
        self.assertIs(plug.get_state(), True)
        self.assertEqual(hosts["10.0.0.1"]["relay_state"], 0)

    def test_search_is_rate_limited_and_a_miss_keeps_the_error(self):
        t = [0.0]
        found = [None]
        plug, calls, changes = self._plug({}, found, lambda: t[0])
        with self.assertRaises(PlugError):
            plug.get_state()
        t[0] = 30.0
        with self.assertRaises(PlugError):
            plug.get_state()
        self.assertEqual(len(calls), 1)                          # one search per 60 s
        t[0] = 61.0
        with self.assertRaises(PlugError):
            plug.get_state()
        self.assertEqual(len(calls), 2)
        self.assertEqual(changes, [])
        self.assertEqual(plug.host, "10.0.0.1")

    def test_first_use_with_no_address(self):
        hosts = {"10.0.0.5": {"mac": MAC, "relay_state": 1}}
        calls = []
        plug = KasaLegacyPlug("", mac=MAC, discover_fn=lambda *a, **k: (calls.append(1), "10.0.0.5")[1])

        def query(request):
            info = hosts.get(plug.host)
            if info is None:
                raise PlugError("no route")
            return {"system": {"get_sysinfo": dict(info, err_code=0)}}

        orig = plug.query
        plug.query = lambda request: orig(request) if not plug.host else query(request)
        self.assertIs(plug.get_state(), True)
        self.assertEqual(plug.host, "10.0.0.5")


class Discovery(unittest.TestCase):
    def test_broadcast_reply_is_parsed_and_matched_by_mac(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        srv.bind(("127.0.0.1", 0))
        srv.settimeout(3.0)
        port = srv.getsockname()[1]

        def serve():
            try:
                data, addr = srv.recvfrom(4096)
                req = json.loads(decrypt(data).decode())
                assert "get_sysinfo" in req["system"]
                reply = {"system": {"get_sysinfo": {"mac": MAC, "alias": "First plug", "relay_state": 0}}}
                srv.sendto(encrypt(json.dumps(reply).encode()), addr)
            except Exception:
                pass

        th = threading.Thread(target=serve, daemon=True)
        th.start()
        try:
            found = discover(timeout_s=1.5, port=port, targets=("127.0.0.1",))
            self.assertEqual(normalize_mac(found["127.0.0.1"]["mac"]), "58D812F1EDDE")
        finally:
            th.join(timeout=3.0)
            srv.close()

    def test_quiet_lan_and_bad_mac(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        self.assertIsNone(discover_by_mac(MAC, timeout_s=0.6, port=port, targets=("127.0.0.1",)))
        self.assertIsNone(discover_by_mac("nonsense"))


if __name__ == "__main__":
    unittest.main()
