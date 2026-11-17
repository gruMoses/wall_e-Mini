"""Tests for the Kasa plug driver (pi_app/hardware/kasa_plug.py): cipher,
framing and the two plug calls against a fake plug on 127.0.0.1."""

import json
import socket
import struct
import threading
import unittest

from pi_app.hardware.kasa_plug import KasaLegacyPlug, PlugError, decrypt, encrypt


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


class _FakePlug:
    """Answers get_sysinfo / set_relay_state like an EP10 on a loopback port."""

    def __init__(self, relay=0, reply_override=None):
        self.relay = relay
        self.reply_override = reply_override
        self.requests = []
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
                    elif "get_sysinfo" in req.get("system", {}):
                        reply = {"system": {"get_sysinfo": {"model": "EP10(US)", "relay_state": self.relay,
                                                            "alias": "WALL-E Charger", "err_code": 0}}}
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
            plug = KasaLegacyPlug("127.0.0.1", fake.port, timeout_s=2.0)
            self.assertIs(plug.get_state(), False)
            self.assertTrue(plug.set_state(True))
            self.assertIs(plug.get_state(), True)
            self.assertTrue(plug.set_state(False))
            self.assertIs(plug.get_state(), False)
            self.assertEqual(fake.requests[1], {"system": {"set_relay_state": {"state": 1}}})
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

    def test_empty_host_rejected(self):
        with self.assertRaises(ValueError):
            KasaLegacyPlug("")


if __name__ == "__main__":
    unittest.main()
