"""TP-Link Kasa smart plug over the local network (the charger plug).

Speaks the original Kasa protocol: JSON obscured with the XOR autokey cipher
(seed 171). Over TCP port 9999 each frame carries a 4-byte big-endian length;
over UDP port 9999 (discovery) it does not. No cloud, no credentials, no
third-party library. The EP10 on WALL-E's charger answers this protocol
(docs/charger_plug.md).

Identity: the plug is known by its MAC address. The address in the config is
only a first guess. Every call reads the plug's sysinfo first and compares the
MAC; a wrong or silent device makes the driver look for the MAC with a UDP
broadcast and move to the address that answers (Kevin, 2026-11-16: find it by
MAC, no DHCP reservation). A relay command is never sent to a device whose
MAC was not just verified, so a second Kasa plug that inherits the old
address cannot be switched by mistake.

A failure raises ``PlugError``; the caller (ChargeManager) treats that as
"plug unknown" and never as a reason to drive anything.
"""

from __future__ import annotations

import json
import socket
import struct
import time
from typing import Any, Callable, Dict, Iterable, Optional

_XOR_SEED = 171


class PlugError(RuntimeError):
    """The plug did not answer, or answered something unexpected."""


def encrypt(plain: bytes) -> bytes:
    key = _XOR_SEED
    out = bytearray()
    for b in plain:
        key = key ^ b
        out.append(key)
    return bytes(out)


def decrypt(data: bytes) -> bytes:
    key = _XOR_SEED
    out = bytearray()
    for b in data:
        out.append(key ^ b)
        key = b
    return bytes(out)


def normalize_mac(mac: Any) -> str:
    """'58:d8:12-f1.ed de' -> '58D812F1EDDE'; anything else -> ''."""
    if not isinstance(mac, str):
        return ""
    hexed = "".join(c for c in mac if c in "0123456789abcdefABCDEF").upper()
    return hexed if len(hexed) == 12 else ""


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise PlugError("connection closed mid-frame")
        buf.extend(chunk)
    return bytes(buf)


_SYSINFO_REQ = b'{"system":{"get_sysinfo":{}}}'


def discover(timeout_s: float = 2.5, port: int = 9999,
             targets: Iterable[str] = ("255.255.255.255",)) -> Dict[str, Dict[str, Any]]:
    """Broadcast a sysinfo query; return {address: sysinfo} for every Kasa
    device that answers within ``timeout_s``. Never raises on a quiet LAN."""
    found: Dict[str, Dict[str, Any]] = {}
    payload = encrypt(_SYSINFO_REQ)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError:
        return found
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.25)
        deadline = time.monotonic() + max(0.5, float(timeout_s))
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                for tgt in targets:
                    try:
                        sock.sendto(payload, (tgt, int(port)))
                    except OSError:
                        pass
                next_send = now + 0.8
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                info = json.loads(decrypt(data).decode("utf-8"))["system"]["get_sysinfo"]
            except Exception:
                continue
            if isinstance(info, dict):
                found[addr[0]] = info
    finally:
        try:
            sock.close()
        except OSError:
            pass
    return found


def discover_by_mac(mac: str, timeout_s: float = 2.5, port: int = 9999,
                    targets: Iterable[str] = ("255.255.255.255",)) -> Optional[str]:
    """Address of the Kasa device with this MAC, or None."""
    want = normalize_mac(mac)
    if not want:
        return None
    for addr, info in discover(timeout_s, port, targets).items():
        if normalize_mac(info.get("mac") or info.get("mic_mac")) == want:
            return addr
    return None


class KasaLegacyPlug:
    """One Kasa plug, identified by MAC when one is given."""

    def __init__(self, host: str, port: int = 9999, timeout_s: float = 3.0,
                 mac: str = "", discovery_targets: Iterable[str] = ("255.255.255.255",),
                 rediscover_min_interval_s: float = 60.0,
                 on_host_change: Optional[Callable[[str], None]] = None,
                 discover_fn: Optional[Callable[..., Optional[str]]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.mac = normalize_mac(mac)
        if not host and not self.mac:
            raise ValueError("Kasa plug needs a host or a MAC")
        self.host = host or ""
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._targets = tuple(discovery_targets)
        self._rediscover_min_interval_s = float(rediscover_min_interval_s)
        self._on_host_change = on_host_change
        self._discover_fn = discover_fn or discover_by_mac
        self._clock = clock
        self._last_discovery_t: Optional[float] = None
        self._timer_rule_id: Optional[str] = None

    def set_host_change_callback(self, fn: Optional[Callable[[str], None]]) -> None:
        self._on_host_change = fn

    # -- transport --------------------------------------------------------

    def query(self, request: Dict[str, Any]) -> Dict[str, Any]:
        if not self.host:
            raise PlugError("no address for the plug yet")
        payload = encrypt(json.dumps(request, separators=(",", ":")).encode("utf-8"))
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
                sock.settimeout(self.timeout_s)
                sock.sendall(struct.pack(">I", len(payload)) + payload)
                (length,) = struct.unpack(">I", _recv_exact(sock, 4))
                if length <= 0 or length > 1_000_000:
                    raise PlugError(f"bad frame length {length}")
                body = decrypt(_recv_exact(sock, length))
        except PlugError:
            raise
        except (OSError, struct.error) as e:
            raise PlugError(f"{self.host}:{self.port}: {e}") from e
        try:
            obj = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise PlugError(f"{self.host}: undecodable reply") from e
        if not isinstance(obj, dict):
            raise PlugError(f"{self.host}: reply is not an object")
        return obj

    def _sysinfo_once(self) -> Dict[str, Any]:
        reply = self.query({"system": {"get_sysinfo": {}}})
        info = reply.get("system", {}).get("get_sysinfo")
        if not isinstance(info, dict):
            raise PlugError(f"{self.host}: no get_sysinfo in reply")
        if info.get("err_code", 0) != 0:
            raise PlugError(f"{self.host}: get_sysinfo err_code {info.get('err_code')}")
        if self.mac:
            got = normalize_mac(info.get("mac") or info.get("mic_mac"))
            if got != self.mac:
                raise PlugError(f"{self.host}: wrong device (MAC {got or 'none'})")
        return info

    # -- identity ---------------------------------------------------------

    def _rediscover(self) -> bool:
        """Look for the MAC on the LAN; True when the address changed."""
        if not self.mac:
            return False
        now = self._clock()
        if (self._last_discovery_t is not None
                and now - self._last_discovery_t < self._rediscover_min_interval_s):
            return False
        self._last_discovery_t = now
        try:
            addr = self._discover_fn(self.mac, timeout_s=2.5, port=self.port, targets=self._targets)
        except Exception:
            addr = None
        if not addr or addr == self.host:
            return False
        self.host = addr
        if self._on_host_change is not None:
            try:
                self._on_host_change(addr)
            except Exception:
                pass
        return True

    def get_sysinfo(self) -> Dict[str, Any]:
        """Sysinfo of the verified plug; moves to a new address when the MAC
        is found elsewhere."""
        try:
            return self._sysinfo_once()
        except PlugError as first:
            if self._rediscover():
                return self._sysinfo_once()
            raise first

    # -- plug API ---------------------------------------------------------

    def get_state(self) -> Optional[bool]:
        """True = relay closed (charger powered), False = open."""
        info = self.get_sysinfo()
        relay = info.get("relay_state")
        if relay is None:
            # Multi-outlet strips report children; the EP10 never does.
            raise PlugError(f"{self.host}: no relay_state in sysinfo")
        return bool(int(relay))

    def set_state(self, on: bool) -> bool:
        # Identity first: never switch a device whose MAC was not just read.
        self.get_sysinfo()
        reply = self.query({"system": {"set_relay_state": {"state": 1 if on else 0}}})
        res = reply.get("system", {}).get("set_relay_state")
        if not isinstance(res, dict):
            raise PlugError(f"{self.host}: no set_relay_state in reply")
        if res.get("err_code", 1) != 0:
            raise PlugError(f"{self.host}: set_relay_state err_code {res.get('err_code')}")
        return True

    # -- dead-man timer (the plug's own countdown rule) ---------------------

    _TIMER_NAME = "walle deadman"

    def arm_off_timer(self, seconds: float) -> bool:
        """The plug switches itself OFF after ``seconds`` unless this is
        called again. The countdown runs in the plug, so it still fires when
        the robot or the Wi-Fi link is gone. One rule exists at a time; a
        timer set in the Kasa app is replaced."""
        delay = max(10, int(seconds))
        self.get_sysinfo()                      # identity first
        rule = {"enable": 1, "delay": delay, "act": 0, "name": self._TIMER_NAME}
        if self._timer_rule_id:
            # Re-arm in place: no moment without a rule.
            try:
                reply = self.query({"count_down": {"edit_rule": dict(rule, id=self._timer_rule_id)}})
                if reply.get("count_down", {}).get("edit_rule", {}).get("err_code", 1) == 0:
                    return True
            except PlugError:
                raise
            self._timer_rule_id = None
        self.query({"count_down": {"delete_all_rules": {}}})
        reply = self.query({"count_down": {"add_rule": rule}})
        res = reply.get("count_down", {}).get("add_rule")
        if not isinstance(res, dict) or res.get("err_code", 1) != 0:
            raise PlugError(f"{self.host}: countdown rule refused ({res})")
        self._timer_rule_id = res.get("id") or None
        return True

    def clear_off_timer(self) -> bool:
        self.get_sysinfo()
        self._timer_rule_id = None
        reply = self.query({"count_down": {"delete_all_rules": {}}})
        return reply.get("count_down", {}).get("delete_all_rules", {}).get("err_code", 1) == 0

    def off_timer_remaining(self) -> Optional[int]:
        """Seconds left on an armed countdown, or None."""
        reply = self.query({"count_down": {"get_rules": {}}})
        for r in reply.get("count_down", {}).get("get_rules", {}).get("rule_list", []) or []:
            if r.get("enable") == 1:
                return int(r.get("remain", 0))
        return None
