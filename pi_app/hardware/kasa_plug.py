"""TP-Link Kasa smart plug over the local network (the charger plug).

Speaks the original Kasa protocol: JSON over TCP port 9999, obscured with
the XOR autokey cipher (seed 171), each frame prefixed with its 4-byte
big-endian length. No cloud, no credentials, no third-party library. The
EP10 on WALL-E's charger answers this protocol (docs/charger_plug.md).

Every call opens one short connection and closes it. A failure raises
``PlugError``; the caller (ChargeManager) treats that as "plug unknown" and
never as a reason to drive anything.
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Any, Dict, Optional

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


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise PlugError("connection closed mid-frame")
        buf.extend(chunk)
    return bytes(buf)


class KasaLegacyPlug:
    """One Kasa plug at a fixed address."""

    def __init__(self, host: str, port: int = 9999, timeout_s: float = 3.0) -> None:
        if not host:
            raise ValueError("Kasa plug host is empty")
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    # -- transport --------------------------------------------------------

    def query(self, request: Dict[str, Any]) -> Dict[str, Any]:
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

    # -- plug API ---------------------------------------------------------

    def get_sysinfo(self) -> Dict[str, Any]:
        reply = self.query({"system": {"get_sysinfo": {}}})
        info = reply.get("system", {}).get("get_sysinfo")
        if not isinstance(info, dict):
            raise PlugError(f"{self.host}: no get_sysinfo in reply")
        if info.get("err_code", 0) != 0:
            raise PlugError(f"{self.host}: get_sysinfo err_code {info.get('err_code')}")
        return info

    def get_state(self) -> Optional[bool]:
        """True = relay closed (charger powered), False = open."""
        info = self.get_sysinfo()
        relay = info.get("relay_state")
        if relay is None:
            # Multi-outlet strips report children; the EP10 never does.
            raise PlugError(f"{self.host}: no relay_state in sysinfo")
        return bool(int(relay))

    def set_state(self, on: bool) -> bool:
        reply = self.query({"system": {"set_relay_state": {"state": 1 if on else 0}}})
        res = reply.get("system", {}).get("set_relay_state")
        if not isinstance(res, dict):
            raise PlugError(f"{self.host}: no set_relay_state in reply")
        if res.get("err_code", 1) != 0:
            raise PlugError(f"{self.host}: set_relay_state err_code {res.get('err_code')}")
        return True
